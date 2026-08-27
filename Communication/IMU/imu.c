/**
 * @file    imu.c
 * @brief   IMU 姿态传感器串口模块实现(Yesense 固定协议,FreeRTOS Queue 版)。
 *
 * 数据流:
 *   接收: UART DMA/IDLE → Imu_UartRxEvent() → 环形缓冲
 *        → Imu_RxTask 解析 → RX Queue(覆盖式,只留最新) → 业务 Imu_Receive()
 *
 * 并发原语(与蓝牙模块保持一致):
 *   原始字节:环形缓冲(中断写,任务读,复用 Algorithm/ring_buffer);
 *   中断→接收任务的"唤醒":任务通知(ulTaskNotifyTake/vTaskNotifyGiveFromISR);
 *   业务数据:FreeRTOS 基础 Queue(xQueueCreate/xQueueSend/Receive)。
 *
 * 中断里只做"搬数据 + 通知",所有解析(找包头/校验/拆字段)都在任务里。
 * 模块不碰 UART 外设:DMA 缓冲与接收启停由 main.c / IQRhandler.c 统一管理。
 */
#include "imu.h"

#include "includes.h" /* __RAM_D2_ / ALIGN_32B 等 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "ring_buffer.h" /* 复用现有字节环形缓冲 */

/* ------------------------------------------------------------------ */
/* 参数宏:协议与缓冲配置(编译期定死,与 IMU 输出约定一致)                   */
/* ------------------------------------------------------------------ */
#define IMU_YS_I32_SCALE          0.000001f
#define IMU_YS_RAW_MAG_SCALE      0.001f
#define IMU_YS_DATA_LEN_3I32      12U
#define IMU_YS_DATA_LEN_4I32      16U

#define IMU_RX_RING_SIZE 256U           /* 原始字节环形缓冲大小 */

/* ------------------------------------------------------------------ */
/* 静态变量                                                            */
/* ------------------------------------------------------------------ */
typedef struct
{
    float accel[3];
    float gyro[3];
    float mag[3];
    float raw_mag[3];
    float euler[3];
    float quaternion[4];
    float euler_zero[3];
    bool euler_zero_inited;
    uint32_t frame_count;
} ImuYesenseCache_t;

static QueueHandle_t s_imu_rx_queue;        /* 业务层接收队列(最新一帧) */
static TaskHandle_t s_imu_rx_task_handle;          /* 接收任务句柄(模块内部创建) */
static RingBuffer_t s_imu_ring;             /* 原始字节环形缓冲(中断写,任务读) */
static uint8_t s_imu_ring_buf[IMU_RX_RING_SIZE];

static ImuRxBuffer_t s_imu_rx = {0};        /* 正在拼接的帧(只在接收任务里访问) */
static ImuYesenseCache_t s_imu_cache = {0}; /* 协议字段缓存(跨帧保持) */
static uint32_t s_imu_last_frame_tick = 0U; /* 最后一帧成功解析的 tick(0 = 从未收到) */
static bool s_imu_init_done = false;

/* ------------------------------------------------------------------ */
/* 接收:解析函数(只在接收任务里调用)                                      */
/* ------------------------------------------------------------------ */
static int32_t Imu_ReadI32LE(const uint8_t *data)
{
    return (int32_t)(((uint32_t)data[3] << 24U) |
                     ((uint32_t)data[2] << 16U) |
                     ((uint32_t)data[1] << 8U) |
                     (uint32_t)data[0]);
}

/* Yesense 双字节校验:ck_a = 逐字节和,ck_b = 逐字节累计和 */
static bool Imu_CalcChecksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    if ((data == NULL) || (len == 0U) || (ck_a == NULL) || (ck_b == NULL))
    {
        return false;
    }

    uint8_t check_a = 0U;
    uint8_t check_b = 0U;

    for (uint16_t i = 0U; i < len; ++i)
    {
        check_a = (uint8_t)(check_a + data[i]);
        check_b = (uint8_t)(check_b + check_a);
    }

    *ck_a = check_a;
    *ck_b = check_b;
    return true;
}

/* 按 data id 解析一段 payload,结果写进跨帧缓存,返回本段解析出的字段标志 */
static bool Imu_ParsePayloadById(uint8_t id,
                                 uint8_t len,
                                 const uint8_t *data,
                                 ImuYesenseCache_t *cache,
                                 uint16_t *parsed_flag)
{
    if ((data == NULL) || (cache == NULL) || (parsed_flag == NULL))
    {
        return false;
    }

    *parsed_flag = IMU_DATA_VALID_NONE;

    switch (id)
    {
#if (IMU_YS_OUT_ACCEL == 1U)
    case IMU_YS_ID_ACCEL:
        if (len != IMU_YS_DATA_LEN_3I32)
        {
            return false;
        }
        cache->accel[0] = (float)Imu_ReadI32LE(data) * IMU_YS_I32_SCALE;
        cache->accel[1] = (float)Imu_ReadI32LE(data + 4U) * IMU_YS_I32_SCALE;
        cache->accel[2] = (float)Imu_ReadI32LE(data + 8U) * IMU_YS_I32_SCALE;
        *parsed_flag = IMU_DATA_VALID_ACCEL;
        return true;
#endif

    case IMU_YS_ID_GYRO:
        if (len != IMU_YS_DATA_LEN_3I32)
        {
            return false;
        }
        cache->gyro[0] = (float)Imu_ReadI32LE(data) * IMU_YS_I32_SCALE;
        cache->gyro[1] = (float)Imu_ReadI32LE(data + 4U) * IMU_YS_I32_SCALE;
        cache->gyro[2] = (float)Imu_ReadI32LE(data + 8U) * IMU_YS_I32_SCALE;
        *parsed_flag = IMU_DATA_VALID_GYRO;
        return true;

#if (IMU_YS_OUT_MAG == 1U)
    case IMU_YS_ID_MAG:
        if (len != IMU_YS_DATA_LEN_3I32)
        {
            return false;
        }
        cache->mag[0] = (float)Imu_ReadI32LE(data) * IMU_YS_I32_SCALE;
        cache->mag[1] = (float)Imu_ReadI32LE(data + 4U) * IMU_YS_I32_SCALE;
        cache->mag[2] = (float)Imu_ReadI32LE(data + 8U) * IMU_YS_I32_SCALE;
        *parsed_flag = IMU_DATA_VALID_MAG;
        return true;
#endif

#if (IMU_YS_OUT_RAW_MAG == 1U)
    case IMU_YS_ID_RAW_MAG:
        if (len != IMU_YS_DATA_LEN_3I32)
        {
            return false;
        }
        cache->raw_mag[0] = (float)Imu_ReadI32LE(data) * IMU_YS_RAW_MAG_SCALE;
        cache->raw_mag[1] = (float)Imu_ReadI32LE(data + 4U) * IMU_YS_RAW_MAG_SCALE;
        cache->raw_mag[2] = (float)Imu_ReadI32LE(data + 8U) * IMU_YS_RAW_MAG_SCALE;
        *parsed_flag = IMU_DATA_VALID_RAW_MAG;
        return true;
#endif

    case IMU_YS_ID_EULER:
        if (len != IMU_YS_DATA_LEN_3I32)
        {
            return false;
        }
        {
            /* Yesense示例顺序是 pitch, roll, yaw;对外统一成 roll, pitch, yaw。 */
            const float pitch = (float)Imu_ReadI32LE(data) * IMU_YS_I32_SCALE;
            const float roll = (float)Imu_ReadI32LE(data + 4U) * IMU_YS_I32_SCALE;
            const float yaw = (float)Imu_ReadI32LE(data + 8U) * IMU_YS_I32_SCALE;

            if (!cache->euler_zero_inited)
            {
                cache->euler_zero[0] = roll;
                cache->euler_zero[1] = pitch;
                cache->euler_zero[2] = yaw;
                cache->euler_zero_inited = true;
            }

            cache->euler[0] = roll - cache->euler_zero[0];
            cache->euler[1] = pitch - cache->euler_zero[1];
            float yaw_diff = yaw - cache->euler_zero[2];
            // wrap 到 [-180, 180], 消除 IMU 输出范围边界跨零跳变
            if (yaw_diff > 180.0f)      yaw_diff -= 360.0f;
            else if (yaw_diff < -180.0f) yaw_diff += 360.0f;
            cache->euler[2] = yaw_diff;
        }
        *parsed_flag = IMU_DATA_VALID_EULER;
        return true;

#if (IMU_YS_OUT_QUATERNION == 1U)
    case IMU_YS_ID_QUATERNION:
        if (len != IMU_YS_DATA_LEN_4I32)
        {
            return false;
        }
        cache->quaternion[0] = (float)Imu_ReadI32LE(data) * IMU_YS_I32_SCALE;
        cache->quaternion[1] = (float)Imu_ReadI32LE(data + 4U) * IMU_YS_I32_SCALE;
        cache->quaternion[2] = (float)Imu_ReadI32LE(data + 8U) * IMU_YS_I32_SCALE;
        cache->quaternion[3] = (float)Imu_ReadI32LE(data + 12U) * IMU_YS_I32_SCALE;
        *parsed_flag = IMU_DATA_VALID_QUATERNION;
        return true;
#endif

    default:
        return false;
    }
}

/* 整帧校验 + 拆 payload,输出一帧业务数据(失败返回 false,不产生输出) */
static bool Imu_ParseYesenseFrame(const uint8_t *frame, uint16_t frame_len, ImuData_t *out_data)
{
    if ((frame == NULL) || (out_data == NULL))
    {
        return false;
    }

    const uint8_t payload_len = frame[4];
    const uint16_t tid = (uint16_t)(((uint16_t)frame[3] << 8U) | (uint16_t)frame[2]);
    const uint16_t expected_len = (uint16_t)(IMU_YS_MIN_FRAME_LEN + payload_len);
    if (frame_len != expected_len)
    {
        return false;
    }

    uint8_t ck_a = 0U;
    uint8_t ck_b = 0U;
    if (!Imu_CalcChecksum(&frame[2], (uint16_t)(payload_len + 3U), &ck_a, &ck_b))
    {
        return false;
    }

    if ((ck_a != frame[(uint16_t)(5U + payload_len)]) ||
        (ck_b != frame[(uint16_t)(6U + payload_len)]))
    {
        return false;
    }

    uint16_t pos = 5U;
    uint16_t remaining = payload_len;
    bool has_valid_payload = false;
    uint16_t valid_flags = IMU_DATA_VALID_NONE;

    while (remaining > 0U)
    {
        if (remaining < 2U)
        {
            break;
        }

        const uint8_t data_id = frame[pos];
        const uint8_t data_len = frame[(uint16_t)(pos + 1U)];

        if (((uint16_t)data_len + 2U) <= remaining)
        {
            uint16_t parsed_flag = IMU_DATA_VALID_NONE;
            if (Imu_ParsePayloadById(data_id,
                                     data_len,
                                     &frame[(uint16_t)(pos + 2U)],
                                     &s_imu_cache,
                                     &parsed_flag))
            {
                pos = (uint16_t)(pos + data_len + 2U);
                remaining = (uint16_t)(remaining - data_len - 2U);
                has_valid_payload = true;
                valid_flags = (uint16_t)(valid_flags | parsed_flag);
            }
            else
            {
                ++pos;
                --remaining;
            }
        }
        else
        {
            ++pos;
            --remaining;
        }
    }

    if (!has_valid_payload)
    {
        return false;
    }

    memset(out_data, 0, sizeof(*out_data));

    out_data->body_orientation[0] = s_imu_cache.euler[0];
    out_data->body_orientation[1] = s_imu_cache.euler[1];
    out_data->body_orientation[2] = s_imu_cache.euler[2];

    out_data->body_rate[0] = s_imu_cache.gyro[0];
    out_data->body_rate[1] = s_imu_cache.gyro[1];
    out_data->body_rate[2] = s_imu_cache.gyro[2];

#if (IMU_YS_OUT_ACCEL == 1U)
    out_data->body_accel[0] = s_imu_cache.accel[0];
    out_data->body_accel[1] = s_imu_cache.accel[1];
    out_data->body_accel[2] = s_imu_cache.accel[2];
#endif

#if (IMU_YS_OUT_MAG == 1U)
    out_data->body_mag[0] = s_imu_cache.mag[0];
    out_data->body_mag[1] = s_imu_cache.mag[1];
    out_data->body_mag[2] = s_imu_cache.mag[2];
#endif

#if (IMU_YS_OUT_RAW_MAG == 1U)
    out_data->body_raw_mag[0] = s_imu_cache.raw_mag[0];
    out_data->body_raw_mag[1] = s_imu_cache.raw_mag[1];
    out_data->body_raw_mag[2] = s_imu_cache.raw_mag[2];
#endif

#if (IMU_YS_OUT_QUATERNION == 1U)
    out_data->body_quaternion[0] = s_imu_cache.quaternion[0];
    out_data->body_quaternion[1] = s_imu_cache.quaternion[1];
    out_data->body_quaternion[2] = s_imu_cache.quaternion[2];
    out_data->body_quaternion[3] = s_imu_cache.quaternion[3];
#endif

    out_data->valid_flags = valid_flags;
    out_data->last_tid = tid;
    ++s_imu_cache.frame_count;
    out_data->frame_count = s_imu_cache.frame_count;

    return true;
}

/* 校验通过的一帧 → 覆盖式入队(永远保留最新一帧,队列满时旧帧被覆盖) */
static void Imu_PublishFrame(const uint8_t *frame, uint16_t frame_len)
{
    ImuData_t parsed = {0};

    if (!Imu_ParseYesenseFrame(frame, frame_len, &parsed))
    {
        return;
    }

    s_imu_last_frame_tick = xTaskGetTickCount();
    (void)xQueueSend(s_imu_rx_queue, &parsed, 0U);
}

static void Imu_ResetParser(ImuRxBuffer_t *rx)
{
    if (rx == NULL)
    {
        return;
    }

    rx->parser_state = IMU_WAIT_HEADER1;
    rx->index = 0U;
    rx->remain_bytes = 0U;
}

/* 逐字节拼帧状态机:找 0x59 0x53 包头 → 读 meta(定出 payload 长度)
   → 收满 payload + 校验 → 整帧校验解析。帧失败后回到找包头状态自动重新同步。 */
static void Imu_RxByte(uint8_t b)
{
    switch (s_imu_rx.parser_state)
    {
    case IMU_WAIT_HEADER1:
        if (b == IMU_YS_HEADER1)
        {
            s_imu_rx.frame[0] = b;
            s_imu_rx.index = 1U;
            s_imu_rx.parser_state = IMU_WAIT_HEADER2;
        }
        break;

    case IMU_WAIT_HEADER2:
        if (b == IMU_YS_HEADER2)
        {
            s_imu_rx.frame[1] = b;
            s_imu_rx.index = 2U;
            s_imu_rx.remain_bytes = 3U; /* tid(2) + len(1) */
            s_imu_rx.parser_state = IMU_READ_META;
        }
        else if (b == IMU_YS_HEADER1)
        {
            s_imu_rx.frame[0] = b;
            s_imu_rx.index = 1U;
        }
        else
        {
            Imu_ResetParser(&s_imu_rx);
        }
        break;

    case IMU_READ_META:
        if (s_imu_rx.index >= IMU_YS_MAX_FRAME_LEN)
        {
            Imu_ResetParser(&s_imu_rx);
            break;
        }

        s_imu_rx.frame[s_imu_rx.index++] = b;
        if (--s_imu_rx.remain_bytes == 0U)
        {
            const uint8_t payload_len = s_imu_rx.frame[4];
            const uint16_t total_len = (uint16_t)(IMU_YS_MIN_FRAME_LEN + payload_len);
            if ((payload_len > IMU_YS_MAX_PAYLOAD_LEN) || (total_len > IMU_YS_MAX_FRAME_LEN))
            {
                Imu_ResetParser(&s_imu_rx);
            }
            else
            {
                s_imu_rx.remain_bytes = (uint16_t)(payload_len + 2U); /* payload + ck_a + ck_b */
                s_imu_rx.parser_state = IMU_READ_PAYLOAD_AND_CRC;
            }
        }
        break;

    case IMU_READ_PAYLOAD_AND_CRC:
        if (s_imu_rx.index >= IMU_YS_MAX_FRAME_LEN)
        {
            Imu_ResetParser(&s_imu_rx);
            break;
        }

        s_imu_rx.frame[s_imu_rx.index++] = b;
        if (--s_imu_rx.remain_bytes == 0U)
        {
            Imu_PublishFrame(s_imu_rx.frame, s_imu_rx.index);
            Imu_ResetParser(&s_imu_rx);
        }
        break;

    default:
        Imu_ResetParser(&s_imu_rx);
        break;
    }
}

/* ------------------------------------------------------------------ */
/* 接收:任务与中断入口                                                  */
/* ------------------------------------------------------------------ */
/* 接收任务:阻塞在唤醒信号量上,醒来后把环形缓冲里的字节全部喂给解析器 */
static void Imu_RxTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        /* 等"有数据"信号(计数信号量,每收到一段 DMA 数据 Give 一次) */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 一次取完缓冲里所有字节(中断里可能已塞了不止一段) */
        uint8_t b;
        while (RingBuffer_Pop(&s_imu_ring, &b))
        {
            Imu_RxByte(b);
        }
    }
}

/* UART IDLE 中断入口:只搬运 + 发信号,不做任何解析 */
void Imu_UartRxEvent(const uint8_t *data, uint16_t size)
{
    /* 1. 把 DMA 收到的原始字节搬进环形缓冲(段吞吐,满了就丢,不阻塞中断) */
    RingBuffer_PushSegment(&s_imu_ring, data, size);

    /* 2. Give 一个信号把接收任务唤醒;
           真正的数据在环形缓冲里,任务醒来后自己取 */
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_imu_rx_task_handle, &woken);
    if (woken == pdTRUE)
    {
        portYIELD_FROM_ISR(woken);
    }
}

/* ------------------------------------------------------------------ */
/* 对外 API                                                            */
/* ------------------------------------------------------------------ */
void Imu_Init(void)
{
    if (s_imu_init_done)
    {
        return;
    }

    /* 业务接收队列:队列里存完整 ImuData_t(按值拷贝);
       深度 2  */
    s_imu_rx_queue = xQueueCreate(IMU_RX_QUEUE_SIZE, sizeof(ImuData_t));

    /* 接收侧:任务通知 + 原始字节环形缓冲 */
    RingBuffer_Init(&s_imu_ring, s_imu_ring_buf, sizeof(s_imu_ring_buf));

    Imu_ResetParser(&s_imu_rx);
    memset(&s_imu_cache, 0, sizeof(s_imu_cache));

    /* 接收任务在模块内部创建,方便整个模块整体搬走 */
    xTaskCreate(Imu_RxTask, "ImuRxTask", IMU_TASK_STACK_WORDS, NULL,
                IMU_RX_TASK_PRIORITY, &s_imu_rx_task_handle);

    /* 注意:USART3 的 RX DMA 接收由 main.c 用 UART3_RxBuffer 启动、
       IQRhandler.c 的空闲/错误回调里重启(与其它串口一致),这里不再配 DMA。
       Imu_Init() 必须先于 DMA 启动调用,否则中断回调可能访问空信号量。 */
    s_imu_init_done = true;
}

/* 业务层接收:从 RX Queue 取最新一帧,超时返回 false(timeout_ms=0 时非阻塞) */
bool Imu_Receive(ImuData_t *data, uint32_t timeout_ms)
{
    if (data == NULL)
    {
        return false;
    }
    return (xQueueReceive(s_imu_rx_queue, data, pdMS_TO_TICKS(timeout_ms)) == pdPASS);
}

/* 获取最后一次成功解析 IMU 帧的系统 tick(0 表示从未收到) */
uint32_t Imu_GetLastFrameTick(void)
{
    return s_imu_last_frame_tick;
}
