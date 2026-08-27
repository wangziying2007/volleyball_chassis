/**
 * @file    imu.h
 * @brief   IMU 姿态传感器串口模块(Yesense 协议,FreeRTOS Queue 版)。
 *
 * 职责分层:
 *   UART 层  : 只负责 DMA 收发与 IDLE 中断(main.c 启动接收 + IQRhandler.c 回调)
 *   本模块   : Yesense 协议帧解析(协议固定,编译期定死)、RX Queue、接收任务
 *   业务层   : 只调用 Imu_Init() / Imu_Receive() / Imu_GetLastFrameTick()
 *
 * 业务层不需要知道 UART、DMA、IDLE、校验、缓冲区、解析器的存在。
 * 模块只收不发:IMU 姿态数据是单向流(Yesense 0x59 0x53 帧)。
 *
 * 用法(在业务任务里):
 *   // 取最新一帧姿态(超时返回 false;timeout_ms = 0 非阻塞)
 *   ImuData_t data;
 *   if (Imu_Receive(&data, 100)) {
 *       // data.body_orientation[0..2] = roll/pitch/yaw(相对上电零位, deg)
 *       // data.body_rate[0..2]       = 角速度(deg/s)
 *   }
 */
#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

#include "app_config.h" /* APP_PRIO_* 任务优先级 */

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* 配置(软件配置,按需修改;协议固定,与 IMU 输出约定一致)                    */
/* ------------------------------------------------------------------ */
#define IMU_RX_QUEUE_SIZE       2U      /* 业务层接收队列深度(帧数);队列满时丢新帧 */
#define IMU_TASK_STACK_WORDS    128U    /* 接收任务栈大小(字) */
#define IMU_RX_TASK_PRIORITY    APP_PRIO_NORMAL + 1 /* 接收任务优先级 */

/* ============ Yesense 协议配置(固定,勿改) ============ */
#define IMU_YS_HEADER1           0x59U
#define IMU_YS_HEADER2           0x53U
#define IMU_YS_MIN_FRAME_LEN     7U  /* header(2) + tid(2) + len(1) + ck_a(1) + ck_b(1) */
#define IMU_YS_MAX_PAYLOAD_LEN   255U
#define IMU_YS_MAX_FRAME_LEN     (IMU_YS_MIN_FRAME_LEN + IMU_YS_MAX_PAYLOAD_LEN)

/* payload data id */
#define IMU_YS_ID_ACCEL          0x10U
#define IMU_YS_ID_GYRO           0x20U
#define IMU_YS_ID_MAG            0x30U
#define IMU_YS_ID_RAW_MAG        0x31U
#define IMU_YS_ID_EULER          0x40U
#define IMU_YS_ID_QUATERNION     0x41U

/* ============ 输出字段配置(只需改这里) ============ */
/* LINK_TEST: 最小链路验证(姿态 + 角速度 + 有效标志 + 帧计数) */
/* MOTION_BASIC: 在LINK_TEST基础上增加加速度 */
/* FULL: 输出当前已支持的全部字段 */
#define IMU_YS_PROFILE_LINK_TEST    1U
#define IMU_YS_PROFILE_MOTION_BASIC 2U
#define IMU_YS_PROFILE_FULL         3U

#ifndef IMU_YS_OUTPUT_PROFILE
#define IMU_YS_OUTPUT_PROFILE IMU_YS_PROFILE_MOTION_BASIC
#endif

#if (IMU_YS_OUTPUT_PROFILE == IMU_YS_PROFILE_LINK_TEST)
#define IMU_YS_OUT_ACCEL_DEFAULT      0U
#define IMU_YS_OUT_MAG_DEFAULT        0U
#define IMU_YS_OUT_RAW_MAG_DEFAULT    0U
#define IMU_YS_OUT_QUATERNION_DEFAULT 0U
#elif (IMU_YS_OUTPUT_PROFILE == IMU_YS_PROFILE_MOTION_BASIC)
#define IMU_YS_OUT_ACCEL_DEFAULT      1U
#define IMU_YS_OUT_MAG_DEFAULT        0U
#define IMU_YS_OUT_RAW_MAG_DEFAULT    0U
#define IMU_YS_OUT_QUATERNION_DEFAULT 1U
#elif (IMU_YS_OUTPUT_PROFILE == IMU_YS_PROFILE_FULL)
#define IMU_YS_OUT_ACCEL_DEFAULT      1U
#define IMU_YS_OUT_MAG_DEFAULT        1U
#define IMU_YS_OUT_RAW_MAG_DEFAULT    1U
#define IMU_YS_OUT_QUATERNION_DEFAULT 1U
#else
#error "Invalid IMU_YS_OUTPUT_PROFILE"
#endif

/* 可选:按字段覆盖profile默认值(0=关闭,1=开启) */
#ifndef IMU_YS_OUT_ACCEL
#define IMU_YS_OUT_ACCEL IMU_YS_OUT_ACCEL_DEFAULT
#endif
#ifndef IMU_YS_OUT_MAG
#define IMU_YS_OUT_MAG IMU_YS_OUT_MAG_DEFAULT
#endif
#ifndef IMU_YS_OUT_RAW_MAG
#define IMU_YS_OUT_RAW_MAG IMU_YS_OUT_RAW_MAG_DEFAULT
#endif
#ifndef IMU_YS_OUT_QUATERNION
#define IMU_YS_OUT_QUATERNION IMU_YS_OUT_QUATERNION_DEFAULT
#endif

/* ------------------------------------------------------------------ */
/* 解析器状态机(模块内部使用,定义在头文件便于调试/复用)                     */
/* ------------------------------------------------------------------ */
typedef enum
{
    IMU_WAIT_HEADER1 = 0,
    IMU_WAIT_HEADER2,
    IMU_READ_META,            /* tid(2) + len(1) */
    IMU_READ_PAYLOAD_AND_CRC, /* payload(len) + ck_a + ck_b */
} ImuParserState_t;

typedef struct
{
    ImuParserState_t parser_state;
    uint8_t frame[IMU_YS_MAX_FRAME_LEN];
    uint16_t index;
    uint16_t remain_bytes;
} ImuRxBuffer_t;

/* ------------------------------------------------------------------ */
/* 业务数据结构:Queue 里存整个结构体(按值拷贝),业务层不用管内存/生命周期    */
/* ------------------------------------------------------------------ */
typedef enum
{
    IMU_DATA_VALID_NONE       = 0U,
    IMU_DATA_VALID_EULER      = (1U << 0),
    IMU_DATA_VALID_GYRO       = (1U << 1),
    IMU_DATA_VALID_ACCEL      = (1U << 2),
    IMU_DATA_VALID_MAG        = (1U << 3),
    IMU_DATA_VALID_RAW_MAG    = (1U << 4),
    IMU_DATA_VALID_QUATERNION = (1U << 5),
} ImuDataValidBits;

typedef struct
{
    float body_orientation[3];        // 姿态 (roll, pitch, yaw), 单位 deg
    float body_rate[3];               // 角速度 (wx, wy, wz), 单位 deg/s
#if (IMU_YS_OUT_ACCEL == 1U)
    float body_accel[3];              // 加速度 (ax, ay, az), 单位 m/s^2
#endif
#if (IMU_YS_OUT_MAG == 1U)
    float body_mag[3];                // 归一化磁场
#endif
#if (IMU_YS_OUT_RAW_MAG == 1U)
    float body_raw_mag[3];            // 原始磁场, 单位 mGauss
#endif
#if (IMU_YS_OUT_QUATERNION == 1U)
    float body_quaternion[4];         // 四元数 (w, x, y, z)
#endif
    uint16_t valid_flags;             // 本帧成功解析的字段标志(ImuDataValidBits)
    uint16_t last_tid;                // 本帧TID,便于验证链路连续性
    uint32_t frame_count;             // 成功解析帧计数
} ImuData_t;

/* ------------------------------------------------------------------ */
/* API(业务层只用这三个)                                                 */
/* ------------------------------------------------------------------ */
void Imu_Init(void);
bool Imu_Receive(ImuData_t *data, uint32_t timeout_ms);
uint32_t Imu_GetLastFrameTick(void);

/* 以下函数由 UART 中断回调调用(IQRhandler.c),业务层不要碰 */
void Imu_UartRxEvent(const uint8_t *data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
