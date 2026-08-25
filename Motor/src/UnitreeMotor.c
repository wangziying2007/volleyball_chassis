/**
 * @file    UnitreeMotor.c
 * @brief   Unitree GO-M8010-6 RS485 motor driver.
 *
 * Ported from the 4-bus sample to the current single-UART7 RS485 hardware.
 * UART7 is initialized with HAL_RS485Ex_Init(), so DE direction is handled by
 * the peripheral; no manual GPIO toggling is required.
 */
#include "includes.h"
#include "UnitreeMotor.h"
#include "usart.h"
#include "crc_ccitt.h"

#if USE_UNITREE

#define UNITREE_2PI 6.28318f
#define UNITREE_SATURATE(_IN, _MIN, _MAX) \
    {                                     \
        if ((_IN) <= (_MIN))              \
            (_IN) = (_MIN);               \
        else if ((_IN) >= (_MAX))         \
            (_IN) = (_MAX);               \
    }

UnitreeMotor Unitree_motors[UNITREE_MOTOR_NUM];
__RAM_D1_ ALIGN_32B uint8_t Unitree_UART7_RxBuffer[UNITREE_RX_BUFFER_SIZE] = {0};

static bool s_unitree_initialized = false;
static uint8_t s_unitree_tx_index = 0U;
static UnitreeMotorData_t s_unitree_rx_data;
static __RAM_D1_ ALIGN_32B UnitreeMotorCmd_t s_unitree_tx_cmd;

static UART_HandleTypeDef *UnitreeMotor_GetUart(void)
{
#if (MOTOR_UNITREE_UART == 4)
    return &huart4;
#elif (MOTOR_UNITREE_UART == 7)
    return &huart7;
#elif (MOTOR_UNITREE_UART == 9)
    return &huart9;
#else
    return NULL;
#endif
}
static inline void Unitree_SetRxMode(void)
{
    UART_HandleTypeDef *uart = UnitreeMotor_GetUart();

    if (uart == NULL)
    {
        return;
    }

    HAL_GPIO_WritePin(RS_485_U7_GPIO_Port, RS_485_U7_Pin, GPIO_PIN_RESET);
}

static inline void Unitree_SetTxMode(void)
{
    UART_HandleTypeDef *uart = UnitreeMotor_GetUart();

    if (uart == NULL)
    {
        return;
    }

    HAL_GPIO_WritePin(RS_485_U7_GPIO_Port, RS_485_U7_Pin, GPIO_PIN_SET);
}

void UnitreeMotor_Init(void)
{
    UnitreeMotorCmd_t default_cmd;

    memset(&default_cmd, 0, sizeof(default_cmd));
    default_cmd.mode = UNITREE_MOTOR_MODE_IDLE;
    default_cmd.kp = MOTOR_UNITREE_DEFAULT_KP;
    default_cmd.kd = MOTOR_UNITREE_DEFAULT_KW;

    for (uint32_t i = 0U; i < UNITREE_MOTOR_NUM; i++)
    {
        memset(&Unitree_motors[i], 0, sizeof(Unitree_motors[i]));
        Unitree_motors[i].cmd = default_cmd;
        Unitree_motors[i].cmd.id = (uint16_t)i;
        Unitree_motors[i].data.id = (uint8_t)i;
        Unitree_motors[i].enable = false;
        Unitree_motors[i].set_zero = false;
        Unitree_motors[i].zero_offset = 0.0f;
        Unitree_motors[i].begin = true;
    }

    s_unitree_tx_index = 0U;
    s_unitree_initialized = true;
    
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, Unitree_UART7_RxBuffer, UNITREE_RX_BUFFER_SIZE);
    Unitree_SetRxMode();
}

void UnitreeMotor_EncodeFrame(UnitreeMotorCmd_t *cmd)
{
    UnitreeMotorControlFrame_t *frame;
    uint8_t *packet;
    int16_t tor_des = 0;
    int16_t spd_des = 0;
    int32_t pos_des = 0;
    int16_t kp = 0;
    int16_t kd = 0;

    if (cmd == NULL)
    {
        return;
    }

    frame = &cmd->frame;
    packet = (uint8_t *)frame;

    UNITREE_SATURATE(cmd->id, 0, 15);
    UNITREE_SATURATE(cmd->mode, 0, 7);
    UNITREE_SATURATE(cmd->kp, 0.0f, 25.599f);
    UNITREE_SATURATE(cmd->kd, 0.0f, 25.599f);
    UNITREE_SATURATE(cmd->torque, -127.99f, 127.99f);
    UNITREE_SATURATE(cmd->speed, -804.00f, 804.00f);
    UNITREE_SATURATE(cmd->position, -411774.0f, 411774.0f);

    /* 协议实测为转子侧量:上层关节语义需在此做减速比换算 */
    tor_des = (int16_t)(cmd->torque * UNITREE_GEAR_RATIO * 256.0f);
    spd_des = (int16_t)((cmd->speed * UNITREE_GEAR_RATIO) / UNITREE_2PI * 256.0f);
    pos_des = (int32_t)((cmd->position * UNITREE_GEAR_RATIO) / UNITREE_2PI * 32768.0f);
    kp = (int16_t)(cmd->kp / 25.6f * 32768.0f);
    kd = (int16_t)(cmd->kd / 25.6f * 32768.0f);

    packet[0] = 0xFE;
    packet[1] = 0xEE;
    packet[2] = (uint8_t)((cmd->id & 0x0FU) | ((cmd->mode & 0x07U) << 4));
    packet[3] = (uint8_t)(tor_des & 0xFF);
    packet[4] = (uint8_t)((tor_des >> 8) & 0xFF);
    packet[5] = (uint8_t)(spd_des & 0xFF);
    packet[6] = (uint8_t)((spd_des >> 8) & 0xFF);
    packet[7] = (uint8_t)(pos_des & 0xFF);
    packet[8] = (uint8_t)((pos_des >> 8) & 0xFF);
    packet[9] = (uint8_t)((pos_des >> 16) & 0xFF);
    packet[10] = (uint8_t)((pos_des >> 24) & 0xFF);
    packet[11] = (uint8_t)(kp & 0xFF);
    packet[12] = (uint8_t)((kp >> 8) & 0xFF);
    packet[13] = (uint8_t)(kd & 0xFF);
    packet[14] = (uint8_t)((kd >> 8) & 0xFF);

    {
        uint16_t crc = crc_ccitt(0, packet,
                                 sizeof(*frame) - sizeof(frame->crc16));
        packet[15] = (uint8_t)(crc & 0xFF);
        packet[16] = (uint8_t)((crc >> 8) & 0xFF);
    }
}

void UnitreeMotor_DecodeFrame(UnitreeMotorData_t *data)
{
    const UnitreeMotorFeedbackFrame_t *frame;
    const uint8_t *packet;
    uint16_t rx_crc = 0U;
    uint16_t status_force = 0U;
    int16_t speed = 0;
    int16_t torque = 0;
    int32_t pos = 0;

    if (data == NULL)
    {
        return;
    }

    frame = &data->frame;
    packet = (const uint8_t *)frame;

    if (packet[0] != 0xFD || packet[1] != 0xEE)
    {
        data->correct = 0;
        return;
    }

    status_force = (uint16_t)packet[12] | ((uint16_t)packet[13] << 8U);

    data->calc_crc = crc_ccitt(0, packet,
                               sizeof(*frame) - sizeof(frame->crc16));
    rx_crc = (uint16_t)packet[14] | ((uint16_t)packet[15] << 8U);

    if (rx_crc != data->calc_crc)
    {
        memset(&data->frame, 0, sizeof(data->frame));
        data->correct = 0;
        data->bad_msg++;
        return;
    }

    data->id = packet[2] & 0x0FU;
    data->mode = (packet[2] >> 4) & 0x07U;
    data->temperature = (int)(int8_t)packet[11];
    data->error = (int)(status_force & 0x0007U);

    torque = (int16_t)((uint16_t)packet[3] | ((uint16_t)packet[4] << 8U));
    speed = (int16_t)((uint16_t)packet[5] | ((uint16_t)packet[6] << 8U));
    pos = (int32_t)((uint32_t)packet[7] |
                    ((uint32_t)packet[8] << 8U) |
                    ((uint32_t)packet[9] << 16U) |
                    ((uint32_t)packet[10] << 24U));

    /* 协议回包实测为转子侧量:转换到上层统一的关节语义 */
    data->torque = ((float)torque) / 256.0f * UNITREE_GEAR_RATIO;
    data->speed = (((float)speed / 256.0f) * UNITREE_2PI) / UNITREE_GEAR_RATIO;
    data->position = (UNITREE_2PI * ((float)pos) / 32768.0f) / UNITREE_GEAR_RATIO;
    data->foot_force = (int)((status_force >> 3U) & 0x0FFFU);
    data->correct = 1;
}

void UnitreeMotor_UART_RxHandler(const uint8_t *data, uint16_t size)
{
    UnitreeMotorData_t *rx_data = &s_unitree_rx_data;

    if (data == NULL || size != sizeof(UnitreeMotorFeedbackFrame_t))
    {
        return;
    }

    memset(rx_data, 0, sizeof(*rx_data));
    memcpy(&rx_data->frame, data, size);
    UnitreeMotor_DecodeFrame(rx_data);

    if (rx_data->correct != 1 || rx_data->id >= UNITREE_MOTOR_NUM)
    {
        return;
    }

    {
        uint32_t bad_msg = Unitree_motors[rx_data->id].data.bad_msg;
        Unitree_motors[rx_data->id].data = *rx_data;
        Unitree_motors[rx_data->id].data.bad_msg = bad_msg;
        Unitree_motors[rx_data->id].data.position -=
            Unitree_motors[rx_data->id].zero_offset;
    }
}

void UnitreeMotor_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == NULL || huart != UnitreeMotor_GetUart())
    {
        return;
    }
    Unitree_SetRxMode();
}

static void UnitreeMotor_SendCommand(UnitreeMotor *motor)
{
    UART_HandleTypeDef *uart = UnitreeMotor_GetUart();

    if (motor == NULL || uart == NULL || !motor->begin)
    {
        return;
    }

    /* UART7 的 RX DMA 长期处于接收状态,因此用 gState(TX 状态)判断是否可发送 */
    if (uart->gState != HAL_UART_STATE_READY)
    {
        return;
    }
    Unitree_SetTxMode();

    s_unitree_tx_cmd = motor->cmd;
    s_unitree_tx_cmd.position += motor->zero_offset; /* 用户目标 + 零位偏移 */
    UnitreeMotor_EncodeFrame(&s_unitree_tx_cmd);

    SCB_CleanDCache_by_Addr((uint32_t *)&s_unitree_tx_cmd.frame,
                            sizeof(s_unitree_tx_cmd.frame));
    HAL_UART_Transmit_DMA(uart, (uint8_t *)&s_unitree_tx_cmd.frame,
                          sizeof(s_unitree_tx_cmd.frame));
}

static void UnitreeMotor_SetZero(int motor_id)
{
    if (motor_id < 0 || motor_id >= UNITREE_MOTOR_NUM)
    {
        for (uint32_t i = 0U; i < UNITREE_MOTOR_NUM; i++)
        {
            Unitree_motors[i].zero_offset += Unitree_motors[i].data.position;
            Unitree_motors[i].cmd.position = 0.0f;
            Unitree_motors[i].data.position = 0.0f;
        }
        return;
    }

    Unitree_motors[motor_id].zero_offset += Unitree_motors[motor_id].data.position;
    Unitree_motors[motor_id].cmd.position = 0.0f;
    Unitree_motors[motor_id].data.position = 0.0f;
}

static void UnitreeMotor_Enable(int motor_id)
{
    if (motor_id < 0 || motor_id >= UNITREE_MOTOR_NUM)
    {
        for (uint32_t i = 0U; i < UNITREE_MOTOR_NUM; i++)
        {
            Unitree_motors[i].enable = true;
        }
        return;
    }

    Unitree_motors[motor_id].enable = true;
}

static void UnitreeMotor_Disable(int motor_id)
{
    if (motor_id < 0 || motor_id >= UNITREE_MOTOR_NUM)
    {
        for (uint32_t i = 0U; i < UNITREE_MOTOR_NUM; i++)
        {
            Unitree_motors[i].enable = false;
        }
        return;
    }

    Unitree_motors[motor_id].enable = false;
}

void UnitreeMotor_Func(void)
{
    if (!s_unitree_initialized)
    {
        return;
    }

    for (uint32_t i = 0U; i < UNITREE_MOTOR_NUM; i++)
    {
        UnitreeMotor *motor = &Unitree_motors[i];

        if (motor->enable)
        {
            motor->cmd.mode = UNITREE_MOTOR_MODE_FOC;
        }
        else
        {
            motor->cmd.mode = UNITREE_MOTOR_MODE_IDLE;
        }

        if (motor->set_zero)
        {
            UnitreeMotor_SetZero((int)i);
            motor->set_zero = false;
        }
    }

    UnitreeMotor_SendCommand(&Unitree_motors[s_unitree_tx_index]);

    s_unitree_tx_index++;
    if (s_unitree_tx_index >= UNITREE_MOTOR_NUM)
    {
        s_unitree_tx_index = 0U;
    }
}

#endif /* USE_UNITREE */
