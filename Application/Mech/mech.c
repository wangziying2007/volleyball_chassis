/**
 * @file    mech.c
 * @brief   机构主控侧业务层实现(接口见 mech.h)。
 *
 * 数据流(保留与电机一致的"任务写目标、ISR 发帧"解耦):
 *   Chassis_Task: Mech_PollButtons() → 按键边沿 → Mech_Stage() 暂存命令(op/dlc/data)
 *   TIM2 200Hz  : Mech_Func() → 组装扩展帧(0x01010500|op) → CAN_Enqueue(CAN1_Txqueue)
 *   FDCAN1 RX IRQ: Mech_IsOurs() 过滤 → Mech_OnFeedback() 解析 0x050101xx → 更新状态
 */
#include "mech.h"

#include "includes.h"
#include "FD_Canqueue.h"

static Mech_t s_mech = {0};

/* 把一条命令暂存为"待发"(任务上下文;忙时丢弃,人工按键节奏远小于 5ms) */
bool Mech_Stage(uint8_t op, uint8_t dlc, const uint8_t *data)
{
    if (s_mech.pending_new)
    {
        return false; /* 上一条还没发出,丢弃本次按键 */
    }
    if (dlc > 8U)
    {
        dlc = 8U;
    }

    s_mech.pending_op  = op;
    s_mech.pending_dlc = dlc;
    for (uint8_t i = 0U; i < dlc; i++)
    {
        s_mech.pending_data[i] = data[i];
    }
    s_mech.pending_new = true; /* 最后置标志,ISR 看到它才读数据 */
    return true;
}

/* 蓝牙按键集合 → 边沿检测 → 对应操作码(线程上下文,200Hz) */
void Mech_PollButtons(const MechButtons_t *btn)
{
    if (btn == NULL)
    {
        return;
    }

    if (btn->enable && !s_mech.last_buttons.enable)
    {
        const uint8_t d[2] = {1U, 0U};
        (void)Mech_Stage(MECH_OP_ENABLE, 2U, d);
    }
    if (btn->disable && !s_mech.last_buttons.disable)
    {
        const uint8_t d[2] = {0U, 0U};
        (void)Mech_Stage(MECH_OP_ENABLE, 2U, d);
    }
    if (btn->reset && !s_mech.last_buttons.reset)
    {
        const uint8_t d[2] = {0U, 0U}; /* R=0 双臂 */
        (void)Mech_Stage(MECH_OP_RESET, 2U, d);
    }
    if (btn->left_work1 && !s_mech.last_buttons.left_work1)
    {
        const uint8_t d[1] = {0U};
        (void)Mech_Stage(MECH_OP_LEFT_WORK_1, 1U, d);
    }
    if (btn->left_work2 && !s_mech.last_buttons.left_work2)
    {
        const uint8_t d[1] = {0U};
        (void)Mech_Stage(MECH_OP_LEFT_WORK_2, 1U, d);
    }
    if (btn->right_work1 && !s_mech.last_buttons.right_work1)
    {
        const uint8_t d[1] = {0U};
        (void)Mech_Stage(MECH_OP_RIGHT_WORK_1, 1U, d);
    }
    if (btn->right_work2 && !s_mech.last_buttons.right_work2)
    {
        const uint8_t d[1] = {0U};
        (void)Mech_Stage(MECH_OP_RIGHT_WORK_2, 1U, d);
    }
    if (btn->left_pulldown && !s_mech.last_buttons.left_pulldown)
    {
        const uint8_t d[1] = {0U};
        (void)Mech_Stage(MECH_OP_LEFT_PULLDOWN, 1U, d);
    }
    if (btn->right_pulldown && !s_mech.last_buttons.right_pulldown)
    {
        const uint8_t d[1] = {0U};
        (void)Mech_Stage(MECH_OP_RIGHT_PULLDOWN, 1U, d);
    }
    if (btn->grip_open && !s_mech.last_buttons.grip_open)
    {
        const uint8_t d[1] = {0U};
        (void)Mech_Stage(MECH_OP_GRIP_OPEN, 1U, d);
    }
    if (btn->grip_close && !s_mech.last_buttons.grip_close)
    {
        const uint8_t d[1] = {0U};
        (void)Mech_Stage(MECH_OP_GRIP_CLOSE, 1U, d);
    }
    if (btn->grip_release && !s_mech.last_buttons.grip_release)
    {
        const uint8_t d[1] = {0U};
        (void)Mech_Stage(MECH_OP_GRIP_RELEASE, 1U, d);
    }
    if (btn->grip_zero && !s_mech.last_buttons.grip_zero)
    {
        const uint8_t d[1] = {0U};
        (void)Mech_Stage(MECH_OP_GRIP_ZERO, 1U, d);
    }

    s_mech.last_buttons = *btn;
}

/* 把暂存命令发到 CAN1(仅 TIM2 200Hz ISR 调;与 CAN_DequeueTx 同上下文,无需加锁) */
void Mech_Func(void)
{
    if (!s_mech.pending_new)
    {
        return;
    }

    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[8];
    const uint8_t dlc = s_mech.pending_dlc;

    for (uint8_t i = 0U; i < dlc; i++)
    {
        data[i] = s_mech.pending_data[i]; /* 从 volatile 拷到本地再发 */
    }

    hdr.IdType       = FDCAN_EXTENDED_ID;
    hdr.Identifier   = MECH_CAN_TX_BASE | s_mech.pending_op;
    hdr.DataLength   = dlc;

    CAN_Enqueue(&CAN1_Txqueue, hdr, data);
    s_mech.pending_new = false;
}

/* FDCAN1 接收过滤:扩展帧 + 低 3 字节 = 0x050101(机构反馈) */
bool Mech_IsOurs(const FDCAN_RxHeaderTypeDef *rxheader)
{
    if (rxheader == NULL)
    {
        return false;
    }
    if (rxheader->IdType != FDCAN_EXTENDED_ID)
    {
        return false;
    }
    return ((rxheader->Identifier & 0xFFFFFF00U) == MECH_CAN_RX_BASE);
}

/* 解析从板反馈,更新本地状态(仅 FDCAN1 RX ISR 调) */
void Mech_OnFeedback(const FDCAN_RxHeaderTypeDef *rxheader, const uint8_t *data)
{
    const uint8_t op = (uint8_t)(rxheader->Identifier & 0xFFU);

    switch (op)
    {
    case MECH_OP_ENABLE:                 /* 0x05010101 使能确认 */
        s_mech.enabled = (data[0] != 0U);
        break;
    case MECH_OP_LEFT_WORK_1:
    case MECH_OP_LEFT_WORK_2:
    case MECH_OP_LEFT_PULLDOWN:          /* 左臂执行反馈 */
        if (data[0] == 0x00U)      s_mech.arm_left_state = MECH_ARM_STATE_RUNNING;
        else if (data[0] == 0x01U) s_mech.arm_left_state = MECH_ARM_STATE_DONE;
        else if (data[0] == 0x02U) s_mech.arm_left_state = MECH_ARM_STATE_REJECT;
        break;
    case MECH_OP_RIGHT_WORK_1:
    case MECH_OP_RIGHT_WORK_2:
    case MECH_OP_RIGHT_PULLDOWN:         /* 右臂执行反馈 */
        if (data[0] == 0x00U)      s_mech.arm_right_state = MECH_ARM_STATE_RUNNING;
        else if (data[0] == 0x01U) s_mech.arm_right_state = MECH_ARM_STATE_DONE;
        else if (data[0] == 0x02U) s_mech.arm_right_state = MECH_ARM_STATE_REJECT;
        break;
    case MECH_OP_GRIP_OPEN:
    case MECH_OP_GRIP_CLOSE:
    case MECH_OP_GRIP_RELEASE:
    case MECH_OP_GRIP_ZERO:              /* 灵石夹爪执行反馈 */
        if (data[0] == 0x00U)      s_mech.grip_state = MECH_ARM_STATE_RUNNING;
        else if (data[0] == 0x01U) s_mech.grip_state = MECH_ARM_STATE_DONE;
        else if (data[0] == 0x02U) s_mech.grip_state = MECH_ARM_STATE_REJECT;
        break;
    case MECH_OP_RESET:                  /* 复位执行反馈:所有执行器回到空闲 */
        if ((data[0] == 0x01U) || (data[0] == 0x02U))
        {
            s_mech.arm_left_state  = MECH_ARM_STATE_NONE;
            s_mech.arm_right_state = MECH_ARM_STATE_NONE;
            s_mech.grip_state      = MECH_ARM_STATE_NONE;
        }
        break;
    case MECH_OP_ERROR:                  /* 0x050101EE 错误反馈 */
        s_mech.error = true;
        break;
    case MECH_OP_SELFCHECK:              /* 0x05010100 自检应答:可扩展就绪位 */
    default:
        break;
    }
}

/* 组装 mech_status 字节(TX 状态帧 use) */
uint8_t Mech_GetStatus(void)
{
    uint8_t st = 0U;

    if (s_mech.enabled)
    {
        st |= MECH_STATUS_BIT_ENABLED;
    }
    st |= (uint8_t)((s_mech.arm_left_state & 0x3U) << 1);
    st |= (uint8_t)((s_mech.arm_right_state & 0x3U) << 3);
    st |= (uint8_t)((s_mech.grip_state & 0x3U) << 5);
    if (s_mech.error)
    {
        st |= MECH_STATUS_BIT_ERROR;
    }
    return st;
}

void Mech_Init(void)
{
    memset(&s_mech, 0, sizeof(s_mech));
}
