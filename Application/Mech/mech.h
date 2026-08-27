/**
 * @file    mech.h
 * @brief   机构(B2:2 吸盘臂 + 1 灵石夹爪)主控侧业务层(CAN1 与从板通信)。
 *
 * 职责(主控侧,与从板对接):
 *   - 蓝牙按键上升沿 → 一条 CAN1 命令(op 码)发给机构从板;
 *   - 从板反馈(0x050101xx)在 FDCAN1 RX 中断解析,更新本地状态;
 *   - 提供紧凑 mech_status 字节,供蓝牙状态帧回传(调试看臂/爪响应)。
 *
 * 并发(沿用 ZDrive 模式):
 *   - Chassis_Task(任务,200Hz)调用 Mech_PollButtons():按键边沿检测 + 暂存一条待发命令;
 *   - TIM2 200Hz 里 Mech_Func():把暂存命令入 CAN1_Txqueue 发出(任务与队列解耦);
 *   - FDCAN1 RX 中断里 Mech_OnFeedback():解析从板反馈(与 TIM2 同为优先级 5,天然互斥)。
 *
 * 从板做状态机,本模块只保证"按键事件 → 操作码"的映射,不推算臂/爪姿态。
 */
#ifndef MECH_H
#define MECH_H

#include <stdint.h>
#include <stdbool.h>

#include "main.h" /* FDCAN_RxHeaderTypeDef */

#ifdef __cplusplus
extern "C"
{
#endif

/* ------------------------------------------------------------------ */
/* 机构操作码(与 DOCS/B2通信协议.md 对齐)                               */
/* ------------------------------------------------------------------ */
#define MECH_OP_SELFCHECK       0x00U  /* 主控自检广播,从板回 0x05010100 */
#define MECH_OP_ENABLE          0x01U  /* 使能/失能:data[0]=0失能(急停)/1使能 */
#define MECH_OP_LEFT_WORK_1     0x03U  /* 左臂取块 */
#define MECH_OP_LEFT_WORK_2     0x04U  /* 左臂放块 */
#define MECH_OP_RIGHT_WORK_1    0x05U  /* 右臂取块 */
#define MECH_OP_RIGHT_WORK_2    0x06U  /* 右臂放块 */
#define MECH_OP_LEFT_PULLDOWN   0x07U  /* 左臂释放气泵 */
#define MECH_OP_RIGHT_PULLDOWN  0x08U  /* 右臂释放气泵 */
#define MECH_OP_GRIP_OPEN       0x09U  /* 灵石夹爪张开(预备) */
#define MECH_OP_GRIP_CLOSE      0x0AU  /* 灵石夹爪夹紧(取) */
#define MECH_OP_GRIP_RELEASE    0x0BU  /* 灵石夹爪放开(放) */
#define MECH_OP_GRIP_ZERO       0x0CU  /* 灵石夹爪回零 */
#define MECH_OP_ERROR           0xEEU  /* 错误警报(从板回确认) */
#define MECH_OP_RESET           0xFFU  /* 复位:data[0]=0全部/1左/2右/3夹爪, data[1]=0 */

/* CAN ID 基准(扩展帧):主控→机构 = 0x01010500|op;机构→主控 = 0x05010100|op */
#define MECH_CAN_TX_BASE        0x01010500U
#define MECH_CAN_RX_BASE        0x05010100U

/* 执行器状态(mech_status 编码):00=无 / 01=执行中 / 10=完成 / 11=拒绝 */
#define MECH_ARM_STATE_NONE     0U
#define MECH_ARM_STATE_RUNNING  1U
#define MECH_ARM_STATE_DONE     2U
#define MECH_ARM_STATE_REJECT   3U

/* mech_status 位布局(TX 状态帧第 1 字节) */
#define MECH_STATUS_BIT_ENABLED   (1U << 0)
#define MECH_STATUS_BIT_LEFT      (0x3U << 1)   /* 左臂状 2bit */
#define MECH_STATUS_BIT_RIGHT     (0x3U << 3)   /* 右臂状 2bit */
#define MECH_STATUS_BIT_GRIP      (0x3U << 5)   /* 夹爪状 2bit */
#define MECH_STATUS_BIT_ERROR     (1U << 7)

/* ------------------------------------------------------------------ */
/* 蓝牙按键集合(上位机每个"机构按键"的当前状态,用于边沿检测)              */
/* ------------------------------------------------------------------ */
typedef struct
{
    bool enable;         /* mech 使能 */
    bool disable;        /* mech 失能(急停) */
    bool reset;          /* mech 复位(臂/爪回未上电位姿) */
    bool left_work1;     /* 左臂取块 */
    bool left_work2;     /* 左臂放块 */
    bool right_work1;    /* 右臂取块 */
    bool right_work2;    /* 右臂放块 */
    bool left_pulldown;  /* 左臂放气 */
    bool right_pulldown; /* 右臂放气 */
    bool grip_open;      /* 灵石夹爪张开(预备) */
    bool grip_close;     /* 灵石夹爪夹紧(取) */
    bool grip_release;   /* 灵石夹爪放开(放) */
    bool grip_zero;      /* 灵石夹爪回零 */
} MechButtons_t;

typedef struct
{
    /* ---- 待发命令(任务写,Mech_Func ISR 读) ---- */
    volatile bool  pending_new;
    volatile uint8_t pending_op;
    volatile uint8_t pending_dlc;
    volatile uint8_t pending_data[8];

    /* ---- 从板反馈状态(FDCAN1 RX ISR 写,任务读) ---- */
    volatile bool  enabled;
    volatile uint8_t arm_left_state;  /* MECH_ARM_STATE_* */
    volatile uint8_t arm_right_state;
    volatile uint8_t grip_state;      /* MECH_ARM_STATE_* */
    volatile bool  error;

    /* ---- 边沿检测上次按键状态 ---- */
    MechButtons_t last_buttons;
} Mech_t;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */
void        Mech_Init(void);
/* 蓝牙按键边沿检测 + 暂存待发命令(任务上下文 200Hz 调) */
void        Mech_PollButtons(const MechButtons_t *btn);
/* 暂存一条待发命令(忙时返回 false 丢弃);task 调 */
bool        Mech_Stage(uint8_t op, uint8_t dlc, const uint8_t *data);
/* 把暂存命令入 CAN1 队列发出(仅 TIM2 200Hz ISR 调) */
void        Mech_Func(void);
/* FDCAN1 RX 过滤:是不是本从板反馈(扩展帧,低三字节 = 0x050101) */
bool        Mech_IsOurs(const FDCAN_RxHeaderTypeDef *rxheader);
/* FDCAN1 RX:解析从板反馈,更新本地状态(仅 FDCAN1 RX ISR 调) */
void        Mech_OnFeedback(const FDCAN_RxHeaderTypeDef *rxheader, const uint8_t *data);
/* 组装 mech_status 字节(TX 状态帧用) */
uint8_t     Mech_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* MECH_H */
