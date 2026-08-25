/**
 * @file    bluetooth_serv.h
 * @brief   蓝牙业务消息层(Application 层):板子与上位机之间的命令/状态收发。
 *
 * 收发各用一个独立结构体,命名直接体现方向:
 *   接收(RX):BT_Cmd_t      —— 板子收到的"命令"(上位机 → 板子)
 *   发送(TX):BT_Status_t   —— 板子反馈的"状态"(板子 → 上位机)
 *
 * 协议层(bluetooth_protocol.h)已按方向拆成两套宏:
 *   RX 用 BT_PROTO_RX_*_NUM + BT_Proto_RX_* 访问函数;
 *   TX 用 BT_PROTO_TX_*_NUM + BT_Proto_TX_* 访问函数。
 * 两方向字段类型/数量可各自独立配置,互不影响。
 *
 * 业务层(用户)要写的就是 bluetooth_serv.c 里的 Pack(状态)/Parse(命令),
 * 其余是快速调用样板。
 */
#ifndef BLUETOOTH_SERV_H
#define BLUETOOTH_SERV_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* 服务开关:1 = 在 VOFA_SendTask 里周期发状态 + 收命令(联调用,默认关)。   */
/* ------------------------------------------------------------------ */
#define BT_SERV_ENABLE 0U

/* ------------------------------------------------------------------ */
/* 板子接收到的"命令"(字段顺序与上位机协议一致)                          */
/* ------------------------------------------------------------------ */
typedef struct {
    bool     enable;  /* flag 0: 使能按钮(按下=底盘使能) */
    bool     disable; /* flag 1: 失能按钮(按下=底盘失能) */
    bool     reset;   /* flag 2: 复位按钮(按下=整板复位 NVIC_SystemReset) */
    float    vx;      /* 目标纵向速度,mm/s */
    float    vy;      /* 目标横向速度,mm/s */
    float    wz;      /* 目标自转角速度,deg/s */
} BT_Cmd_t;

/* ------------------------------------------------------------------ */
/* 板子发送的"状态"反馈(字段顺序与上位机协议一致)                         */
/* ------------------------------------------------------------------ */
typedef struct {

} BT_Status_t;

/* ------------------------------------------------------------------ */
/* 快速调用框架                                                         */
/* ------------------------------------------------------------------ */
/* 发送状态(板子 → 上位机):BT_Status_t → 打包 → Bluetooth_Send()(入 TX 队列)。
   返回 Bluetooth_Send 的结果(入队是否成功)。 */
bool BT_Serv_Status_Send(const BT_Status_t *status);

/* 接收命令(上位机 → 板子):Bluetooth_Receive() → 解析 → BT_Cmd_t。
   timeout_ms 同 Bluetooth_Receive(0 = 非阻塞);没收到帧返回 false。 */
bool BT_Serv_Cmd_Receive(BT_Cmd_t *cmd, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BLUETOOTH_SERV_H */