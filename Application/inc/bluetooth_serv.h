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
/* 服务开关:1 = 开,由任务层(Chassis_Task)周期发状态 + 收命令。            */
/* ------------------------------------------------------------------ */
#define BT_SERV_ENABLE 1U

/* 状态回传周期(ms):Chassis_Task 按此周期发一帧状态(~50Hz) */
#define BT_SERV_STATUS_PERIOD_MS 20U

/* 蓝牙掉线超时(ms):超过该时长未收到任何底板命令,Chassis_Task 判定链路断开,
   执行底盘急停(Chassis_Disable + Zmotor 置 Disable)。应 > 操作手命令更新间隔。 */
#define BT_CMD_TIMEOUT_MS 200U

/* ------------------------------------------------------------------ */
/* 板子接收到的"命令"(字段顺序与上位机协议一致)                          */
/* ------------------------------------------------------------------ */
typedef struct {
    /* 底盘 */
    bool     enable;  /* flag 0: 使能按钮(按下=底盘使能) */
    bool     disable; /* flag 1: 失能按钮(按下=底盘失能) */
    bool     reset;   /* flag 2: 复位按钮(按下=整板复位 NVIC_SystemReset) */
    bool     fast;    /* flag 3: 模式档(1=快速 / 0=慢速) */
    bool     turn_left90;  /* flag 4: 左转90°(边沿,相对当前航向) */
    bool     turn_right90; /* flag 5: 右转90°(边沿,相对当前航向) */
    bool     lock;            /* flag 19: 世界坐标系锁定(1=世界系,0=车身系;上升沿重新冻结基准) */
    bool     tune;            /* flag 20: 速度环 P/I 调参(上升沿把 vel_p/vel_i 应用给四轮电调) */
    /* 目标速度 */
    float    vx;      /* 目标纵向速度,mm/s */
    float    vy;      /* 目标横向速度,mm/s */
    float    wz;      /* 目标自转角速度,deg/s */
    float    vel_p;   /* 速度环 P(tune 触发;量程[0,100],RAM 生效不落 flash) */
    float    vel_i;   /* 速度环 I(tune 触发;量程[0,100],RAM 生效不落 flash) */
} BT_Cmd_t;

/* ------------------------------------------------------------------ */
/* 板子发送的"状态"反馈(字段顺序与上位机协议一致)                         */
/* ------------------------------------------------------------------ */
typedef struct {
    bool     enable;      /* 底盘使能 */
    bool     fast;        /* 模式档:1=快速 / 0=慢速 */
    bool     lock_active; /* 世界坐标系锁定是否激活 */
    float    yaw;         /* IMU 航向遥测(deg,相对上电零位,wrap 到 [-180,180]) */
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