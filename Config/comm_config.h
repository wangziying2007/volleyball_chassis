/**
 * @file    comm_config.h
 * @brief   通信层配置宏(蓝牙/串口等协议参数)。
 *
 * 这里集中放各通信模块的可调宏,改参数不用去翻协议实现文件。
 * 当前含蓝牙 Payload 协议层的字段数量配置(bluetooth_protocol.h 会引用本文件):
 *   按上位机约定分别填写收发两个方向的每种字段数量(0 = 不出现)。
 *
 *   RX = 接收(命令,上位机 → 板子),对应业务层 BT_Cmd_t,只读;
 *   TX = 发送(状态,板子 → 上位机),对应业务层 BT_Status_t,只写。
 */
#ifndef COMM_CONFIG_H
#define COMM_CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ------------------------------------------------------------------ */
/* 蓝牙 Payload 协议字段数量配置(bluetooth_protocol.h 读取)             */
/* ------------------------------------------------------------------ */

/* ---- RX(接收)方向 ---- */
/* 命令字段:bool[21]=底盘(enable/disable/reset/fast/turn_left90/turn_right90/world_lock/tune) + 机构(mech_enable/mech_disable/
   mech_reset/left_work1/left_work2/right_work1/right_work2/left_pulldown/right_pulldown/
   grip_open/grip_close/grip_release/grip_zero);float[5]=vx/vy/wz/vel_p/vel_i */
#ifndef BT_PROTO_RX_BOOL_NUM
#define BT_PROTO_RX_BOOL_NUM   21U  /* bool 数量:底盘 8 + 机构 13 */
#endif
#ifndef BT_PROTO_RX_U8_NUM
#define BT_PROTO_RX_U8_NUM     0U   /* u8 数量 */
#endif
#ifndef BT_PROTO_RX_U16_NUM
#define BT_PROTO_RX_U16_NUM    0U   /* u16 数量(命令不使用) */
#endif
#ifndef BT_PROTO_RX_U32_NUM
#define BT_PROTO_RX_U32_NUM    0U   /* u32 数量 */
#endif
#ifndef BT_PROTO_RX_FLOAT_NUM
#define BT_PROTO_RX_FLOAT_NUM  5U   /* float 数量:vx/vy/wz + vel_p/vel_i(速度环 P/I 调参) */
#endif

/* ---- TX(发送)方向 ---- */
/* 状态字段:bool[3]=enable/fast/lock_active,u8[1]=mech_status(紧凑机构状态),float[1]=yaw(IMU 航向遥测,deg)。 */
#ifndef BT_PROTO_TX_BOOL_NUM
#define BT_PROTO_TX_BOOL_NUM   3U   /* bool 数量:enable / fast / lock_active */
#endif
#ifndef BT_PROTO_TX_U8_NUM
#define BT_PROTO_TX_U8_NUM     1U   /* u8 数量:mech_status */
#endif
#ifndef BT_PROTO_TX_U16_NUM
#define BT_PROTO_TX_U16_NUM    0U   /* u16 数量 */
#endif
#ifndef BT_PROTO_TX_U32_NUM
#define BT_PROTO_TX_U32_NUM    0U   /* u32 数量 */
#endif
#ifndef BT_PROTO_TX_FLOAT_NUM
#define BT_PROTO_TX_FLOAT_NUM  1U   /* float 数量:yaw 航向遥测 */
#endif

#ifdef __cplusplus
}
#endif

#endif /* COMM_CONFIG_H */