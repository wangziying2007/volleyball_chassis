/**
 * @file    bluetooth_protocol.h
 * @brief   蓝牙 Payload 协议层:宏驱动的定长编解码,收发(RX/TX)各一套布局。
 *
 * 与业务层(bluetooth_serv.h)对齐:那里已区分"接收命令 BT_Cmd_t(RX)"
 * 与"发送状态 BT_Status_t(TX)",所以协议层也**按方向分两套宏**——
 * 收(RX)、发(TX)各自独立的字段类型与数量,分别推导各自的定长布局,
 * 避免两方向字段不同时互相干扰。
 *
 * 职责划分:RX = 接收(解析命令,只读),TX = 发送(打包状态,只写)。
 * 因此同一数据类型下,RX 只提供 BT_Proto_RX_Read*,TX 只提供 BT_Proto_TX_Write*,
 * 不会出现方向与操作错配。
 *
 * 设计前提:每个方向实际只会用**一种**蓝牙帧,字段类型与数量固定,
 * 字段数量由 Config/comm_config.h 按上位机协议约定填好。本层根据宏推导出定长布局,
 * 提供逐字段的读写访问函数;业务层唯一要做的是写解析函数,
 * 把每个字段赋给业务结构体成员,不碰位移/按位与/字节偏移。
 *
 * 布局(编译期固定,类型按协议顺序分组,每个方向各自独立):
 *   [0, BOOL_BYTES)         : bool 打包区(LSB 在前,第 1 个 bool 在 bit0)
 *   [BOOL_BYTES, +U8_NUM)   : u8
 *   [.., +2*U16_NUM)        : u16(小端)
 *   [.., +4*U32_NUM)        : u32(小端)
 *   [.., +4*FLOAT_NUM)      : float(小端)
 *
 * 某方向某类型数量为 0 时,该方向该类型访问函数被 #if 取消编译,不占代码。
 * 调用方保证:payload 长度 >= 对应方向的 BT_PROTO_<RX|TX>_PAYLOAD_SIZE,
 * idx 在数量范围内。
 *
 * 帧层缓冲(BluetoothFrame_t.payload)要同时装得下收发两向,取两者较大值:
 * BT_PROTO_PAYLOAD_MAX。
 */
#ifndef BLUETOOTH_PROTOCOL_H
#define BLUETOOTH_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h> /* memcpy(仅 float 位拷贝) */

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
/* 协议字段数量配置:由 Config/comm_config.h 提供。                       */
/*   RX = 接收(命令,只读)→ BT_PROTO_RX_*_NUM;                       */
/*   TX = 发送(状态,只写)→ BT_PROTO_TX_*_NUM。                       */
/* 按上位机约定填写每种字段数量(0 = 该类型不出现)。                     */
/*                                                                      */
/* 底盘/机构分支字段约定(见 comm_config.h 顶部说明):                           */
/*   RX 命令(BT_Cmd_t):   bool[0]=enable 使能, [1]=disable 失能,                */
/*                        [2]=reset 板子复位, [3]=fast 模式档,                   */
/*                        [4]=turn_left90 左转90°, [5]=turn_right90 右转90°;      */
/*                        bool[6..18]=机构 13 字段;                               */
/*                        float[0]=vx(mm/s), [1]=vy(mm/s), [2]=wz(°/s)           */
/*   TX 状态(BT_Status_t): bool[0]=enable 使能, [1]=fast 模式档,                  */
/*                        u8[0]=mech_status(紧凑机构状态),                        */
/*                        float[0]=yaw(IMU 航向遥测, deg,相对上电零位)。           */
/* ------------------------------------------------------------------ */
#include "comm_config.h"

/* ------------------------------------------------------------------ */
/* 由数量推导的定长布局(编译期常量,勿手动改)                             */
/* ------------------------------------------------------------------ */

/* ---- RX(接收)方向布局 ---- */
#define BT_PROTO_RX_BOOL_BYTES   ((BT_PROTO_RX_BOOL_NUM + 7U) / 8U)
#define BT_PROTO_RX_U8_OFFSET    (BT_PROTO_RX_BOOL_BYTES)
#define BT_PROTO_RX_U16_OFFSET   (BT_PROTO_RX_U8_OFFSET + BT_PROTO_RX_U8_NUM)
#define BT_PROTO_RX_U32_OFFSET   (BT_PROTO_RX_U16_OFFSET + 2U * BT_PROTO_RX_U16_NUM)
#define BT_PROTO_RX_FLOAT_OFFSET (BT_PROTO_RX_U32_OFFSET + 4U * BT_PROTO_RX_U32_NUM)
#define BT_PROTO_RX_PAYLOAD_SIZE (BT_PROTO_RX_FLOAT_OFFSET + 4U * BT_PROTO_RX_FLOAT_NUM)

/* ---- TX(发送)方向布局 ---- */
#define BT_PROTO_TX_BOOL_BYTES   ((BT_PROTO_TX_BOOL_NUM + 7U) / 8U)
#define BT_PROTO_TX_U8_OFFSET    (BT_PROTO_TX_BOOL_BYTES)
#define BT_PROTO_TX_U16_OFFSET   (BT_PROTO_TX_U8_OFFSET + BT_PROTO_TX_U8_NUM)
#define BT_PROTO_TX_U32_OFFSET   (BT_PROTO_TX_U16_OFFSET + 2U * BT_PROTO_TX_U16_NUM)
#define BT_PROTO_TX_FLOAT_OFFSET (BT_PROTO_TX_U32_OFFSET + 4U * BT_PROTO_TX_U32_NUM)
#define BT_PROTO_TX_PAYLOAD_SIZE (BT_PROTO_TX_FLOAT_OFFSET + 4U * BT_PROTO_TX_FLOAT_NUM)

/* 帧层缓冲尺寸:取收发两向 payload 的较大值,保证一帧能装下任一方向 */
#if BT_PROTO_RX_PAYLOAD_SIZE > BT_PROTO_TX_PAYLOAD_SIZE
#define BT_PROTO_PAYLOAD_MAX  BT_PROTO_RX_PAYLOAD_SIZE
#else
#define BT_PROTO_PAYLOAD_MAX  BT_PROTO_TX_PAYLOAD_SIZE
#endif

/* ------------------------------------------------------------------ */
/* 字段访问函数:每个方向每个类型独立 #if,数量为 0 时该段不编译。          */
/* 职责划分:RX 只读(解析命令) → BT_Proto_RX_Read*;                   */
/*           TX 只写(打包状态) → BT_Proto_TX_Write*。                 */
/* ------------------------------------------------------------------ */

/* ---- RX(接收)方向访问函数:只读 ---- */
#if BT_PROTO_RX_BOOL_NUM > 0U
/* 第 idx 个 bool(0 起),LSB 在前:第 1 个 = 字节 bit0,第 8 个 = bit7 */
static inline bool BT_Proto_RX_ReadBool(const uint8_t *payload, uint8_t idx)
{
    return ((payload[idx >> 3] >> (idx & 7U)) & 1U) != 0U;
}
#endif /* BT_PROTO_RX_BOOL_NUM > 0 */

#if BT_PROTO_RX_U8_NUM > 0U
static inline uint8_t BT_Proto_RX_ReadU8(const uint8_t *payload, uint8_t idx)
{
    return payload[BT_PROTO_RX_U8_OFFSET + idx];
}
#endif /* BT_PROTO_RX_U8_NUM > 0 */

#if BT_PROTO_RX_U16_NUM > 0U
/* 第 idx 个 u16(0 起),小端 */
static inline uint16_t BT_Proto_RX_ReadU16(const uint8_t *payload, uint8_t idx)
{
    uint16_t off = (uint16_t)(BT_PROTO_RX_U16_OFFSET + 2U * idx);
    return (uint16_t)((uint16_t)payload[off] | ((uint16_t)payload[off + 1U] << 8));
}
#endif /* BT_PROTO_RX_U16_NUM > 0 */

#if BT_PROTO_RX_U32_NUM > 0U
static inline uint32_t BT_Proto_RX_ReadU32(const uint8_t *payload, uint8_t idx)
{
    uint16_t off = (uint16_t)(BT_PROTO_RX_U32_OFFSET + 4U * idx);
    return (uint32_t)payload[off]
         | ((uint32_t)payload[off + 1U] << 8)
         | ((uint32_t)payload[off + 2U] << 16)
         | ((uint32_t)payload[off + 3U] << 24);
}
#endif /* BT_PROTO_RX_U32_NUM > 0 */

#if BT_PROTO_RX_FLOAT_NUM > 0U
/* 第 idx 个 float(0 起),IEEE754 单精度,小端 */
static inline float BT_Proto_RX_ReadFloat(const uint8_t *payload, uint8_t idx)
{
    uint16_t off = (uint16_t)(BT_PROTO_RX_FLOAT_OFFSET + 4U * idx);
    uint32_t bits = (uint32_t)payload[off]
                  | ((uint32_t)payload[off + 1U] << 8)
                  | ((uint32_t)payload[off + 2U] << 16)
                  | ((uint32_t)payload[off + 3U] << 24);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}
#endif /* BT_PROTO_RX_FLOAT_NUM > 0 */

/* ---- TX(发送)方向访问函数:只写 ---- */
#if BT_PROTO_TX_BOOL_NUM > 0U
/* 第 idx 个 bool(0 起),LSB 在前:第 1 个 = 字节 bit0,第 8 个 = bit7 */
static inline void BT_Proto_TX_WriteBool(uint8_t *payload, uint8_t idx, bool value)
{
    if (value)
    {
        payload[idx >> 3] |= (uint8_t)(1U << (idx & 7U));
    }
    else
    {
        payload[idx >> 3] &= (uint8_t)~(1U << (idx & 7U));
    }
}
#endif /* BT_PROTO_TX_BOOL_NUM > 0 */

#if BT_PROTO_TX_U8_NUM > 0U
static inline void BT_Proto_TX_WriteU8(uint8_t *payload, uint8_t idx, uint8_t value)
{
    payload[BT_PROTO_TX_U8_OFFSET + idx] = value;
}
#endif /* BT_PROTO_TX_U8_NUM > 0 */

#if BT_PROTO_TX_U16_NUM > 0U
/* 第 idx 个 u16(0 起),小端 */
static inline void BT_Proto_TX_WriteU16(uint8_t *payload, uint8_t idx, uint16_t value)
{
    uint16_t off = (uint16_t)(BT_PROTO_TX_U16_OFFSET + 2U * idx);
    payload[off]       = (uint8_t)(value & 0xFFU);
    payload[off + 1U]  = (uint8_t)(value >> 8);
}
#endif /* BT_PROTO_TX_U16_NUM > 0 */

#if BT_PROTO_TX_U32_NUM > 0U
static inline void BT_Proto_TX_WriteU32(uint8_t *payload, uint8_t idx, uint32_t value)
{
    uint16_t off = (uint16_t)(BT_PROTO_TX_U32_OFFSET + 4U * idx);
    payload[off]       = (uint8_t)(value & 0xFFU);
    payload[off + 1U]  = (uint8_t)((value >> 8) & 0xFFU);
    payload[off + 2U]  = (uint8_t)((value >> 16) & 0xFFU);
    payload[off + 3U]  = (uint8_t)((value >> 24) & 0xFFU);
}
#endif /* BT_PROTO_TX_U32_NUM > 0 */

#if BT_PROTO_TX_FLOAT_NUM > 0U
static inline void BT_Proto_TX_WriteFloat(uint8_t *payload, uint8_t idx, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    uint16_t off = (uint16_t)(BT_PROTO_TX_FLOAT_OFFSET + 4U * idx);
    payload[off]       = (uint8_t)(bits & 0xFFU);
    payload[off + 1U]  = (uint8_t)((bits >> 8) & 0xFFU);
    payload[off + 2U]  = (uint8_t)((bits >> 16) & 0xFFU);
    payload[off + 3U]  = (uint8_t)((bits >> 24) & 0xFFU);
}
#endif /* BT_PROTO_TX_FLOAT_NUM > 0 */

#ifdef __cplusplus
}
#endif

#endif /* BLUETOOTH_PROTOCOL_H */