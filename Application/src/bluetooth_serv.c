/**
 * @file    bluetooth_serv.c
 * @brief   蓝牙业务消息层实现(接口见 bluetooth_serv.h)。
 *
 * 数据流:
 *   发送状态:BT_Status_t → BT_Status_Pack → Bluetooth_Send() → TX Queue → 帧层
 *   接收命令:Bluetooth_Receive() → frame.payload → BT_Cmd_Parse → BT_Cmd_t
 *
 * 业务层(用户)要写的就是下面 Pack/Parse 里的字段映射,
 * 其余是固定样板。收发方向字段类型/数量分别改 bluetooth_protocol.h 顶部
 * BT_PROTO_RX_*_NUM / BT_PROTO_TX_*_NUM 即可。
 */
#include "bluetooth_serv.h"

#include "bluetooth.h"          /* Bluetooth_Send / Bluetooth_Receive */
#include "bluetooth_protocol.h" /* BT_Proto_RX_* / BT_Proto_TX_* 访问函数 + 收发方向 payload 大小 */

/* ================================================================== */
/* 用户实现区:按上位机协议逐字段解析/打包(业务层唯一要写的地方)。         */
/* RX 用 BT_Proto_RX_* 访问函数,索引顺序必须与 bluetooth_protocol.h 的
 * BT_PROTO_RX_*_NUM 一致;TX 用 BT_Proto_TX_* 访问函数,索引与 BT_PROTO_TX_*_NUM 一致。         */
/* ================================================================== */
/* 接收:命令帧解析(RX) */
static void BT_Cmd_Parse(const uint8_t *payload, BT_Cmd_t *cmd)
{
    uint8_t bool_cnt = 0U;
    uint8_t f32_cnt  = 0U;
    /* 命令字段(顺序须与 comm_config.h 的 BT_PROTO_RX_* 一致):
       bool[0..19] = enable/disable/reset/fast/turn_left90/turn_right90 + 机构 13 字段 + lock,
       float[0..2] = vx/vy/wz */
    cmd->enable         = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->disable        = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->reset          = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->fast           = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->turn_left90    = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->turn_right90   = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->mech_enable    = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->mech_disable   = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->mech_reset     = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->left_work1     = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->left_work2     = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->right_work1    = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->right_work2    = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->left_pulldown  = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->right_pulldown = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->grip_open      = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->grip_close     = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->grip_release   = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->grip_zero      = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->lock           = BT_Proto_RX_ReadBool(payload, bool_cnt++);
    cmd->vx             = BT_Proto_RX_ReadFloat(payload, f32_cnt++);
    cmd->vy             = BT_Proto_RX_ReadFloat(payload, f32_cnt++);
    cmd->wz             = BT_Proto_RX_ReadFloat(payload, f32_cnt++);
}

/* 发送:状态帧打包(TX) */
static void BT_Status_Pack(uint8_t *payload, const BT_Status_t *status)
{
    BT_Proto_TX_WriteBool(payload, 0, status->enable);
    BT_Proto_TX_WriteBool(payload, 1, status->fast);
    BT_Proto_TX_WriteBool(payload, 2, status->lock_active);
    BT_Proto_TX_WriteU8(payload, 0, status->mech_status);
    BT_Proto_TX_WriteFloat(payload, 0, status->yaw);
}

/* ================================================================== */
/* 固定样板:框架层调用入口,一般不用改                                   */
/* ================================================================== */
bool BT_Serv_Status_Send(const BT_Status_t *status)
{
    uint8_t payload[BT_PROTO_TX_PAYLOAD_SIZE];

    BT_Status_Pack(payload, status);
    return Bluetooth_Send(payload, BT_PROTO_TX_PAYLOAD_SIZE);
}

bool BT_Serv_Cmd_Receive(BT_Cmd_t *cmd, uint32_t timeout_ms)
{
    BluetoothFrame_t frame;

    if (Bluetooth_Receive(&frame, timeout_ms) != true)
    {
        return false;
    }

    BT_Cmd_Parse(frame.payload, cmd);
    return true;
}