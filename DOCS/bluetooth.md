# 蓝牙串口通信模块使用说明(Communication/Bluetooth)

> 面向“刚学会 STM32 + FreeRTOS 任务/队列”的队友。本文只讲怎么用、怎么改,
> 原理上每个文件里都有注释。

## 1. 这个模块解决什么问题

机器人上要通过蓝牙串口(HC-05 / JDY-31 等蓝牙模块,接到 USART6:PC6=TX, PC7=RX)
和上位机互发数据。串口数据是“流”,没有边界;我们约定一种带包头包尾和校验的
**帧格式**,把流切成一条条完整的消息。


业务层只需要:

```c
Bluetooth_Send(数据, 长度);          // 发一条消息
Bluetooth_Receive(&frame, 超时ms);   // 收一条消息
```

业务层**不需要知道**:UART、DMA、IDLE、CRC、缓冲区、发送忙不忙。

## 2. 数据流向

```
发送:
  业务数据 --Bluetooth_Send()--> TX Queue --> Bluetooth_TxTask
              --组帧(包头/包长开关/包体/CRC/包尾)--> UART DMA --> 蓝牙模块 --> 上位机

接收:
  上位机 --> 蓝牙模块 --> UART DMA 收到 --> IDLE 中断
              --Bluetooth_UartRxEvent()--> 环形缓冲(只搬数据,不解析)
              --信号量(唤醒)--> Bluetooth_RxTask(找包头->读包长开关->收满一帧->查包尾->算CRC)
              --> RX Queue --> 业务 Bluetooth_Receive()
```

中断里只做“搬数据 + 通知”,**所有解析(包头/包尾/CRC)都在任务里**,不会卡死中断。

## 3. 线上帧格式(收发双方必须一致)

帧格式由几个配置宏编译期定死(收发双方约定一致,无运行时判断):

| 组成 | 配置宏 | 说明 |
| --- | --- | --- |
| 包头 | `BT_FRAME_HEADER1` / `BT_FRAME_HEADER2` | 双包头;**包头2 设为 0 = 单包头**(只用包头1) |
| 包长 | `BT_PAYLOAD_LENGTH` | 0 = 无包长字段;N(1..32) = 带 1 字节包长字段 |
| 包体 | — | 固定 `BT_FRAME_PAYLOAD_LEN` 字节 |
| 校验 | `BT_CHECKSUM_MODE` | 0 = CRC16,1 = 单字节校验和 |
| 包尾 | `BT_FRAME_TAIL1` / `BT_FRAME_TAIL2` | 双包尾;**包尾2 设为 0 = 单包尾**(只用包尾1) |

示意(双包头 + 有包长 + 双包尾为例):

| 字节 | 内容 | 说明 |
| --- | --- | --- |
| 0 .. H-1 | 包头 | H = 包头字节数(单=1 / 双=2) |
| H | N | 包长(仅当 BT_PAYLOAD_LENGTH > 0) |
| .. | payload[N] | 包体,固定 N 字节 |
| .. | 校验 | CRC16(2 字节小端)或单字节校验和 |
| 最后 1/2 | 包尾 | T = 包尾字节数(单=1 / 双=2) |

**帧长 = 包头(H) + 包长(1,如有) + 包体(N) + 校验(1/2) + 包尾(T)**

**校验方式(`BT_CHECKSUM_MODE`,bluetooth.c 顶部,与上位机约定二选一)**:

- `0` = **CRC16**:CRC-CCITT(init=0xFFFF),2 字节小端,用 Algorithm/crc_ccitt.c
  (查表多项式 0x8408,初值 0xFFFF);
- `1` = **单字节校验和**:覆盖字节求和,取低 8 位,1 字节。

**校验范围(不含包头)**:

- 有包长字段:`包长 + 包体`;
- 无包长字段:`包体` 本身。

示例(包体 N = 13,CRC16 / SUM8):

| 包头 | 包长 | 包尾 | 帧长(CRC16) | 帧长(SUM8) |
| --- | --- | --- | --- | --- |
| 双(2) | 无 | 单(1) | 2+15+2+1 = 20 | 19 |
| 双(2) | 有 | 单(1) | 2+1+15+2+1 = 21 | 20 |
| 单(1) | 无 | 双(2) | 1+15+2+2 = 20 | 19 |
| 单(1) | 有 | 双(2) | 1+1+15+2+2 = 21 | 20 |
| **单(1)** | 无 | **单(1)** | 1+15+2+1 = 19 | **1+15+1+1 = 18** |

最后一行示例只是"包体 13B"的通用演示。本工程当前配置(单包头 `0xA5`、无包长字段、单包尾 `0x5A`、SUM8):
**收发方向各自用各自包长,不再补到同一长度**:命令帧(包体 23B)= 26 字节,状态帧(包体 6B)= 9 字节。

补充说明:

- **包头/包尾单双**:把 `BT_FRAME_HEADER2` / `BT_FRAME_TAIL2` 设为 0 即取消第二个字节;
  单双由收发双方约定,编译期定死,运行时不变;
- **包长字段的作用**:帧长按宏编译期定死(单帧类型场景,不做运行时变长);
  接收时除了校验,还会显式核对收到的包长字节 == 约定值,写错直接丢帧;
- **想省带宽**(蓝牙串口慢):把 `BT_PAYLOAD_LENGTH` 改成实际需要的字节数(如 14),
  或改用 `BT_CHECKSUM_MODE = 1`(少 1 字节校验);
- 校验范围里包含包长,包长被干扰时校验会失败,不会收错帧;
- 想改包长/校验方式:改 bluetooth.c 顶部的宏,收发两边一起改。

## 4. 业务层用法

```c
#include "bluetooth.h"

// 任务初始化时调用一次(main.c 已经调好了,新人不用管)
Bluetooth_Init();

// ---- 发送 ----
uint8_t cmd[2] = {0x01, 0x02};
if (Bluetooth_Send(cmd, 2)) {
    // 发送成功(只是入队成功,不代表已经发出去;真正发由 TX 任务负责)
}

// ---- 接收 ----
BluetoothFrame_t frame;
if (Bluetooth_Receive(&frame, 100)) {        // 等 100ms,没消息返回 false
    // frame.payload[0 .. frame.length-1] 就是本帧数据
    // frame.length 是实际包体长度(<= BT_PROTO_PAYLOAD_MAX)
    // 业务层在这里按自己的约定解析 payload
}
```

队列深度、超时、任务栈等参数都在 bluetooth.h 顶部,改数字就行。

## 5. 已经帮你接好的线(新人了解即可)

| 位置 | 做了什么 |
| --- | --- |
| Config/board_config.h | BOARD_BLUETOOTH_UART = &huart6(蓝牙用的串口) |
| Core/Src/main.c | 启动时调用 Bluetooth_Init()(在串口 DMA 接收启动之前) |
| Application/src/IQRhandler.c | USART6 空闲中断 -> Bluetooth_UartRxEvent();发送完成中断 -> Bluetooth_UartTxCplt() |
| MDK-ARM/*.uvprojx | Communication/Bluetooth 组挂 bluetooth.c;bluetooth_protocol.h 为纯头文件,无 .c(include path 已含该目录与 Config 目录) |

换串口:改 board_config.h 的 BOARD_BLUETOOTH_UART,并把 IQRhandler.c 里
对应串口的“在这里加 XX 的协议处理”替换成 Bluetooth_UartRxEvent(...)。

## 6. Payload 协议层(宏驱动的定长编解码)

协议层(Communication/Bluetooth/bluetooth_protocol.h,**纯头文件**,无 .c)与业务层
对齐,**按收发方向各拆一套宏**:收(RX,命令)一套、发(TX,状态)一套,
每方向独立用宏把字段定死,编译期推导各自的定长布局,再提供逐字段读写函数:

- 在 `Config/comm_config.h` 里按上位机约定**分方向**填数量宏(`bluetooth_protocol.h` 会引用它):
  - RX(接收/命令):`BT_PROTO_RX_BOOL_NUM / BT_PROTO_RX_U8_NUM / BT_PROTO_RX_U16_NUM / BT_PROTO_RX_U32_NUM / BT_PROTO_RX_FLOAT_NUM`;
  - TX(发送/状态):`BT_PROTO_TX_BOOL_NUM / BT_PROTO_TX_U8_NUM / BT_PROTO_TX_U16_NUM / BT_PROTO_TX_U32_NUM / BT_PROTO_TX_FLOAT_NUM`。
  (0 = 该方向该类型不出现,对应访问函数整段被 `#if` 取消编译);
  RX 方向只提供 `BT_Proto_RX_Read*`(解析命令只读),TX 方向只提供 `BT_Proto_TX_Write*`(打包状态只写);
- 每个方向布局各自独立:`bool` 打包区(LSB 在前,第 1 个 bool 在 bit0)→ `u8` → `u16`(小端)→
  `u32`(小端)→ `float`(小端),RX/TX 各有一个 `BT_PROTO_RX_PAYLOAD_SIZE` / `BT_PROTO_TX_PAYLOAD_SIZE` 定长;
  帧层缓冲用两者较大值 `BT_PROTO_PAYLOAD_MAX`;
- 业务层**唯一要做的是写解析函数**,把每个字段赋给业务结构体成员,
  不碰位移/按位与/字节偏移/`sizeof`/`memcpy`:

```c
#include "bluetooth_protocol.h"

typedef struct {
    bool     enable, auto_mode, stop;   /* 字段顺序与协议一致 */
    uint16_t speed, angle;
    float    vx;
} MyMsg_t;

/* 用户实现:解析(接收) */
void MyMsg_Parse(const uint8_t *payload, MyMsg_t *msg)
{
    msg->enable    = BT_Proto_RX_ReadBool(payload, 0);   /* 解析命令用 RX 访问函数 */
    msg->auto_mode = BT_Proto_RX_ReadBool(payload, 1);
    msg->stop      = BT_Proto_RX_ReadBool(payload, 2);
    msg->speed     = BT_Proto_RX_ReadU16(payload, 0);
    msg->angle     = BT_Proto_RX_ReadU16(payload, 1);
    msg->vx        = BT_Proto_RX_ReadFloat(payload, 0);
}

/* 用户实现:打包(发送),对称的 Write 系列 */
void MyMsg_Pack(uint8_t *payload, const MyMsg_t *msg) { ... }
```

完整示例见 `Application/bluetooth_serv.c`(`BT_Cmd_Parse` 用 BT_Proto_RX_* 收命令 / `BT_Status_Pack`
用 BT_Proto_TX_* 发状态;底盘协议的收发字段见 6.1 节字段表)。
把 `bluetooth_serv.h` 里 `BT_SERV_ENABLE` 改成 1,`Chassis_Task` 收命令并回传状态,
用于验证整条链路。换真实协议时:改对应方向宏数量 → 改对应业务结构体 → 改 Pack/Parse 函数。

> 省带宽:帧层默认 `BT_PAYLOAD_LENGTH = 0`。**收发方向各自用各自包长**:RX(命令)包体 = `BT_PROTO_RX_PAYLOAD_SIZE`,TX(状态)包体 = `BT_PROTO_TX_PAYLOAD_SIZE`,互不补齐。
> 帧长 = 包头(1) + 包体(`BT_PROTO_RX|TX_PAYLOAD_SIZE`) + 校验(1) + 包尾(1)。当前:命令帧 26 字节、状态帧 9 字节。
> 改包体大小:改 `Config/comm_config.h` 的收发方向字段数量宏,对应方向的 `BT_PROTO_RX|TX_PAYLOAD_SIZE` 随之变化,收发两边一起改。

### 6.1 底盘命令/状态字段表

**RX 命令(上位机 → 板子)**:body = 23 字节 = bool 打包区 3 字节 + 5 个 float 20 字节。

| body 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | bool[0] | 1 bit | enable(底盘使能) |
| .. | bool[1] | 1 bit | disable(底盘失能) |
| .. | bool[2] | 1 bit | reset(整板复位) |
| .. | bool[3] | 1 bit | fast(快速模式,默认 1.0x) |
| .. | bool[4] | 1 bit | turn_left90(左转90°,边沿,相对当前航向) |
| .. | bool[5] | 1 bit | turn_right90(右转90°,边沿,相对当前航向) |
| .. | bool[6] | 1 bit | mech_enable(机构使能) |
| .. | bool[7] | 1 bit | mech_disable(机构失能/急停) |
| 1 | bool[8] | 1 bit | mech_reset(机构复位) |
| .. | bool[9] | 1 bit | left_work1(左臂取块) |
| .. | bool[10] | 1 bit | left_work2(左臂放块) |
| .. | bool[11] | 1 bit | right_work1(右臂取块) |
| .. | bool[12] | 1 bit | right_work2(右臂放块) |
| .. | bool[13] | 1 bit | left_pulldown(左臂放气) |
| .. | bool[14] | 1 bit | right_pulldown(右臂放气) |
| .. | bool[15] | 1 bit | grip_open(灵石夹爪张开) |
| 2 | bool[16] | 1 bit | grip_close(灵石夹爪夹紧) |
| .. | bool[17] | 1 bit | grip_release(灵石夹爪放开) |
| .. | bool[18] | 1 bit | grip_zero(灵石夹爪回零) |
| 2 | bool[19] | 1 bit | world_lock(世界坐标系锁定:1=世界系 / 0=车身系) |
| 2 | bool[20] | 1 bit | tune(速度环 P/I 调参:上升沿应用 vel_p/vel_i 给四轮) |
| 3 | float[0] | 4B 小端 | vx 纵向速度,mm/s |
| 7 | float[1] | 4B 小端 | vy 横向速度,mm/s |
| 11 | float[2] | 4B 小端 | wz 自转角速度,°/s |
| 15 | float[3] | 4B 小端 | vel_p 速度环 P(量程[0,100],tune 沿应用;RAM 生效,掉电回退) |
| 19 | float[4] | 4B 小端 | vel_i 速度环 I(量程[0,100],同上) |

21 个 bool 打包进字节 0/1/2(LSB 在前):字节0 = enable/disable/reset/fast/turn_left90/turn_right90/mech_enable/mech_disable;
字节1 = mech_reset/left_work1/left_work2/right_work1/right_work2/left_pulldown/right_pulldown/grip_open;
字节2 = grip_close/grip_release/grip_zero/world_lock/tune + 3 位保留。

**TX 状态(板子 → 上位机)**:body = 6 字节 = bool 打包区(3 bit,LSB 在前)+ mech_status(1 字节)+ yaw(4B 小端);其余 body 字节补 0。

| body 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | bool[0] | 1 bit | enable(当前使能状态) |
| .. | bool[1] | 1 bit | fast(当前快速模式) |
| .. | bool[2] | 1 bit | world_lock(世界系锁定当前是否激活) |
| 1 | mech_status | 1B | 紧凑机构状态(位布局见 `Application/Mech/mech.h` 的 `MECH_STATUS_*`) |
| 2 | yaw | 4B 小端 | IMU 航向遥测,deg,相对上电零位(wrap 到 [-180,180]) |

> 蓝牙帧不区分消息类型,payload 直接承载上述字段(RX = 23 字节、TX = 6 字节)。
> 收发方向各自用各自包长,不再补到同一长度:命令帧 **26 字节**(包体 23),状态帧 **9 字节**(包体 6)。

## 7. 常见问题

- **收不到**:先看蓝牙模块波特率(默认 115200)和上位机是否一致;再用串口助手
  直接看 UART6_RxBuffer 有没有原始数据。
- **收到但解析失败**:用 VOFA+ 或串口助手看原始字节,检查包头是否被上位机
  转成了字符串/加了换行;再核对帧格式是否与 BT_PAYLOAD_LENGTH 一致(0 时无包长字段,命令帧 26 字节、状态帧 9 字节;N 时包头后的包长字节 = N)。
- **发送队列满**(Bluetooth_Send 返回 false):把 BT_TX_QUEUE_SIZE 调大,
  或检查 TX 任务是不是卡在 DMA 上(发送完成中断没清 s_tx_busy)。
- **DMA 相关报错**:确认缓冲在 D2 段(__RAM_D2_),发送前刷 D-Cache
  (SCB_CleanDCache_by_Addr),接收后失效(SCB_InvalidateDCache_by_Addr),
  这些模块和 IQRhandler 里都已经按模板规范写好,别删。