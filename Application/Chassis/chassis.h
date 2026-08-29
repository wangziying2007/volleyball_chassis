/**
 * @file    chassis.h
 * @brief   四轮全向底盘(45° 辊轮 / 麦克纳姆式)运动学解算(Application 层业务组件)。
 *
 * 输入:vx / vy(mm/s)、wz(deg/s);输出:四轮目标转速(rpm)写入 ZDrive。
 * 只做运动学解算,不含 PID、通信、FreeRTOS 任务,命令来源不关心(蓝牙命令
 * 由 Application 层 bt_serv 解析后调用本模块)。
 *
 * 轮子编号 = 右手系象限顺序(x+ 前、y+ 左、z+ 上,象限逆时针编号):
 *   0 = FL 第一象限 (x+, y+)
 *   1 = FR 第二象限 (x-, y+)
 *   2 = RR 第三象限 (x-, y-)
 *   3 = RL 第四象限 (x+, y-)
 * Chassis_Init() 自动绑定 Zmotor[0..3] 到 wheels[0..3](按上述象限顺序)。
 *
 * 底盘几何参数见下方宏,按实际车体测量修改。
 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdbool.h>
#include <stdint.h>
#include "wheel.h"
#include "pid.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ------------------------------------------------------------------ */
/* 底盘参数(唯一需要按车体修改的地方)                                   */
/* 默认值取自旧 OmniChassis 参数:轮半径 0.064m、等效回转半径 0.375m     */
/* ------------------------------------------------------------------ */
#define CHASSIS_WHEEL_RADIUS_MM  64.0f    /* 轮半径,mm */
#define CHASSIS_HALF_LENGTH_X_MM 187.5f   /* 轮距纵向半长,mm(四轮中心矩形 X 向一半) */
#define CHASSIS_HALF_LENGTH_Y_MM 187.5f   /* 轮距横向半长,mm(Y 向一半) */
#define CHASSIS_L_MM (CHASSIS_HALF_LENGTH_X_MM + CHASSIS_HALF_LENGTH_Y_MM) /* 等效回转半径 L */
#define CHASSIS_VX_MAX_MM_S   1800.0f   /* vx 限幅,mm/s */
#define CHASSIS_VY_MAX_MM_S   1800.0f   /* vy 限幅,mm/s */
#define CHASSIS_WZ_MAX_DEG_S  360.0f    /* wz 限幅,°/s */
#define CHASSIS_MODE_FAST_SCALE 1.0f    /* 快速档缩放(线+角速度同比) */
#define CHASSIS_MODE_SLOW_SCALE 0.25f   /* 慢速档缩放 */

/* ------------------------------------------------------------------ */
/* 航向保持闭环(方案 A:操作手 wz≈0 时,用 IMU 航向锁定并把车稳在锁定方向)     */
/* ------------------------------------------------------------------ */
#define CHASSIS_WZ_HOLD_DEADBAND_DEG_S 4.0f  /* wz 超过该值视为"转向",航向保持暂失效 */
#define CHASSIS_HEADING_KP 3.0f              /* 航向环比例:输出(°/s) / 误差(°) */
#define CHASSIS_HEADING_KI 0.0f              /* 积分(稳态漂移补偿;漂移先不管,置 0) */
#define CHASSIS_HEADING_KD 0.4f              /* 微分(阻尼,抑制过冲) */
#define CHASSIS_HEADING_OUTPUT_LIMIT_DEG_S 120.0f /* 航向修正最大可输出角速度 */
#define CHASSIS_HEADING_SIGN -1.0f            /* 若转向反了,改成 -1.0f */

/* 定点转向(目标 = 当前航向 ± 90°,纯自转;手动 wz 超死区即接管,见 Chassis_EffWz) */
#define CHASSIS_TURN_ANGLE_DEG    90.0f   /* 定点转向角度,deg */
#define CHASSIS_TURN_LEFT_DIR     1.0f    /* 左转方向(目标角增量符号);右转取其负;物理方向反了改 -1 */
#define CHASSIS_TURN_DONE_TOL_DEG 2.0f    /* 到位容差:|目标-当前| 小于该值视为接近 */
#define CHASSIS_TURN_DONE_HOLDS   3U      /* 连续 N 个 tick 在容差内才算到位(防噪声提前停) */
#define CHASSIS_TURN_TIMEOUT_TICKS 500U   /* 超时保护(200Hz 下 = 2.5s):到不了就强制停并锁当前 */

/* 轮子索引(右手系象限顺序,见文件头注释) */
#define CHASSIS_WHEEL_FL 0
#define CHASSIS_WHEEL_FR 1
#define CHASSIS_WHEEL_RR 2
#define CHASSIS_WHEEL_RL 3

typedef struct
{
    Wheel_t wheels[4]; /* FL / FR / RR / RL(象限顺序) */
    bool    enable; // 映射两个bool字段
    bool    reset;
    bool    fast;       /* 当前模式档:1=快速 / 0=慢速 */
    float   vx;        /* 目标纵向速度,mm/s */
    float   vy;        /* 目标横向速度,mm/s */
    float   wz;        /* 目标自转角速度,deg/s */
    PIDType heading_pid;      /* 航向保持闭环 PID */
    float   hold_yaw;         /* 锁定的航向,deg(IMU 相对上电零位) */
    bool    heading_lock_init; /* 是否已锁定(首次 hold / 转向后重新锁定用) */
    /* 定点转向(±90°)状态 */
    bool    turn_pending;     /* 收到了转向命令,待抓当前航向 */
    int8_t  turn_dir;         /* +1=左转90 / -1=右转90(目标角增量方向) */
    bool    turn_active;      /* 正在执行定点转向 */
    float   turn_target_yaw;  /* 目标航向(绝对, deg) */
    uint8_t turn_done_hold;   /* 连续在容差内的 tick 计数 */
    uint16_t turn_elapsed;    /* 执行已用 tick 数(超时保护) */
} Chassis_t;

/* Init 自动绑定 Zmotor[0..3](顺序见文件头);如接线不同可再用 BindWheel 覆盖 */
void Chassis_Init(Chassis_t *chassis);
void Chassis_BindWheel(Chassis_t *chassis, uint8_t index, Zdrive *motor);
void Chassis_SetWheelDirection(Chassis_t *chassis, uint8_t index, float direction);

void Chassis_SetVelocity(Chassis_t *chassis, float vx, float vy, float wz);
void Chassis_Enable(Chassis_t *chassis);
void Chassis_Disable(Chassis_t *chassis);
void Chassis_SetMode(Chassis_t *chassis, bool fast);   /* fast:1=快速 / 0=慢速 */
bool Chassis_IsEnabled(const Chassis_t *chassis);
bool Chassis_GetFast(const Chassis_t *chassis);
void Chassis_Reset(Chassis_t *chassis);
/* yaw_deg = IMU 测量航向(deg,相对上电零位):wz≈0 时用其做航向保持闭环 */
void Chassis_Update(Chassis_t *chassis, float yaw_deg);
/* 定点转向:dir = +1 左转90° / -1 右转90°(相对当前航向)。
   仅在底盘使能时接受;转进行中忽略新请求;手动 wz 超死区会取消。 */
void Chassis_RequestTurn(Chassis_t *chassis, int8_t dir);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H */
