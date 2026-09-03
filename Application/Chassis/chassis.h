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

/* 航向保持 + 定点转向(车身自转)方向 = CHASSIS_HEADING_SIGN。
   标定(关世界系锁定):使能后 wz=0,手推一下车再放手——回正到锁定方向=稳=>当前符号对;
   若往反方向跑(发散)=> 改成 +1。turn_left/right 转向方向与此同号联调。 */
#define CHASSIS_HEADING_SIGN -1.0f

/* 世界系→车身系 平移旋转方向 = CHASSIS_DIR_LOCK_SIGN(专属宏,与自转方向独立)。*/
#define CHASSIS_DIR_LOCK_SIGN +1.0f

/* 定点转向(目标 = 当前航向 ± 90°,纯自转;手动 wz 超死区即接管)。
   用梯形速度剖面执行取代位置式 PID;
   独立于航向保持 PID(调快不牵连航向保持手感)。 */
#define CHASSIS_TURN_ANGLE_DEG    90.0f   /* 定点转向角度,deg */
#define CHASSIS_TURN_LEFT_DIR     1.0f    /* 左转方向(目标角增量符号);右转取其负;物理方向反了改 -1 */
#define CHASSIS_TURN_SPEED_DEG_S  300.0f  /* 巡航角速度,°/s(快速档上限;纯自转 300°/s 轮速约 250rpm) */
#define CHASSIS_TURN_DECEL_DEG_S2 1500.0f /* 末段恒减速度,°/s²(刹车距离 = V²/(2a) ≈ 30°) */
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
    /* 世界坐标系锁定(方向锁定)状态:1=平移按世界系解析 */
    bool    world_lock;       /* 世界系是否激活 */
    float   world_lock_yaw;   /* 世界系基准航向(deg,打开瞬间的 IMU 航向) */
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

/* 世界坐标系锁定:lock = 1 时把 vx/vy 参考系切到"世界系"(以打开瞬间车身朝向为 +x)。
   每次 lock 由 0->1(上升沿)用 yaw_deg 重新冻结基准;仅作用于平移,wz 不受影响。 */
void Chassis_SetWorldLock(Chassis_t *chassis, bool lock, float yaw_deg);
bool Chassis_GetWorldLock(const Chassis_t *chassis);

/* 底盘四轮速度环 P/I 调参(蓝牙手调):把 vel_p/vel_i(量程[0,100])写给每个绑定的电调速度环。
   只写 RAM(立即生效),不触发 flash 保存;不要求底盘使能,掉电即回调到电调默认值。 */
void Chassis_TuneWheelPID(Chassis_t *chassis, float vel_p, float vel_i);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H */
