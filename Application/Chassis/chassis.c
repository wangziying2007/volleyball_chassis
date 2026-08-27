/**
 * @file    chassis.c
 * @brief   四轮全向底盘运动学解算实现。
 *
 * 长度单位 mm:速度 mm/s、轮半径 mm、L mm,omega 为 rad/s。
 */
#include "chassis.h"
#include "mathFunc.h" /* PI / Clamp */
#include "ZDrive.h"   /* Zdrive 类型 / Zmotor[] 声明 */
#include "pid.h"      /* PIDType / PID_Init / PID_Calculate / PID_Reset */

/* rad/s -> rpm */
#define CHASSIS_RPM_PER_RAD_S (60.0f / (2.0f * PI))

#define CHASSIS_OMNI_45_SCALE 0.70710678f /* 1/sqrt(2) */

/* 角度归一到 [-180, 180](deg),供航向误差使用,避免 ±180 跳变抽筋 */
static inline float Chassis_WrapDeg(float deg)
{
    deg = fmodf(deg, 360.0f);
    if (deg > 180.0f)
        deg -= 360.0f;
    else if (deg < -180.0f)
        deg += 360.0f;
    return deg;
}

void Chassis_Init(Chassis_t *chassis)
{
    uint8_t i;

    for (i = 0; i < 4U; i++)
    {
        Wheel_Init(&chassis->wheels[i]);
#if USE_ZMDR
        /* 自动绑定 ZDrive 0~3,顺序 = 右手系象限顺序:
           Zmotor[0]->FL, [1]->FR, [2]->RR, [3]->RL */
        Wheel_Bind(&chassis->wheels[i], &Zmotor[i]);
#endif
    }
    chassis->enable = false;
    chassis->reset = false;
    chassis->fast = true; /* 默认快速档(慢速为辅助微调用) */
    chassis->vx = 0.0f;
    chassis->vy = 0.0f;
    chassis->wz = 0.0f;
    /* 航向保持闭环初始化 */
    PID_Init(&chassis->heading_pid, CHASSIS_HEADING_KP, CHASSIS_HEADING_KI, CHASSIS_HEADING_KD, PIDPOS);
    chassis->hold_yaw = 0.0f;
    chassis->heading_lock_init = false;
}

void Chassis_BindWheel(Chassis_t *chassis, uint8_t index, Zdrive *motor)
{
    if (index >= 4U)
    {
        return;
    }
    Wheel_Bind(&chassis->wheels[index], motor);
}

/* 电机装反修正:direction = -1.0f 反转该轮 */
void Chassis_SetWheelDirection(Chassis_t *chassis, uint8_t index, float direction)
{
    if (index >= 4U)
    {
        return;
    }
    chassis->wheels[index].direction = direction;
}

/* 入参为操作手原始 vx / vy / wz(mm/s、°/s),内部先按当前模式档缩放、再限幅。
   换档立即生效、无平滑(人工闭环要跟手);不做加减速斜坡。 */
void Chassis_SetVelocity(Chassis_t *chassis, float vx, float vy, float wz)
{
    float scale;

    if (chassis == NULL)
    {
        return;
    }

    scale = chassis->fast ? CHASSIS_MODE_FAST_SCALE : CHASSIS_MODE_SLOW_SCALE;
    vx = Clamp(vx * scale, -CHASSIS_VX_MAX_MM_S, CHASSIS_VX_MAX_MM_S);
    vy = Clamp(vy * scale, -CHASSIS_VY_MAX_MM_S, CHASSIS_VY_MAX_MM_S);
    wz = Clamp(wz * scale, -CHASSIS_WZ_MAX_DEG_S, CHASSIS_WZ_MAX_DEG_S);

    chassis->vx = vx; /* mm/s */
    chassis->vy = vy; /* mm/s */
    chassis->wz = wz; /* deg/s */
}

void Chassis_Enable(Chassis_t *chassis)
{
    chassis->enable = true;
    chassis->heading_lock_init = false; /* 重新使能后,首次航向保持重新锁定当前方向 */
}

void Chassis_Disable(Chassis_t *chassis)
{
    chassis->enable = false;
}

/* 设置模式档:fast = 1 快速 / 0 慢速(仅改缩放,不影响使能状态) */
void Chassis_SetMode(Chassis_t *chassis, bool fast)
{
    if (chassis != NULL)
    {
        chassis->fast = fast;
    }
}

bool Chassis_IsEnabled(const Chassis_t *chassis)
{
    return (chassis != NULL) && chassis->enable;
}

bool Chassis_GetFast(const Chassis_t *chassis)
{
    return (chassis != NULL) && chassis->fast;
}

/* 清空速度目标,并使所有轮子立即停止 */
void Chassis_Reset(Chassis_t *chassis)
{
    uint8_t i;

    chassis->vx = 0.0f;
    chassis->vy = 0.0f;
    chassis->wz = 0.0f;
    for (i = 0; i < 4U; i++)
    {
        Wheel_SetSpeed(&chassis->wheels[i], 0.0f);
    }
}

/* 计算用于运动学的有效 wz(操作手 wz 或航向保持修正量)。
   yaw_deg = IMU 航向(deg,相对上电零位)。 */
static float Chassis_EffWz(Chassis_t *chassis, float yaw_deg)
{
    /* wz 超过死区 → 操作手在转向,航向保持失效:锁定到当前航向,释放后稳在最近角度 */
    if (fabsf(chassis->wz) > CHASSIS_WZ_HOLD_DEADBAND_DEG_S)
    {
        chassis->hold_yaw = yaw_deg;
        chassis->heading_lock_init = false;
        return chassis->wz;
    }

    /* 进入航向保持:首次锁到当前航向,清掉 PID 累积 */
    if (!chassis->heading_lock_init)
    {
        chassis->hold_yaw = yaw_deg;
        PID_Reset(&chassis->heading_pid);
        chassis->heading_lock_init = true;
    }

    /* 目标角折到 yaw 附近(避免 ±180 跳变),位置式 PID 输出角速度修正量 */
    chassis->heading_pid.SetVal = yaw_deg + Chassis_WrapDeg(chassis->hold_yaw - yaw_deg);
    chassis->heading_pid.CurVal = yaw_deg;
    return Clamp(CHASSIS_HEADING_SIGN * PID_Calculate(&chassis->heading_pid),
                 -CHASSIS_HEADING_OUTPUT_LIMIT_DEG_S, CHASSIS_HEADING_OUTPUT_LIMIT_DEG_S);
}

void Chassis_Update(Chassis_t *chassis, float yaw_deg)
{
    float w_rad; /* eff_wz:deg/s -> rad/s */
    float r;     /* 轮半径,mm */
    float L;     /* 等效回转半径,mm */
    float omega_fl, omega_fr, omega_rl, omega_rr;

    if (chassis == NULL)
    {
        return;
    }
    if (!chassis->enable)
    {
        /* 失能时清零速度目标,并使所有轮子立即停止 */
        Chassis_Reset(chassis);
        return;
    }

    r = CHASSIS_WHEEL_RADIUS_MM;
    L = CHASSIS_L_MM;
    w_rad = Chassis_EffWz(chassis, yaw_deg) * (PI / 180.0f);

    /* 运动学直接展开(真全向轮 45° 斜装,统一乘 1/√2 修正系数) */
    omega_fl = (chassis->vx + chassis->vy + w_rad * L) / r * CHASSIS_OMNI_45_SCALE;
    omega_fr = (-chassis->vx + chassis->vy + w_rad * L) / r * CHASSIS_OMNI_45_SCALE;
    omega_rl = (chassis->vx - chassis->vy + w_rad * L) / r * CHASSIS_OMNI_45_SCALE;
    omega_rr = (-chassis->vx - chassis->vy + w_rad * L) / r * CHASSIS_OMNI_45_SCALE;

    /* rad/s -> rpm,按象限顺序写入各轮 */
    Wheel_SetSpeed(&chassis->wheels[CHASSIS_WHEEL_FL], omega_fl * CHASSIS_RPM_PER_RAD_S);
    Wheel_SetSpeed(&chassis->wheels[CHASSIS_WHEEL_FR], omega_fr * CHASSIS_RPM_PER_RAD_S);
    Wheel_SetSpeed(&chassis->wheels[CHASSIS_WHEEL_RR], omega_rr * CHASSIS_RPM_PER_RAD_S);
    Wheel_SetSpeed(&chassis->wheels[CHASSIS_WHEEL_RL], omega_rl * CHASSIS_RPM_PER_RAD_S);
}
