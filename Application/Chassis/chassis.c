/**
 * @file    chassis.c
 * @brief   四轮全向底盘运动学解算实现。
 *
 * 长度单位 mm:速度 mm/s、轮半径 mm、L mm,omega 为 rad/s。
 */
#include "chassis.h"
#include "mathFunc.h" /* PI */
#include "ZDrive.h"   /* Zdrive 类型 / Zmotor[] 声明 */

/* rad/s -> rpm */
#define CHASSIS_RPM_PER_RAD_S (60.0f / (2.0f * PI))

#define CHASSIS_OMNI_45_SCALE 0.70710678f /* 1/sqrt(2) */

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
    chassis->vx = 0.0f;
    chassis->vy = 0.0f;
    chassis->wz = 0.0f;
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

void Chassis_SetVelocity(Chassis_t *chassis, float vx, float vy, float wz)
{
    vx = Clamp(vx, -1800, 1800);
    vy = Clamp(vy, -1800, 1800);
    wz = Clamp(wz, -360, 360);

    chassis->vx = vx; /* mm/s */
    chassis->vy = vy; /* mm/s */
    chassis->wz = wz; /* deg/s */
}

void Chassis_Enable(Chassis_t *chassis)
{
    chassis->enable = true;
}

void Chassis_Disable(Chassis_t *chassis)
{
    chassis->enable = false;
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

void Chassis_Update(Chassis_t *chassis)
{
    float w_rad; /* wz:deg/s -> rad/s */
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
    w_rad = chassis->wz * (PI / 180.0f);

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
