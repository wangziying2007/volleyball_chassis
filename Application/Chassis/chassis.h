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
    float   vx;        /* 目标纵向速度,mm/s */
    float   vy;        /* 目标横向速度,mm/s */
    float   wz;        /* 目标自转角速度,rad/s */
} Chassis_t;

/* Init 自动绑定 Zmotor[0..3](顺序见文件头);如接线不同可再用 BindWheel 覆盖 */
void Chassis_Init(Chassis_t *chassis);
void Chassis_BindWheel(Chassis_t *chassis, uint8_t index, Zdrive *motor);
void Chassis_SetWheelDirection(Chassis_t *chassis, uint8_t index, float direction);

void Chassis_SetVelocity(Chassis_t *chassis, float vx, float vy, float wz);
void Chassis_Enable(Chassis_t *chassis);
void Chassis_Disable(Chassis_t *chassis);
void Chassis_Reset(Chassis_t *chassis);
void Chassis_Update(Chassis_t *chassis);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H */
