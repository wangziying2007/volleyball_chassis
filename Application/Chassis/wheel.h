/**
 * @file    wheel.h
 * @brief   全向轮底盘 - 单轮抽象(Application 层业务组件)。
 *
 * 职责:一个 Wheel 绑定一个 ZDrive 电机,只负责把目标转速写入电机的
 *       valSetNow.speed_rpm 字段。不负责 PID、不负责通信(CAN/UART 发送
 *       由 Motor 层 ZdriveFunc 周期完成)、不负责使能/模式切换。
 */
#ifndef WHEEL_H
#define WHEEL_H

#include "ZDrive.h" /* Zdrive 类型 / Zmotor[] 声明 */

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    Zdrive *motor;     /* 绑定的 ZDrive 电机指针;NULL = 未绑定,SetSpeed 自动跳过 */
    float direction;   /* 方向修正:+1.0f 正向 / -1.0f 反向;装反的电机在此修正,不写死在底盘运动学里 */
} Wheel_t;

void Wheel_Init(Wheel_t *wheel);
void Wheel_Bind(Wheel_t *wheel, Zdrive *motor);
void Wheel_SetSpeed(Wheel_t *wheel, float rpm);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_H */
