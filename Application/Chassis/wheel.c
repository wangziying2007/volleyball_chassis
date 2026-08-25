/**
 * @file    wheel.c
 * @brief   全向轮底盘 - 单轮抽象实现。
 *
 * 只写电机目标速度字段,不调用任何 CAN/UART 发送函数;
 * 实际发送节奏由 Motor 层 ZdriveFunc()(TIM2 200Hz)按
 * Zmotor[i].mode / valSetNow.speed_rpm 自行处理。
 */
#include "wheel.h"

void Wheel_Init(Wheel_t *wheel)
{
    wheel->motor     = NULL;
    wheel->direction = 1.0f;
}

void Wheel_Bind(Wheel_t *wheel, Zdrive *motor)
{
    wheel->motor = motor;
    motor->Begin = true; 
}

/* 直接写 setNow 速度字段(单位 rpm),不做任何发送 */
void Wheel_SetSpeed(Wheel_t *wheel, float rpm)
{
    if ((wheel == NULL) || (wheel->motor == NULL))
    {
        return;
    }
    wheel->motor->valSetNow.speed_rpm = rpm * wheel->direction;
}
