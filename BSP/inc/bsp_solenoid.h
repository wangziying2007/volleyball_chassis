/**
 * @file bsp_solenoid.h
 * @date 2026-08-21
 */

//  配置：GPIO输出 利用队里F4板上的串口的4Pin口
// （RX为SCL  TX为SDA）
// 一个电磁阀板控制4个电磁阀，需要4位
// （分高四位或者低四位）
#ifndef __BSP_SOLENOID_H
#define __BSP_SOLENOID_H

#define SOLENOID_VALVE_CHANNEL_1 ((uint8_t)0x01)
#define SOLENOID_VALVE_CHANNEL_2 ((uint8_t)0x02)
#define SOLENOID_VALVE_CHANNEL_3 ((uint8_t)0x04)
#define SOLENOID_VALVE_CHANNEL_4 ((uint8_t)0x08)

#include "main.h"
#include "board_config.h"

typedef struct Solenoid_t
{
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin_sda;
    uint16_t gpio_pin_clk;
    uint8_t data_prve;
} Solenoid_t;

typedef enum
{
    Solenoid_Bus1 = 1,
    Solenoid_Bus2,
    Solenoid_Bus3,
} Solenoid_Bus;

void SolenoidValve_Init(Solenoid_Bus usart_bus);
void SolenoidValve_On(Solenoid_Bus usart_bus, uint8_t cmd);
uint8_t SolenoidValve_Read(Solenoid_Bus usart_bus);
#endif /* __BSP_SOLENOID_H */