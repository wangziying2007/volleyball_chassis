/**
 * @file bsp_solenoid.c
 * @date 2026-08-21
 */
#include "bsp_solenoid.h"

static Solenoid_t solenoid_bus1_cfg = {0};
static Solenoid_t solenoid_bus2_cfg = {0};
static Solenoid_t solenoid_bus3_cfg = {0};

static void _SolenoidValve_Channel_Init(Solenoid_t *solenoid, GPIO_TypeDef *gpio_port, uint16_t gpio_pin_sda, uint16_t gpio_pin_clk)
{
    solenoid->gpio_port = gpio_port;
    solenoid->gpio_pin_sda = gpio_pin_sda;
    solenoid->gpio_pin_clk = gpio_pin_clk;
    solenoid->data_prve = 0xF0;
}
// usart_bus=串口号 不需要在cube中配置 直接调用即可
void SolenoidValve_Init(Solenoid_Bus usart_bus)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    switch (usart_bus)
    {
    case Solenoid_Bus1:
        _SolenoidValve_Channel_Init(&solenoid_bus1_cfg, BOARD_SOLENOID_PORT1, BOARD_SOLENOID_SDA, BOARD_SOLENOID_CLK);
        __HAL_RCC_GPIOA_CLK_ENABLE();
        HAL_GPIO_WritePin(BOARD_SOLENOID_PORT1, BOARD_SOLENOID_SDA | BOARD_SOLENOID_CLK, GPIO_PIN_RESET);
        GPIO_InitStruct.Pin = BOARD_SOLENOID_SDA | BOARD_SOLENOID_CLK;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        break;
    case Solenoid_Bus2:
        _SolenoidValve_Channel_Init(&solenoid_bus2_cfg, BOARD_SOLENOID_PORT2, BOARD_SOLENOID_SDA2, BOARD_SOLENOID_CLK2);
        __HAL_RCC_GPIOD_CLK_ENABLE();
        HAL_GPIO_WritePin(BOARD_SOLENOID_PORT2, BOARD_SOLENOID_SDA2 | BOARD_SOLENOID_CLK2, GPIO_PIN_RESET);
        GPIO_InitStruct.Pin = BOARD_SOLENOID_SDA2 | BOARD_SOLENOID_CLK2;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
        break;
    case Solenoid_Bus3:
        _SolenoidValve_Channel_Init(&solenoid_bus3_cfg, BOARD_SOLENOID_PORT3, BOARD_SOLENOID_SDA3, BOARD_SOLENOID_CLK3);
        __HAL_RCC_GPIOD_CLK_ENABLE();
        HAL_GPIO_WritePin(BOARD_SOLENOID_PORT3, BOARD_SOLENOID_SDA3 | BOARD_SOLENOID_CLK3, GPIO_PIN_RESET);
        GPIO_InitStruct.Pin = BOARD_SOLENOID_SDA3 | BOARD_SOLENOID_CLK3;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
        break;
    default:
        break;
    }
    SolenoidValve_On(usart_bus, 0);
}
static void _SolenoidValve_Register_Update(Solenoid_t *solenoid, uint8_t *data)
{
    if (*data == solenoid->data_prve)
        return;
    solenoid->data_prve = *data;
    for (int i = 0; i < 4; i++)
    {
        if ((*data & 0x08) == 0x08)
        {
            HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_sda, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_sda, GPIO_PIN_RESET);
        }
        *data <<= 1;
        HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_clk, GPIO_PIN_SET);
        HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_clk, GPIO_PIN_RESET);
    }
    HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_clk, GPIO_PIN_SET);
    HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_clk, GPIO_PIN_RESET);
}
// usart_bus=串口号, cmd=命令(低4位控制) 0000 4321 （4321为对应通道）
void SolenoidValve_On(Solenoid_Bus usart_bus, uint8_t cmd)
{
    uint8_t data = cmd & 0x0f;
    switch (usart_bus)
    {
    case Solenoid_Bus1:
        _SolenoidValve_Register_Update(&solenoid_bus1_cfg, &data);
        break;
    case Solenoid_Bus2:
        _SolenoidValve_Register_Update(&solenoid_bus2_cfg, &data);
        break;
    case Solenoid_Bus3:
        _SolenoidValve_Register_Update(&solenoid_bus3_cfg, &data);
        break;
    default:
        break;
    }
}

uint8_t SolenoidValve_Read(Solenoid_Bus usart_bus)
{
    uint8_t data = 0;
    switch (usart_bus)
    {
    case Solenoid_Bus1:
        data = solenoid_bus1_cfg.data_prve;
        break;
    case Solenoid_Bus2:
        data = solenoid_bus2_cfg.data_prve;
        break;
    case Solenoid_Bus3:
        data = solenoid_bus3_cfg.data_prve;
        break;
    default:
        break;
    }
    return data;
}