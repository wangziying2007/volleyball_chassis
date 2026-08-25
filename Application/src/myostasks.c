/**
 * @file    tasks.c
 * @brief   任务函数实现(强定义覆盖 freertos.c 里的 __weak 占位)。
 *
 * 新建任务的标准流程:
 *   1. CubeMX 里创建任务(生成 freertos.c 的任务创建代码和 __weak 任务函数);
 *   2. 在 CubeMX 任务函数的 USER CODE 里写业务,或者在本文件写强定义覆盖。
 */
#include "includes.h"
#include "myostasks.h"
#include "app_config.h"
#include "board_config.h"

#include "bsp_led.h"
#include "vofa.h"
#include "bluetooth_serv.h" /* 含 BT_SERV_ENABLE 开关,默认 0 */
#include "ZDrive.h"         /* Zmotor[] / Zdrive_Speed / Zdrive_Disable */
#include "chassis.h"        /* 全向轮底盘业务组件 */

/* CubeMX 默认任务 LEDTask:四灯流水 */
void Alarm_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        for (uint8_t i = 0; i < BOARD_LED_NUM; i++)
        {
            BspLed_On(i);
            osDelay(APP_LED_TASK_PERIOD_MS);
            BspLed_Off(i);
            osDelay(APP_LED_TASK_PERIOD_MS);
        }
    }
}

void VOFA_SendTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        static uint8_t Func_cnt = 0;
        Func_cnt++;
        VOFA_Channel_Update(0, VOFA_TYPE_UINT8, (void *)&(Func_cnt));
#if APP_VOFA_ENABLE
        VOFA_Update();
#endif
        osDelay(APP_VOFA_TASK_PERIOD_MS); /* 固定 10ms:底盘 100Hz 更新(VOFA 关闭时也保持) */
    }
}

void Chassis_Task(void *argument)
{
    (void)argument;
    static Chassis_t chassis;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); 

    /* 自动绑定 Zmotor[0..3],顺序 = 右手系象限顺序(见 chassis.h) */
    Chassis_Init(&chassis);
    for (;;)
    {
        // (void)BT_Serv_Status_Send(&status);
        BT_Cmd_t cmd;
        if (BT_Serv_Cmd_Receive(&cmd, 0))
        {
            if (cmd.reset)
            {
                /* reset 按钮 → 整板复位 */
                __disable_irq();
                NVIC_SystemReset();
            }
            else
            {
                if (cmd.disable)
                {
                    Chassis_Disable(&chassis);
                    for (uint8_t i = 0; i < 4U; i++)
                        Zmotor[i].mode = Zdrive_Disable;
                }
                else if (cmd.enable)
                {
                    Chassis_Enable(&chassis);
                    for (uint8_t i = 0; i < 4U; i++)
                        Zmotor[i].mode = Zdrive_Speed;
                }
                /* 每帧更新目标速度(使能后连续可调) */
                Chassis_SetVelocity(&chassis, cmd.vx, cmd.vy, cmd.wz);
            }
        }
        /* 周期解算:使能时按 vx/vy/wz 算轮速写入 Zmotor[].valSetNow.speed_rpm,
           失能时全轮置 0;实际 CAN 发送由 TIM2 里 ZdriveFunc() 完成 */
        Chassis_Update(&chassis);
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}