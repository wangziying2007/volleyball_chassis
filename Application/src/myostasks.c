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
#include "imu.h"            /* IMU 姿态读取(航向保持) */
#include "mech.h"           /* 机构(CAN1)业务组件:Mech_PollButtons / Mech_GetStatus */

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
        static ImuData_t imu;
        Func_cnt++;
        VOFA_Channel_Update(0, VOFA_TYPE_UINT8, (void *)&(Func_cnt));
        /* IMU 打波:帧计数看链路是否持续收帧;数值看静止/转动是否合理 */
        if (Imu_Receive(&imu, 0))
        {
            VOFA_Channel_Update(1, VOFA_TYPE_FLOAT, (void *)&imu.body_orientation[2]); /* yaw,deg */
            VOFA_Channel_Update(2, VOFA_TYPE_FLOAT, (void *)&imu.body_rate[0]);        /* roll 角速度 */
            VOFA_Channel_Update(3, VOFA_TYPE_FLOAT, (void *)&imu.body_rate[2]);        /* yaw 角速度 */
            VOFA_Channel_Update(4, VOFA_TYPE_UINT32, (void *)&imu.frame_count);        /* 帧计数:链路通=持续上涨 */
            VOFA_Channel_Update(5, VOFA_TYPE_FLOAT, (void *)&imu.body_accel[2]);       /* accel_z ≈ ±9.8 */
        }
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
    static float last_yaw = 0.0f;        /* IMU 最近航向(deg),给航向保持/定点转向 + 状态回传 */
    static bool prev_turn_left = false;  /* 左转按钮上一帧状态(边沿检测) */
    static bool prev_turn_right = false; /* 右转按钮上一帧状态(边沿检测) */

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);
    TickType_t last_cmd_tick = xTaskGetTickCount(); /* 最近一次收到底盘命令的时刻,用于掉线超时急停 */

    /* 自动绑定 Zmotor[0..3],顺序 = 右手系象限顺序(见 chassis.h) */
    Chassis_Init(&chassis);
    Mech_Init();
    for (;;)
    {
        TickType_t now = xTaskGetTickCount();

        /* 读 IMU 最新航向(°,相对上电零位):给航向保持/定点转向/世界系锁定 + 状态回传用 */
        ImuData_t imu;
        if (Imu_Receive(&imu, 0))
        {
            last_yaw = imu.body_orientation[2];
        }

        BT_Cmd_t cmd;
        if (BT_Serv_Cmd_Receive(&cmd, 0))
        {
            last_cmd_tick = now;
            if (cmd.reset)
            {
                /* reset 按钮 → 整板复位 */
                __disable_irq();
                NVIC_SystemReset();
            }
            else
            {
                /* 模式档切换:1=快速 / 0=慢速(蓝牙触发,无论是否使能都先落档,立即生效) */
                Chassis_SetMode(&chassis, cmd.fast);

                /* 世界坐标系锁定:1=世界系 / 0=车身系;上升沿在内部重新冻结世界系基准 */
                Chassis_SetWorldLock(&chassis, cmd.lock, last_yaw);

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
                /* 每帧更新目标速度(使能后连续可调;内部先按当前档缩放、再限幅) */
                Chassis_SetVelocity(&chassis, cmd.vx, cmd.vy, cmd.wz);

                /* 定点转向:按钮边沿 → RequestTurn(相对当前航向 ±90°)。
                   边沿用上一帧状态判沿(与机构按键同模式);Chassis_RequestTurn 内部仅在使能时生效。 */
                bool turnL_edge = cmd.turn_left90 && !prev_turn_left;
                bool turnR_edge = cmd.turn_right90 && !prev_turn_right;
                prev_turn_left  = cmd.turn_left90;
                prev_turn_right = cmd.turn_right90;
                if (turnL_edge)
                {
                    Chassis_RequestTurn(&chassis, +1); /* +1 = 左转90° */
                }
                else if (turnR_edge)
                {
                    Chassis_RequestTurn(&chassis, -1); /* -1 = 右转90° */
                }

                /* 机构按键:边沿检测 → 暂存 CAN1 命令(实际发送由 TIM2 里 Mech_Func 完成) */
                MechButtons_t mb = {
                    .enable        = cmd.mech_enable,
                    .disable       = cmd.mech_disable,
                    .reset         = cmd.mech_reset,
                    .left_work1    = cmd.left_work1,
                    .left_work2    = cmd.left_work2,
                    .right_work1   = cmd.right_work1,
                    .right_work2   = cmd.right_work2,
                    .left_pulldown = cmd.left_pulldown,
                    .right_pulldown = cmd.right_pulldown,
                    .grip_open     = cmd.grip_open,
                    .grip_close    = cmd.grip_close,
                    .grip_release  = cmd.grip_release,
                    .grip_zero     = cmd.grip_zero,
                };
                Mech_PollButtons(&mb);
            }
        }

        /* 蓝牙掉线超时急停:使能状态下若超过 BT_CMD_TIMEOUT_MS 未收到任何新命令,
           判定链路断开 → 底盘立即停止(全轮 Disable)。收到新命令后由 enable 分支恢复。 */
        if (Chassis_IsEnabled(&chassis)
            && ((now - last_cmd_tick) >= pdMS_TO_TICKS(BT_CMD_TIMEOUT_MS)))
        {
            Chassis_Disable(&chassis);
            for (uint8_t i = 0; i < 4U; i++)
                Zmotor[i].mode = Zdrive_Disable;
        }

#if BT_SERV_ENABLE
        /* 周期回传状态(enable + fast + world_lock + 机构状态 + yaw 航向遥测),每 BT_SERV_STATUS_PERIOD_MS 一次 */
        static TickType_t last_status_tick = 0U;
        if ((now - last_status_tick) >= pdMS_TO_TICKS(BT_SERV_STATUS_PERIOD_MS))
        {
            BT_Status_t status;
            status.enable      = Chassis_IsEnabled(&chassis);
            status.fast        = Chassis_GetFast(&chassis);
            status.lock_active = Chassis_GetWorldLock(&chassis);
            status.mech_status = Mech_GetStatus();
            status.yaw         = last_yaw; /* 实时航向遥测,即使未使能也回传(停着也能看朝向) */
            BT_Serv_Status_Send(&status);
            last_status_tick = now;
        }
#endif

        /* 周期解算:使能时按 vx/vy/wz 算轮速写入 Zmotor[].valSetNow.speed_rpm,
           失能时全轮置 0;实际 CAN 发送由 TIM2 里 ZdriveFunc() 完成。
           传入 yaw_deg 后,底盘在 wz≈0 时用 IMU 维持航向(纠正漂移/跑偏) */
        Chassis_Update(&chassis, last_yaw);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}