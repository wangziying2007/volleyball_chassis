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
    /* 航向保持闭环(级联外环)初始化 */
    PID_Init(&chassis->heading_pid, CHASSIS_HEADING_KP, CHASSIS_HEADING_KI, CHASSIS_HEADING_KD, PIDPOS);
    chassis->hold_yaw = 0.0f;
    chassis->heading_lock_init = false;
    /* 级联内环(速度闭环)初始化 */
    PID_Init(&chassis->yaw_ctrl.pid_vel, CHASSIS_YAW_KP_VEL, CHASSIS_YAW_KI_VEL, CHASSIS_YAW_KD_VEL, PIDPOS);
    chassis->yaw_ctrl.wz_real   = 0.0f;
    chassis->yaw_ctrl.wz_target = 0.0f;
    chassis->yaw_ctrl.wz_cmd    = 0.0f;
    /* 世界坐标系锁定:默认关(车身系),基准清 0 */
    chassis->world_lock = false;
    chassis->world_lock_yaw = 0.0f;
    /* 定点转向状态清零 */
    chassis->turn_pending = false;
    chassis->turn_dir = 0;
    chassis->turn_active = false;
    chassis->turn_target_yaw = 0.0f;
    chassis->turn_done_hold = 0U;
    chassis->turn_elapsed = 0U;
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

/* 速度输入二次映射(一条曲线):低端细腻、高端冲到 MAP_MAX+MAP_BOOST=2000。
   输入已 clamp 到 ±MAP_MAX,输出为映射后的速度;档位缩放由调用方乘以 scale(见 Chassis_SetVelocity)。 */
static float Chassis_SpeedMap(float v)
{
    const float sign = (v < 0.0f) ? -1.0f : 1.0f;
    const float t = (v < 0.0f) ? -v : v;
    const float u = t / CHASSIS_MAP_MAX;

    return sign * (CHASSIS_MAP_MAX * u * u + CHASSIS_MAP_BOOST * u * u);
}

/* 入参为操作手原始 vx / vy / wz(mm/s、°/s):
   vx/vy 先 clamp 到映射满刻度,再过二次映射,最后乘档位缩放(顶级 fast 2000 / slow 500);
   wz 保持线性 + 档位缩放(独立刻度)。操作手帧→底盘帧的 180° 旋转(CHASSIS_OP_FRAME_SIGN)
   在 Chassis_Update 统一应用(车身 + 世界系两种模式都要)。
   换档立即生效、无平滑(人工闭环要跟手);不做加减速斜坡。 */
void Chassis_SetVelocity(Chassis_t *chassis, float vx, float vy, float wz)
{
    float scale;
    float vx_raw, vy_raw, wz_raw; /* 摇杆原始命令(mm/s、°/s) */

    if (chassis == NULL)
    {
        return;
    }

    scale = chassis->fast ? CHASSIS_MODE_FAST_SCALE : CHASSIS_MODE_SLOW_SCALE;
    vx_raw = Clamp(vx, -CHASSIS_VX_MAX_MM_S, CHASSIS_VX_MAX_MM_S);
    vy_raw = Clamp(vy, -CHASSIS_VY_MAX_MM_S, CHASSIS_VY_MAX_MM_S);
    wz_raw = Clamp(wz, -CHASSIS_WZ_MAX_DEG_S, CHASSIS_WZ_MAX_DEG_S);

    chassis->vx = Chassis_SpeedMap(vx_raw) * scale; /* mm/s(顶级 fast 2000 / slow 500) */
    chassis->vy = Chassis_SpeedMap(vy_raw) * scale; /* mm/s */
    chassis->wz = wz_raw * scale;                   /* deg/s */
}

void Chassis_Enable(Chassis_t *chassis)
{
    chassis->enable = true;
    chassis->heading_lock_init = false; /* 重新使能后,首次航向保持重新锁定当前方向 */
    chassis->turn_pending = false;      /* 重新使能:清掉任何残留转向请求/状态 */
    chassis->turn_active = false;
    PID_Reset(&chassis->yaw_ctrl.pid_vel); /* 重新使能:清内环累积 */
}

void Chassis_Disable(Chassis_t *chassis)
{
    chassis->enable = false;
    chassis->turn_pending = false; /* 失能/急停:取消定点转向 */
    chassis->turn_active = false;
    PID_Reset(&chassis->yaw_ctrl.pid_vel); /* 失能:清内环累积 */
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
    chassis->turn_pending = false; /* 复位速度目标:一并取消定点转向 */
    chassis->turn_active = false;
    PID_Reset(&chassis->yaw_ctrl.pid_vel); /* 复位:清内环累积 */
    for (i = 0; i < 4U; i++)
    {
        Wheel_SetSpeed(&chassis->wheels[i], 0.0f);
    }
}

/* 定点转向:dir = +1 左转90° / -1 右转90°(目标角增量方向)。
   仅在底盘使能时接受;转进行中忽略新请求(转完自动重新武装,见 Chassis_EffWz)。 */
void Chassis_RequestTurn(Chassis_t *chassis, int8_t dir)
{
    if (chassis == NULL)
    {
        return;
    }
    if (!chassis->enable) /* 失能/掉线急停时不接受(与自动转前提一致) */
    {
        return;
    }
    chassis->turn_pending = true;
    chassis->turn_dir = (dir >= 0) ? +1 : -1;
}

/* 世界坐标系锁定:lock 由 0->1 上升沿时,用 yaw_deg 冻结世界系基准(对齐车身当前朝向)。
   只改平移参考帧;车身朝向稳定(航向保持/定点转向)不受影响。 */
void Chassis_SetWorldLock(Chassis_t *chassis, bool lock, float yaw_deg)
{
    if (chassis == NULL)
    {
        return;
    }
    if (lock && !chassis->world_lock) /* 上升沿:重新冻结世界系,对齐当前车身朝向 */
    {
        chassis->world_lock_yaw = yaw_deg;
    }
    chassis->world_lock = lock;
}

bool Chassis_GetWorldLock(const Chassis_t *chassis)
{
    return (chassis != NULL) && chassis->world_lock;
}

/* 底盘四轮速度环 P/I 调参(蓝牙手调):clamp 到电调量程,写给每个绑定的电机。
   只写 RAM(立即生效),不触发 flash 保存;掉电即回调到电调默认值(见 ZDrive 手册)。 */
void Chassis_TuneWheelPID(Chassis_t *chassis, float vel_p, float vel_i)
{
    uint8_t i;

    if (chassis == NULL)
    {
        return;
    }

    const float vp = Clamp(vel_p, 0.0f, 100.0f);
    const float vi = Clamp(vel_i, 0.0f, 100.0f); 

    for (i = 0; i < 4U; i++)
    {
        Zdrive *m = chassis->wheels[i].motor;
        if (m == NULL)
        {
            continue;
        }
        m->param.kpVel = vp;
        m->param.kiVel = vi;
        ZdriveSet(vp, m->param.zdrive_id, Vel_PID_P);
        ZdriveSet(vi, m->param.zdrive_id, Vel_PID_I);
    }
}

/* 级联内环(速度 PID):把外环目标角速度(yaw 域)与 IMU 实测角速度的误差 → wz 命令(kinematics 域)。
   yaw_target 由外环给出;in_deadband = true 时输出归 0(防到位抖动/爬行)、清内环累积;
   out_limit 为最终输出限幅(°/s):定向转向用 WZ_MAX、航向保持用 HEADING_OUTPUT_LIMIT(温和)。 */
static float Chassis_YawInnerLoop(Chassis_t *chassis, float yaw_target, float wz_real,
                                  bool in_deadband, float out_limit)
{
    YawCtrl_t *y = &chassis->yaw_ctrl;
    float wz_cmd_yaw;

    y->wz_real   = wz_real;
    y->wz_target = yaw_target;

    if (in_deadband)
    {
        /* 位置误差在死区内:内环输出归 0、清历史,防到位后的微调/抖动/爬行 */
        PID_Reset(&y->pid_vel);
        y->wz_cmd = 0.0f;
        return 0.0f;
    }

    /* yaw 域:PID 目标 = 外环规划的目标角速度,反馈 = IMU 实测角速度(同一符号约定) */
    y->pid_vel.SetVal = yaw_target;
    y->pid_vel.CurVal = wz_real;
    wz_cmd_yaw = ClampPeak(PID_Calculate(&y->pid_vel), out_limit);
    /* 转 kinematics 域:方向共用 CHASSIS_HEADING_SIGN(与既有航向保持/转向一致) */
    y->wz_cmd = CHASSIS_HEADING_SIGN * wz_cmd_yaw;
    return y->wz_cmd;
}

/* 计算用于运动学的有效 wz(操作手 wz、定点转向或航向保持),经级联内环产生。
   yaw_deg = IMU 航向(deg,相对上电零位);wz_real = IMU 实测 yaw 角速度(deg/s,级联内环反馈)。 */
static float Chassis_EffWz(Chassis_t *chassis, float yaw_deg, float wz_real)
{
    /* 手动转向优先:操作手 wz 超死区 → 取消一切自动(定点转向/航向保持),回到手动,
       并锁到当前航向,释放后稳在最近角度 */
    if (fabsf(chassis->wz) > CHASSIS_WZ_HOLD_DEADBAND_DEG_S)
    {
        chassis->turn_pending = false;
        chassis->turn_active = false;
        chassis->hold_yaw = yaw_deg;
        chassis->heading_lock_init = false;
        PID_Reset(&chassis->yaw_ctrl.pid_vel); /* 手动:不经级联,清内环累积 */
        return chassis->wz;
    }

    /* 定点转向:有请求 → 抓当前航向为起点,朝目标角走(目标角增量 = dir * 90°) */
    if (chassis->turn_pending)
    {
        chassis->turn_pending = false;
        chassis->turn_active = true;
        chassis->turn_target_yaw = yaw_deg
                                   + CHASSIS_TURN_LEFT_DIR * chassis->turn_dir * CHASSIS_TURN_ANGLE_DEG;
        PID_Reset(&chassis->yaw_ctrl.pid_vel);
        chassis->turn_elapsed = 0U;
        chassis->turn_done_hold = 0U;
    }

    /* 正在执行定点转向:外环 = 梯形速度剖面 → 目标角速度;到位(连续 N tick 容差内)或超时即结束,
       停、锁当前航向、重新武装下一指令 */
    if (chassis->turn_active)
    {
        float err = Chassis_WrapDeg(chassis->turn_target_yaw - yaw_deg);
        float brake;
        float mag;
        float yaw_target;

        chassis->turn_elapsed++;

        if (fabsf(err) < CHASSIS_TURN_DONE_TOL_DEG)
        {
            chassis->turn_done_hold++;
        }
        else
        {
            chassis->turn_done_hold = 0U;
        }

        if ((chassis->turn_done_hold >= CHASSIS_TURN_DONE_HOLDS)
            || (chassis->turn_elapsed >= CHASSIS_TURN_TIMEOUT_TICKS))
        {
            chassis->turn_active = false;
            chassis->hold_yaw = yaw_deg;     /* 锁住当前航向,继续航向保持 */
            PID_Reset(&chassis->heading_pid);
            chassis->heading_lock_init = true;
            /* 本轮落回航向保持(误差≈0,输出≈0,平滑续接) */
        }
        else
        {
            /* 梯形速度剖面:大误差巡航,接近目标时恒减速到位(不拖尾、不超调)。
               方向 = err 符号(yaw 域,正 = 让 IMU 航向增大);最终方向由内环 ×HEADING_SIGN 统一。 */
            brake = CHASSIS_TURN_SPEED_DEG_S * CHASSIS_TURN_SPEED_DEG_S
                  / (2.0f * CHASSIS_TURN_DECEL_DEG_S2); /* 刹车距离(deg) */
            mag = (fabsf(err) >= brake)
                      ? CHASSIS_TURN_SPEED_DEG_S                      /* 巡航 */
                      : sqrtf(2.0f * CHASSIS_TURN_DECEL_DEG_S2 * fabsf(err)); /* 恒减速 */
            yaw_target = (err >= 0.0f) ? mag : -mag;
            return Chassis_YawInnerLoop(chassis, yaw_target, wz_real,
                                        fabsf(err) < CHASSIS_YAW_DEADBAND_DEG,
                                        CHASSIS_WZ_MAX_DEG_S);
        }
    }

    /* 进入航向保持:首次锁到当前航向,清掉 PID 累积 */
    if (!chassis->heading_lock_init)
    {
        chassis->hold_yaw = yaw_deg;
        PID_Reset(&chassis->heading_pid);
        chassis->heading_lock_init = true;
    }

    /* 航向保持:外环 = 位置 PID(温和修正,不猛拉),输出 yaw 域目标角速度;
       目标角折到 yaw 附近(避免 ±180 跳变)。内环限幅到温和上限(不猛拉)。 */
    {
        float err = Chassis_WrapDeg(chassis->hold_yaw - yaw_deg);
        float yaw_target;

        chassis->heading_pid.SetVal = yaw_deg + err;   /* 目标角折到 yaw 附近 */
        chassis->heading_pid.CurVal = yaw_deg;
        yaw_target = PID_Calculate(&chassis->heading_pid);
        return Chassis_YawInnerLoop(chassis, yaw_target, wz_real,
                                    fabsf(err) < CHASSIS_YAW_DEADBAND_DEG,
                                    CHASSIS_HEADING_OUTPUT_LIMIT_DEG_S);
    }
}

void Chassis_Update(Chassis_t *chassis, float yaw_deg, float wz_deg_s)
{
    float w_rad; /* eff_wz:deg/s -> rad/s */
    float r;     /* 轮半径,mm */
    float L;     /* 等效回转半径,mm */
    float omega_fl, omega_fr, omega_rl, omega_rr;
    float vx, vy; /* 车身系平移速度:mm/s(world_lock 时由世界系旋转而来) */

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
    w_rad = Chassis_EffWz(chassis, yaw_deg, wz_deg_s) * (PI / 180.0f);

    /* 操作手命令帧(body 意图 / 世界系 +x)相对底盘运动学帧固定差 180°(整车装反):
       vx/vy 恒取反。两种模式都要 —— 世界系锁定只是再按实时航向把取反后的命令转回车身系;
       `rel` 只测车身转动变化量(锁定时从 0 起),不含这固定 180° 静态偏移,故不能只在 body 模式取反。 */
    vx = chassis->vx * CHASSIS_OP_FRAME_SIGN;
    vy = chassis->vy * CHASSIS_OP_FRAME_SIGN;

    /* 世界坐标系锁定:开启时把 vx/vy(已取反的世界系命令)旋转到车身系再解算;
       wz 保持车体相对(天然如此,不受影响)。rel = 车身当前朝向 - 世界系基准朝向。 */
    if (chassis->world_lock)
    {
        float rel = Chassis_WrapDeg(yaw_deg - chassis->world_lock_yaw);
        Rotate(&vx, &vy, 0.0f, 0.0f, CHASSIS_DIR_LOCK_SIGN * rel);
    }

    /* 运动学直接展开(真全向轮 45° 斜装,统一乘 1/√2 修正系数) */
    omega_fl = (vx + vy + w_rad * L) / r * CHASSIS_OMNI_45_SCALE;
    omega_fr = (-vx + vy + w_rad * L) / r * CHASSIS_OMNI_45_SCALE;
    omega_rl = (vx - vy + w_rad * L) / r * CHASSIS_OMNI_45_SCALE;
    omega_rr = (-vx - vy + w_rad * L) / r * CHASSIS_OMNI_45_SCALE;

    /* rad/s -> rpm,按象限顺序写入各轮 */
    Wheel_SetSpeed(&chassis->wheels[CHASSIS_WHEEL_FL], omega_fl * CHASSIS_RPM_PER_RAD_S);
    Wheel_SetSpeed(&chassis->wheels[CHASSIS_WHEEL_FR], omega_fr * CHASSIS_RPM_PER_RAD_S);
    Wheel_SetSpeed(&chassis->wheels[CHASSIS_WHEEL_RR], omega_rr * CHASSIS_RPM_PER_RAD_S);
    Wheel_SetSpeed(&chassis->wheels[CHASSIS_WHEEL_RL], omega_rl * CHASSIS_RPM_PER_RAD_S);
}
