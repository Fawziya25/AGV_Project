#include "yawPIDcontroller.h"
#include "hwt101.h"
#include <stdint.h>

static YawPID_TypeDef yawPID;        /* 角度PID参数与状态(仅本文件使用) */
static YawOffset_TypeDef yawOffset;  /* 直行/转弯偏差记录(仅本文件使用) */
static float omega;                  /* 小车整体角速度 */
static uint8_t TurnCpltFlag = 0;     /* 转弯完成标志位 */
static float target;                 /* 目标航向角 */

// 0:X+ 1:Y+ 2:X- 3:Y-
uint8_t AGV_Direction = 3;           /* 小车朝向 */

static void AGV_StopNow(void);
static void AGV_SetOmega(float omega);
static void YawOffset_Init(void);
/**********************************************************
*** 带门控的陀螺零偏在线估计器
***
*** 原理:车体静止时,角速度均值 ≈ 当前零偏 b;
***      静止窗口内航向读数变化 ≈ 该窗口内累积的纯漂移。
*** 门控:窗口内任何 |wz| > ZB_RATE_GATE 的样本判定为真实运动
***      (机械臂/舵机甩动、底盘蠕动),使窗口作废,防止污染估计。
***
*** 接线(原型已在 yawPIDcontroller.h 中声明):
***   ① ZeroBias_Feed():HWT101 驱动每解析到一帧有效角度帧时自动调用,
***       已移除原TIM7节拍喂入(见 hwt101.c)。
***   ② ZeroBias_Start()/End():上电 HWT101_ManualCal() 内各调用一次,
***       只采集一次静止窗口即得零偏(单窗口,直接用均值,无EMA需求)。
***   ③ ZeroBias_Reset():HWT101_Init() 起始处调用,清估计状态。
*** 本文件内所有控制读数已改用 ZeroBias_GetYaw() 代替 HWT101_GetYaw()。
**********************************************************/

static ZeroBias_TypeDef zeroBias = {0};
static uint32_t ZeroBiasLastFeedTick;   /* 上次Feed时刻(ms),用于运动期连续补偿 */

/**
 * @brief 清零全部状态(上电/发车前调用一次)
 * @retval None
 */
void ZeroBias_Reset(void)
{
    zeroBias.active       = 0;
    zeroBias.dirty        = 0;
    zeroBias.n            = 0U;
    zeroBias.sumWz        = 0.0f;
    zeroBias.startYaw     = 0.0f;
    zeroBias.yawDriftComp = 0.0f;
    zeroBias.b_hat        = 0.0f;
    ZeroBiasLastFeedTick  = HAL_GetTick();
}

/**
 * @brief 开始静止窗口(车停稳后调用;若尚未停稳,窗口会被门控作废,安全)
 * @retval None
 */
void ZeroBias_Start(void)
{
    zeroBias.active   = 1;
    zeroBias.dirty    = 0;
    zeroBias.n        = 0U;
    zeroBias.sumWz    = 0.0f;
    zeroBias.startYaw = HWT101_GetYaw();
}

/**
 * @brief 喂入新IMU数据(主循环收到新数据时调用)
 * @retval None
 * @note 窗口外(运动期)用最新零偏估计持续补偿,消除直行/转弯期间的漂移积累;
 *       窗口内按门控采样,结算时更新b_hat并离散重锚。
 */
void ZeroBias_Feed(void)
{
    float wz;
    uint32_t now, dt;

    now = HAL_GetTick();
    dt  = now - ZeroBiasLastFeedTick;          /* 无符号减法,ms回绕安全 */
    ZeroBiasLastFeedTick = now;

    if(!zeroBias.active)
    {
        /* 窗口外:运动期连续补偿,消除转弯期间b*T_t的漂移积累 */
        zeroBias.yawDriftComp += zeroBias.b_hat * (float)dt / 1000.0f;
        return;
    }

    wz = HWT101_GetWz();
    if(wz > ZB_RATE_GATE || wz < -ZB_RATE_GATE)
    {
        /* 出现真实运动(臂/舵机甩动、底盘蠕动),本窗口作废 */
        zeroBias.dirty = 1;
        return;
    }

    zeroBias.sumWz += wz;
    zeroBias.n++;
}

/**
 * @brief 结束静止窗口并结算(开始移动前调用)
 * @retval None
 */
void ZeroBias_End(void)
{
    float b_win;
    float driftWin;

    if(!zeroBias.active)
    {
        return;
    }
    zeroBias.active = 0;
    ZeroBiasLastFeedTick = HAL_GetTick();   /* 避免窗口尾段被连续补偿重复计数 */

    /* 窗口被污染或样本不足:丢弃,不更新任何估计 */
    if(zeroBias.dirty || zeroBias.n < ZB_MIN_SAMPLES)
    {
        return;
    }

    /* 本窗口零偏估计:角速度均值 */
    b_win = zeroBias.sumWz / (float)zeroBias.n;

    /* 静止期读数变化 = 纯漂移,做±180归一防跨边界 */
    driftWin = HWT101_GetYaw() - zeroBias.startYaw;
    if(driftWin > 180.0f)       driftWin -= 360.0f;
    else if(driftWin < -180.0f) driftWin += 360.0f;

    /* 上电只采集一次窗口,直接用窗口均值(若运行中多次采集才需EMA平滑) */
    zeroBias.b_hat = b_win;

    /* 离散重锚:把本窗口内累积的漂移从读数中扣除 */
    zeroBias.yawDriftComp += driftWin;
}

/**
 * @brief 漂移补偿后的航向角,替代 HWT101_GetYaw() 用于所有控制
 * @retval float 补偿后航向(°)
 */
float ZeroBias_GetYaw(void)
{
    return HWT101_GetYaw() - zeroBias.yawDriftComp;
}

/**
 * @brief 当前零偏估计(°/s),用于直行偏差测量中剔除漂移分量
 * @retval float 漂移率估计值
 */
float ZeroBias_GetBias(void)
{
    return zeroBias.b_hat;
}

/**
 * @brief 初始化角度PID控制器（增益取 yawPIDcontroller.h 的 YAW_PID_KP/KI/KD 宏）
 * @retval None
 */
void YawPID_Init(void)
{
    yawPID.kp = YAW_PID_KP;
    yawPID.ki = YAW_PID_KI;
    yawPID.kd = YAW_PID_KD;
    yawPID.target = 0.0f;
    yawPID.current = 0.0f;
    yawPID.error = 0.0f;
    yawPID.prevError = 0.0f;
    yawPID.integral = 0.0f;
    yawPID.output = 0.0f;
    YawOffset_Init();   /* 零偏/偏差状态清零已移至 HWT101_Init() 起始处,此处不重复 */
}

/**
 * @brief 初始化行进偏差
 * 
 * @note 必须在行进过程中调用(不包含转弯)，否则会记录错误的偏差
 */
void YawOffset_Init(void)
{
    yawOffset.MovingOffset = 0.0f;
    yawOffset.MovingOffsetStart = 0.0f;
    yawOffset.MovingOffsetEnd = 0.0f;
    yawOffset.RotatingOffset = 0.0f;
    yawOffset.Offset = 0.0f;
}

/**
 * @brief 计算角度PID输出
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
static void yawPID_Calculate(void)
{
    yawPID.error = yawPID.target - yawPID.current;
    yawPID.error = (yawPID.error >= 180.0f) ? yawPID.error - 360.0f : yawPID.error;
    yawPID.error = (yawPID.error <= -180.0f) ? yawPID.error + 360.0f : yawPID.error;
    // 误差死区：|误差|<0.2°视为已到达，避免到位后小幅度震荡
    if(yawPID.error < 0.2f && yawPID.error > -0.2f)
    {
        AGV_StopNow();
        /* 残差做±180归一:补偿后读数可能超出[-180,180),不归一会出现±360跳变 */
        yawOffset.RotatingOffset = ZeroBias_GetYaw() - yawPID.target;
        while(yawOffset.RotatingOffset > 180.0f)  yawOffset.RotatingOffset -= 360.0f;
        while(yawOffset.RotatingOffset <= -180.0f) yawOffset.RotatingOffset += 360.0f;
        yawPID.error = 0.0f;
        yawPID.integral = 0.0f;
        TurnCpltFlag = 1;
    }
    float P_term, I_term, D_term;
    // 比例项
    P_term = yawPID.kp * yawPID.error;
    // 积分项累加并限幅（抗积分饱和，限的是积分本身）
    if(yawPID.error <= 30.0f && yawPID.error >= -30.0f)
    {
        yawPID.integral += yawPID.error;
    }
    if(yawPID.integral > YAW_PID_INT_LIM)
    {
        yawPID.integral = YAW_PID_INT_LIM;
    }
    else if(yawPID.integral < -YAW_PID_INT_LIM)
    {
        yawPID.integral = -YAW_PID_INT_LIM;
    }
    I_term = yawPID.ki * yawPID.integral;
    // 微分项(对测量值微分:prevError 里存的是上一拍 current,prevError-current=-Δcurrent;
    // 常目标下与标准误差微分等价,只起阻尼;差值做±180归一防±360尖峰)
    {
        float d_delta = yawPID.prevError - yawPID.current;
        d_delta = (d_delta >= 180.0f) ? d_delta - 360.0f : d_delta;
        d_delta = (d_delta <= -180.0f) ? d_delta + 360.0f : d_delta;
        D_term = yawPID.kd * d_delta;
    }
    yawPID.prevError = yawPID.current;
    // 总输出
    yawPID.output = P_term + I_term + D_term;
    // 输出限幅
    if(yawPID.output > 200.0f)
    {
        yawPID.output = 200.0f;
    }
    else if(yawPID.output < -200.0f)
    {
        yawPID.output = -200.0f;
    }
}

/**
 * @brief 设置目标角度并执行一拍角度闭环（无新IMU数据时跳过本拍）
 * @param yaw 目标角度(单位:度,范围:-180~180)
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
void AGV_SetYaw(float yaw)
{
    while(TurnCpltFlag == 0)
    {
        yawPID.target = yaw;
        if(!HWT101_IsDataReady())
        {
            continue; /* 无新数据原地等下帧:不能提前退出,否则本次转弯会被跳过 */
        }
        HWT101_ClearDataReady();
        yawPID.current = ZeroBias_GetYaw();
        while(yawPID.current > 180.0f)
        {
            yawPID.current -= 360.0f;
        }
        while(yawPID.current <= -180.0f)
        {
            yawPID.current += 360.0f;
        }
        yawPID_Calculate();
        if(TurnCpltFlag)break;   /* 完成拍已StopNow,不再下发速度,防止停车后车轮继续转动 */
        omega = yawPID.output;
        AGV_SetOmega(omega);
    }
    TurnCpltFlag = 0;
}

/**
 * @brief 左转
 * 
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
void AGV_TurnLeft(void)
{
    yawPID.current = ZeroBias_GetYaw();
    yawOffset.Offset = yawOffset.MovingOffset + yawOffset.RotatingOffset;
    target = yawPID.current + 90.0f - yawOffset.Offset;
    if(target > 180.0f)target -= 360.0f;
    AGV_SetYaw(target);
    yawOffset.MovingOffset = 0.0f;   /* 已消费,防止连续转弯时重复套用上次直行偏差 */
    AGV_Direction = (AGV_Direction+1)%4;
}

/**
 * @brief 右转
 * 
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
void AGV_TurnRight(void)
{
    yawPID.current = ZeroBias_GetYaw();
    yawOffset.Offset = yawOffset.MovingOffset + yawOffset.RotatingOffset;
    target = yawPID.current - 90.0f - yawOffset.Offset;
    if(target < -180.0f)target += 360.0f;
    AGV_SetYaw(target);
    yawOffset.MovingOffset = 0.0f;   /* 已消费,防止连续转弯时重复套用上次直行偏差 */
    /* 先 +3 再取模：AGV_Direction 是 uint8_t，直接写 (x-1)%4 在 x==0 时
       会算出 -1，赋回 uint8_t 变成 255，之后 AGV_Position_Go 里
       4 个朝向分支全部匹配失败（连续右转 4 次必现） */
    AGV_Direction = (AGV_Direction+3)%4;
}

/*
 * @brief 开始记录行进偏差
 * @note 必须在行进过程中调用(不包含转弯),否则会记录错误的偏差
*/
void Yaw_MovingOffsetStart(void)
{
    yawOffset.MovingOffsetStart = ZeroBias_GetYaw();
}

/*
 * @brief 结束记录行进偏差
 * @note 段内漂移已由运动期连续补偿(b̂·dt)扣除,此处直接取补偿后读数差,
 *       差值 = 真实物理跑偏v + 估计残差(b−b̂)·T_s,不得再减b̂·T_s(会重复计数)
*/
void Yaw_MovingOffsetEnd(void)
{
    yawOffset.MovingOffsetEnd = ZeroBias_GetYaw();
    yawOffset.MovingOffset += yawOffset.MovingOffsetEnd - yawOffset.MovingOffsetStart;
}

/**
 * @brief 设置目标角速度（车体°/s -> 轮子RPM，四轮同向原地旋转）
 * 
 * @param omega 目标角速度(单位:度/秒,范围:-500~500)
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
void AGV_SetOmega(float omega)
{
    /* 正负只决定转向dir，转速统一取绝对值；omega==0时dir=1但vel=0，等价停车指令 */
    uint8_t dir = (omega >= 0.0f) ? 1U : 0U;
    float mag  = (omega < 0.0f) ? -omega : omega;
    uint16_t vel = (uint16_t)(51.0f * mag / 50.0f);

    Emm_V5_Vel_Control(1, dir, vel, 0, 1);
    delay_us(250);
    Emm_V5_Vel_Control(2, dir, vel, 0, 1);
    delay_us(250);
    Emm_V5_Vel_Control(3, dir, vel, 0, 1);
    delay_us(250);
    Emm_V5_Vel_Control(4, dir, vel, 0, 1);
    delay_us(250);

    Emm_V5_Synchronous_motion(0);    // 广播地址0触发
    delay_us(250);

    while(can.rxFrameFlag == false);
    can.rxFrameFlag = false;
}

/**
 * @brief 停车!
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
void AGV_StopNow(void)
{
    Emm_V5_Stop_Now(1, 1);
	delay_us(250);
	Emm_V5_Stop_Now(2, 1);
	delay_us(250);
	Emm_V5_Stop_Now(3, 1);
	delay_us(250);
	Emm_V5_Stop_Now(4, 1);
	delay_us(250);
	
	Emm_V5_Synchronous_motion(0); 								 // 广播地址0触发
	delay_us(250);	
	
	while(can.rxFrameFlag == false);
	can.rxFrameFlag = false;
}
   
/**
 * @brief 将航向角格式化为字符串（nano.specs下sprintf不支持%f，改用整数拆分）
 * @param yaw 航向角（°，-180~180）
 * @param buf 输出缓冲区（至少8字节）
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void FormatYaw(float yaw, char *buf)
{
    int sign = (yaw < 0.0f) ? -1 : 1;
    int v = (int)(yaw * 100.0f * (float)sign + 0.5f);   /* 绝对值放大100倍并四舍五入 */
    if(sign < 0)
    {
        sprintf(buf, "-%d.%02d", v / 100, v % 100);
    }
    else
    {
        sprintf(buf, "%d.%02d", v / 100, v % 100);
    }
}

/**
 * @brief 主循环周期调用：在OLED固定行(80,48)刷新当前航向角(调试显示)
 * @note  显示原始 HWT101_GetYaw();若要看控制所用的补偿后航向，
 *        把下行参数换成 ZeroBias_GetYaw()
 * @retval None
 */
void Yaw_OLED_Show(void)
{
    char buf[16];
    FormatYaw(HWT101_GetYaw(), buf);
    OLED_ShowString(80, 48, (uint8_t *)buf, 16, 0);
    OLED_Refresh();
}
