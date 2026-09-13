#include "yawPIDcontroller.h"
#include "hwt101.h"

YawPID_TypeDef yawPID;
// 小车整体角速度
float omega;
char TextTest[16];

uint8_t TurnCpltFlag  = 0;

static void FormatYaw(float yaw, char *buf);

/**
 * @brief 初始化角度PID控制器
 * 
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void YawPID_Init(float kp, float ki, float kd)
{
    yawPID.kp = kp;
    yawPID.ki = ki;
    yawPID.kd = kd;
    yawPID.target = 0.0f;
    yawPID.current = 0.0f;
    yawPID.error = 0.0f;
    yawPID.prevError = 0.0f;
    yawPID.integral = 0.0f;
    yawPID.output = 0.0f;
}

/**
 * @brief 计算角度PID输出
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
void yawPID_Calculate(void)
{
    yawPID.error = yawPID.target - yawPID.current;
    yawPID.error = (yawPID.error >= 180.0f) ? yawPID.error - 360.0f : yawPID.error;
    yawPID.error = (yawPID.error <= -180.0f) ? yawPID.error + 360.0f : yawPID.error;
    // 误差死区：|误差|<0.4°视为已到达，避免到位后小幅度震荡
    if(yawPID.error < 0.4f && yawPID.error > -0.4f)
    {
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
    if(yawPID.integral > IntegralLimit)
    {
        yawPID.integral = IntegralLimit;
    }
    else if(yawPID.integral < -IntegralLimit)
    {
        yawPID.integral = -IntegralLimit;
    }
    I_term = yawPID.ki * yawPID.integral;
    // 微分项（误差差分，差值做±180归一，避免跨越±180°边界时出现±360尖峰）
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
    yawPID.target = yaw;
    if(!HWT101_IsDataReady())
    {
        return; /* 没有新IMU数据，跳过本拍，不重复下发 */
    }
    HWT101_ClearDataReady();
    yawPID.current = HWT101_GetYaw();
    yawPID_Calculate();
    omega = yawPID.output;
    // FormatYaw(yawPID.current, TextTest);
    // OLED_ShowString(0, 0, (uint8_t *)TextTest, 16, 0);
    // OLED_Refresh();
    AGV_SetOmega(omega);
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
    uint16_t vel = 0;
    if(omega >= 0.0f)
    {
        vel = (uint16_t)(51.0f * omega / 50.0f);
		Emm_V5_Vel_Control(1, 1, vel, 0, 1);
		delay_us(250);
		Emm_V5_Vel_Control(2, 1, vel, 0, 1);
		delay_us(250);
		Emm_V5_Vel_Control(3, 1, vel, 0, 1);
		delay_us(250);
		Emm_V5_Vel_Control(4, 1, vel, 0, 1);
		delay_us(250);
        
        Emm_V5_Synchronous_motion(0); 								 // 广播地址0触发
        delay_us(250);		
        
        while(can.rxFrameFlag == false);
        can.rxFrameFlag = false;
    }
    else if(omega < 0.0f)
    {
        vel = (uint16_t)(-51.0f * omega / 50.0f);
		Emm_V5_Vel_Control(1, 0, vel, 0, 1);
		delay_us(250);
		Emm_V5_Vel_Control(2, 0, vel, 0, 1);
		delay_us(250);
		Emm_V5_Vel_Control(3, 0, vel, 0, 1);
		delay_us(250);
		Emm_V5_Vel_Control(4, 0, vel, 0, 1);
		delay_us(250);
        
        Emm_V5_Synchronous_motion(0); 								 // 广播地址0触发
        delay_us(250);		
        
        while(can.rxFrameFlag == false);
        can.rxFrameFlag = false;
    }
}
   
/**
 * @brief 将航向角格式化为字符串（nano.specs下sprintf不支持%f，改用整数拆分）
 * @param yaw 航向角（°，-180~180）
 * @param buf 输出缓冲区（至少8字节）
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
static void FormatYaw(float yaw, char *buf)
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