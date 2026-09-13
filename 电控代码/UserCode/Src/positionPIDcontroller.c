#include "positionPIDcontroller.h"
#include "Emm_V5.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

PositionPID_TypeDef positionXPID;
PositionPID_TypeDef positionYPID;

#define a 100U
#define b 115U

int time_flag = 0;
int pre_time_flag = 0;

float temp_VelX = 0.0f;
float temp_VelY = 0.0f;

// 本程序中长度单位为mm，速度单位为mm/s，轮子转速单位为RPM

/**
 * @brief 计算Mecanum轮子速度
 * @param velX 小车速度Vx（mm/s）
 * @param velY 小车速度Vy（mm/s）
 * @retval float 轮子转速（RPM，带符号）
 * @author Fawziya
 * @date 2026-08-18
 */
float MecanumIK_getw13(float velX, float velY)
{
    // 60/2piR = 0.2387
    return 0.2387f*(velX+velY);
}

/**
 * @brief 计算Mecanum轮子速度
 * @param velX 小车速度Vx（mm/s）
 * @param velY 小车速度Vy（mm/s）
 * @retval float 轮子转速（RPM，带符号）
 * @author Fawziya
 * @date 2026-08-18
 */
float MecanumIK_getw24(float velX, float velY)
{
    // 60/2piR = 0.2387
    return 0.2387f*(velY-velX);
}

/**
 * @brief 初始化位置PID控制器
 * @param positionPID 位置PID控制器指针
 * @param kp P项系数
 * @param ki I项系数
 * @param kd D项系数
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void PositionPID_Init(PositionPID_TypeDef *positionPID, float kp, float ki, float kd)
{
    positionPID->kp = kp;
    positionPID->ki = ki;
    positionPID->kd = kd;
    positionPID->target = 0.0f;
    positionPID->current = 0.0f;
    positionPID->error = 0.0f;
    positionPID->prevError = 0.0f;
    positionPID->integral = 0.0f;
    positionPID->output = 0.0f;
}

/**
 * @brief 更新位置
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void PositionUpdate()
{
    // Emm_V5_Read_Sys_Params(1, S_VEL);
    // while(can.rxFrameFlag == false){}; 
    // can.rxFrameFlag = false;
    // uint16_t vel1, vel2, vel3, vel4;
    // int Motor_Vel1, Motor_Vel2, Motor_Vel3, Motor_Vel4;
    // if(can.rxData[0] == 0x35 && can.CAN_RxMsg.DLC == 5)
    // {
    //     // 拼接成uint16_t类型数据
    //     vel1 = (uint16_t)(((uint16_t)can.rxData[2] << 8)	| ((uint16_t)can.rxData[3] << 0));
    //     // 实时转速
    //     Motor_Vel1 = vel1;
    //     // 符号
    //     if(can.rxData[1]) { Motor_Vel1 = -Motor_Vel1; }
    //     // 清除缓存
    //     can.rxData[0] = 0; can.CAN_RxMsg.DLC = 0;
    // }
    // Emm_V5_Read_Sys_Params(2, S_VEL);
    // while(can.rxFrameFlag == false){}; 
    // can.rxFrameFlag = false;
    // if(can.rxData[0] == 0x35 && can.CAN_RxMsg.DLC == 5)
    // {
    //     // 拼接成uint16_t类型数据
    //     vel2 = (uint16_t)(((uint16_t)can.rxData[2] << 8)	| ((uint16_t)can.rxData[3] << 0));
    //     // 实时转速
    //     Motor_Vel2 = vel2;
    //     // 符号
    //     if(can.rxData[1]) { Motor_Vel2 = -Motor_Vel2; }
    //     // 清除缓存
    //     can.rxData[0] = 0; can.CAN_RxMsg.DLC = 0;
    // }
    // Emm_V5_Read_Sys_Params(3, S_VEL);
    // while(can.rxFrameFlag == false){}; 
    // can.rxFrameFlag = false;
    // if(can.rxData[0] == 0x35 && can.CAN_RxMsg.DLC == 5)
    // {
    //     // 拼接成uint16_t类型数据
    //     vel3 = (uint16_t)(((uint16_t)can.rxData[2] << 8)	| ((uint16_t)can.rxData[3] << 0));
    //     // 实时转速
    //     Motor_Vel3 = vel3;
    //     // 符号
    //     if(can.rxData[1]) { Motor_Vel3 = -Motor_Vel3; }
    //     // 清除缓存
    //     can.rxData[0] = 0; can.CAN_RxMsg.DLC = 0;
    // }
    // Emm_V5_Read_Sys_Params(4, S_VEL);
    // while(can.rxFrameFlag == false){}; 
    // can.rxFrameFlag = false;
    // if(can.rxData[0] == 0x35 && can.CAN_RxMsg.DLC == 5)
    // {
    //     // 拼接成uint16_t类型数据
    //     vel4 = (uint16_t)(((uint16_t)can.rxData[2] << 8)	| ((uint16_t)can.rxData[3] << 0));
    //     // 实时转速
    //     Motor_Vel4 = vel4;
    //     // 符号
    //     if(can.rxData[1]) { Motor_Vel4 = -Motor_Vel4; }
    //     // 清除缓存
    //     can.rxData[0] = 0; can.CAN_RxMsg.DLC = 0;
    // }
    time_flag = HAL_GetTick();
    // 首次调用同步基准时间，避免dt等于开机总时长导致位置跳变
    if(pre_time_flag == 0)
    {
        pre_time_flag = time_flag;
    }
    // piR/120 = 1.0472
    positionXPID.current += temp_VelX*(time_flag-pre_time_flag)*0.001f;
    positionYPID.current += temp_VelY*(time_flag-pre_time_flag)*0.001f;
    pre_time_flag = time_flag;
}

/**
 * @brief 更新位置PID控制器
 * @retval None
 */
void PositionPID_Calculate(PositionPID_TypeDef *positionPID)
{
    positionPID->error = positionPID->target - positionPID->current;

    float P_out, I_out, D_out;
    // 计算P项
    P_out = positionPID->kp * positionPID->error;
    // 计算I项
    positionPID->integral += positionPID->error;
    // 积分项限幅
    if(positionPID->integral > POSITIONPID_MAX_INTEGRAL)
    {
        positionPID->integral = POSITIONPID_MAX_INTEGRAL;
    }
    else if(positionPID->integral < -POSITIONPID_MAX_INTEGRAL)
    {
        positionPID->integral = -POSITIONPID_MAX_INTEGRAL;
    }
    I_out = positionPID->ki * positionPID->integral;
    // 计算D项
    D_out = positionPID->kd * (positionPID->error - positionPID->prevError);
    // 输出限幅
    positionPID->output = P_out + I_out + D_out;
    if(positionPID->output > POSITIONPID_MAX_OUTPUT)
    {
        positionPID->output = POSITIONPID_MAX_OUTPUT;
    }
    else if(positionPID->output < -POSITIONPID_MAX_OUTPUT)
    {
        positionPID->output = -POSITIONPID_MAX_OUTPUT;
    }
    positionPID->prevError = positionPID->error;
}

/**
 * @brief 获取位置PID控制器输出
 * @param positionPID 位置PID控制器指针
 * @return float 位置PID控制器输出
 */
float PositionPID_GetOutput(PositionPID_TypeDef *positionPID)
{
    return positionPID->output;
}

/**
 * @brief 设置AGV位置控制目标（不执行控制，闭环在下一节拍由AGV_Position_Loop完成）
 * @param targetX 目标X坐标（mm）
 * @param targetY 目标Y坐标（mm）
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
void AGV_Position_Set(float targetX, float targetY)
{
    positionXPID.target = targetX;
    positionYPID.target = targetY;
}

/**
 * @brief 位置闭环单拍执行：读反馈更新位置->计算X/Y PID->麦轮逆解->限幅下发速度
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
void AGV_Position_Loop(void)
{
    PositionUpdate();
    PositionPID_Calculate(&positionXPID);
    PositionPID_Calculate(&positionYPID);

    float velX = PositionPID_GetOutput(&positionXPID)/100.0f;
    float velY = PositionPID_GetOutput(&positionYPID)/100.0f;

    temp_VelX = velX;
    temp_VelY = velY;

    float w13 = MecanumIK_getw13(velX, velY);
    float w24 = MecanumIK_getw24(velX, velY);

    // 轮速限幅（驱动器上限5000RPM）
    if(w13 > 5000.0f) { w13 = 5000.0f; }
    else if(w13 < -5000.0f) { w13 = -5000.0f; }
    if(w24 > 5000.0f) { w24 = 5000.0f; }
    else if(w24 < -5000.0f) { w24 = -5000.0f; }

    // 提取方向与幅值（vel形参为uint16_t，必须取绝对值）
    uint8_t sign13 = (w13 >= 0.0f) ? 0U : 1U;
    uint8_t sign24 = (w24 >= 0.0f) ? 0U : 1U;
    uint16_t vel13 = (uint16_t)((w13 >= 0.0f) ? w13 : -w13);
    uint16_t vel24 = (uint16_t)((w24 >= 0.0f) ? w24 : -w24);

    Emm_V5_Vel_Control(1, Wheel1PositiveDir ^ sign13, vel13, 0, true);
    delay_us(300);
    Emm_V5_Vel_Control(3, Wheel3PositiveDir ^ sign13, vel13, 0, true);
    delay_us(300);
    Emm_V5_Vel_Control(2, Wheel2PositiveDir ^ sign24, vel24, 0, true);
    delay_us(300);
    Emm_V5_Vel_Control(4, Wheel4PositiveDir ^ sign24, vel24, 0, true);
    delay_us(300);
    Emm_V5_Synchronous_motion(0);
    delay_us(300);

    while(can.rxFrameFlag == false){};
	can.rxFrameFlag = false;
}