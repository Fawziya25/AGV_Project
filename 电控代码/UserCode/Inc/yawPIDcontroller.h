#ifndef __YAW_PID_CONTROLLER_H__
#define __YAW_PID_CONTROLLER_H__   

#include "main.h"
#include "Emm_V5.h"
#include "delay.h"
#include "hwt101.h"
#include "oled.h"
#include "stdio.h"

#define YAW_PID_INT_LIM 80.0f   /* 角度PID积分限幅(°) */

/* 航向PID增益:调参直接改这里,调用侧只需 YawPID_Init() 无参初始化 */
#define YAW_PID_KP  1.2f
#define YAW_PID_KI  0.035f
#define YAW_PID_KD  0.045f

#define ZB_RATE_GATE    0.3f    /* 角速度门控(°/s):超过视为真实运动 */
#define ZB_MIN_SAMPLES  20U     /* 窗口最少有效样本数(10Hz≈2s,50Hz≈0.4s) */

typedef struct {
	float kp;
    float ki;
    float kd;
    float target;
    float current;
    float error;
    float prevError;
    float integral;
    float output;
}YawPID_TypeDef;

typedef struct{
    float MovingOffset;
    float MovingOffsetStart;
    float MovingOffsetEnd;
    float RotatingOffset;
    float Offset;
}YawOffset_TypeDef;

typedef struct
{
    volatile uint8_t active;    /* 窗口进行中 */
    uint8_t          dirty;     /* 窗口被污染,结束时丢弃 */
    uint32_t         n;         /* 有效样本数 */
    float            sumWz;     /* 角速度累加(均值 = 零偏) */
    float            startYaw;  /* 窗口起始读数(°) */
    float            yawDriftComp; /* 累计漂移补偿量,读数减去它 */
    float            b_hat;     /* 最新零偏估计(°/s),跨窗口EMA平滑 */
} ZeroBias_TypeDef;

extern uint8_t AGV_Direction;

/* yawPID / yawOffset / TurnCpltFlag 均为 yawPIDcontroller.c 内部静态量,不对外导出 */

// 初始化角度PID控制器(增益取 YAW_PID_KP/KI/KD 宏,无需传参)
void YawPID_Init(void);
// 设置小车目标航向角
void AGV_SetYaw(float yaw);
// 左转
void AGV_TurnLeft(void);
// 右转
void AGV_TurnRight(void);

void FormatYaw(float yaw, char *buf);
// 主循环周期调用:OLED固定行显示当前航向(调试)
void Yaw_OLED_Show(void);

// 带门控的陀螺零偏在线估计器(实现在 yawPIDcontroller.c)
void  ZeroBias_Reset(void);   /* 上电/发车前清零全部状态 */
void  ZeroBias_Start(void);   /* 开始静止窗口(车停稳后调用) */
void  ZeroBias_Feed(void);    /* 每收到新IMU数据时调用 */
void  ZeroBias_End(void);     /* 结束窗口(开始移动前调用) */
float ZeroBias_GetYaw(void);  /* 漂移补偿后的航向角(°),替代HWT101_GetYaw() */
float ZeroBias_GetBias(void); /* 当前零偏估计(°/s),供MovingOffset修正 */
void Yaw_MovingOffsetStart(void);
void Yaw_MovingOffsetEnd(void);

#endif