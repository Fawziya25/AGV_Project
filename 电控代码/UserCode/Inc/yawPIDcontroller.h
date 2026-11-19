#ifndef __YAW_PID_CONTROLLER_H__
#define __YAW_PID_CONTROLLER_H__   

#include "main.h"
#include "Emm_V5.h"
#include "delay.h"
#include "hwt101.h"
#include "oled.h"
#include "stdio.h"

#define YAW_PID_INT_LIM 80.0f   /* 角度PID积分限幅(°) */

/* ---------------- 转向最大轮速 ----------------
   轮速(RPM) = |yawPID.output| × YAW_OMEGA_PER_OUT × YAW_RPM_PER_OMEGA
             = |output| × 0.94 × 1.02 = |output| × 0.9588
   所以 PID 输出限幅由目标轮速反算，改轮速只改 YAW_WHEEL_MAX_RPM 一处。
   ⚠ 这两个系数分别写在 AGV_SetYaw() 的 omega 换算和 AGV_SetOmega() 的
     omega→RPM 换算里，本头文件把它们抽成宏，改系数时三处不会对不上。 */
#define YAW_WHEEL_MAX_RPM   300.0f              /* 转向时的最大轮速(RPM) */
#define YAW_OMEGA_PER_OUT   0.94f               /* PID输出 → 车体角速度(°/s) */
#define YAW_RPM_PER_OMEGA   (51.0f / 50.0f)     /* 车体角速度(°/s) → 轮速(RPM) */
#define YAW_PID_OUT_LIM     (YAW_WHEEL_MAX_RPM / (YAW_OMEGA_PER_OUT * YAW_RPM_PER_OMEGA))

/* 航向PID增益:调参直接改这里。AGV_SetYaw() 每次转弯前自己重装，
   不再要求上电时调 YawPID_Init()（那个现在只多清一次 yawOffset） */
#define YAW_PID_KP  1.2f
#define YAW_PID_KI  0.035f
#define YAW_PID_KD  0.045f

/* ============ 航向：记录 + 修正（相对导航，2026-11-19 简化） ============
 * 一句话：**转弯目标从"当前读数"算起，把上次的残差和这一段的行进偏差扣掉**。
 *   target = 当前读数 + 转向×90/180 − 上次停位残差 − 本段行进偏差
 *
 *   - yawLastResid 上次导航转弯停稳后的残差(°)= 停稳读数 − 目标，下次转弯补回。
 *   - movingDelta  自上次转弯以来，直行/平移期间读数的变化累计(°)，下次转弯反补。
 *   两项都是记录值，没有学习/门限/滤波。
 *   目标从当前读数算起 ⇒ 读数里的恒定零偏在相减中抵消，零漂不会变成真实转动。
 *   ⚠ 代价：没有绝对基准，摆车时相对场地歪的那几度修不回来（实车摆车是准的）。
 * 软件零偏估计器(ZeroBias_*)整套删除：零偏交给 HWT101 模块固件自校准
 * （HWT101_Init → ManualCal 写 0xA6，协议对 AGV 的推荐用法），读数一律
 * HWT101_GetYaw()。 */
#define YAW_SETTLE_EPS    0.05f    /* 停稳判据：相邻两拍读数差(°) */
#define YAW_SETTLE_TICKS  15U      /* 停稳等待上限(拍, 10ms/拍) */

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

/* 小车朝向: 0:X+ 1:Y+ 2:X- 3:Y-，由 AGV_TurnLeft/Right 每次转弯后更新 */
extern uint8_t AGV_Direction;

/* yawPID / 残差记录 / 行进偏差 均为 yawPIDcontroller.c 内部静态量,经下面的 getter 读 */

// 初始化角度PID控制器(增益取 YAW_PID_KP/KI/KD 宏,无需传参)，仅测试模式上电时调
void YawPID_Init(void);
// 设置小车目标航向角(阻塞到进死区停车；导航转弯请用下面的 AGV_TurnXxx)
// **内部不改任何记录**：视觉对正/恢复原航向这类调用不算"转向"
void AGV_SetYaw(float yaw);
// 相对转角微调(闭环、带最低角速度、带超时)：半度级前馈补偿用。
// delta_deg 正负号与 AGV_SetYaw 的目标角同向；**有意**改航向，不写任何记录；
// 返回时车已停住，不会卡死。
void AGV_TrimYaw(float delta_deg);
// 左转/右转/掉头：目标 = 当前读数 ± 90/180 − 上次停位残差 − 本段行进偏差
void AGV_TurnLeft(void);
void AGV_TurnRight(void);
void AGV_TurnBack(void);
void AGV_Turn(uint8_t dir);
// 清空已记录的行进偏差：只在航向被**绝对对正**之后调(例：视觉 0x06 闭环)。
// 平时不用调，导航转弯自己会清零。
void Yaw_ClearMovingDelta(void);

// 行进偏差记录对：AGV_Position_GoStraight/Translate 内部已成对调用，别处不用调
void Yaw_MovingOffsetStart(void);
void Yaw_MovingOffsetEnd(void);

// ---- 观测口（debugger watch / OLED / 上位机） ----
float Yaw_GetLastResid(void);     /* 上次导航转弯停稳后的残差(°)：正=转多了 */
float Yaw_GetMovingDelta(void);   /* 自上次转弯以来记录的行进偏差(°) */

void FormatYaw(float yaw, char *buf);
// 主循环周期调用:OLED固定行显示当前航向(调试)
void Yaw_OLED_Show(void);

#endif