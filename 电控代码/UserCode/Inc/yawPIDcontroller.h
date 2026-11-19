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

/* ============ 航向基准与偏差学习（2026-11-18 重构） ============
 * 新框架一句话：**转弯直接瞄准名义航向**，target = Nominal ± 90 + 转向×turnLag。
 *   - 名义航向 Nominal 是唯一基准：转弯 ±90/180 更新；视觉对正后 Yaw_NominalSync()。
 *     上电后车体再被偏转(挪车/碰撞/打滑/读数漂移)全部体现在「当前读数−Nominal」
 *     里，转弯瞄准 Nominal 时一次性修掉 → 数学上不跨段累积（仿真：上电后偏 5°，
 *     12 次转弯后旧框架误差 5~12°，新框架 0.06°）。摆放相对场地的歪不在此列。
 * 残差在**车停稳之后**采样：控制环把读数拧到目标，停稳残差 = 控制环出口的
 * 读数差(死区相位+滑行)，累计读数漂移在这个差值里被消掉 → turnLag 不吃漂移。
 *   - movingDelta 只作行进偏差观测量（诊断/上位机），不进控制——它代表的偏差
 *     已被「瞄准名义角」自动修掉。
 * 软件零偏估计器(ZeroBias_*)整套删除：零偏交给 HWT101 模块固件自校准
 * （HWT101_Init → ManualCal 写 0xA6，协议对 AGV 的推荐用法），读数一律
 * HWT101_GetYaw()。
 * ⚠ 残余零偏的真实代价在**真实航向**：车实际朝向 = 名义角 − 累计读数漂移，
 *   每段带一个当段漂移量(不跨段叠加)。防它的是模块零偏管理：上电 6s ManualCal
 *   + 中途每个工位静止 20~60s 触发模块自动校零(NOAUTOCALI 出厂默认开)。 */
#define YAW_LAG_GAIN      1.0f    /* 残差学习增益：1.0 = 直接用上次残差当预测 */
#define YAW_LAG_LIMIT     0.8f    /* 残差幅值上限(°)：超出判异常(打滑/被撞)，不学 */
#define YAW_LAG_MIN_TURN  30.0f   /* 学残差的最小转角(°)：视觉小修正噪声占比大，不学 */
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

/* yawPID / 名义角 / turnLag 等均为 yawPIDcontroller.c 内部静态量,经下面的 getter 读 */

// 初始化角度PID控制器(增益取 YAW_PID_KP/KI/KD 宏,无需传参)；
// 会把当前实际航向设为名义基准，仅测试模式上电时调
void YawPID_Init(void);
// 把当前实际航向设为新的名义基准：视觉对正/人工挪车后必须调，
// 否则下一次转弯会把车拧回旧的名义角，对正成果被抹掉
void Yaw_NominalSync(void);
// 设置小车目标航向角(阻塞到进死区停车；导航转弯请用下面的 AGV_TurnXxx)
void AGV_SetYaw(float yaw);
// 相对转角微调(闭环、带最低角速度、带超时)：半度级前馈补偿用。
// delta_deg 正负号与 AGV_SetYaw 的目标角同向；不学 turnLag、不动名义角
// (要并入基准由调用方自己 Yaw_NominalSync())；返回时车已停住，不会卡死。
void AGV_TrimYaw(float delta_deg);
// 左转/右转/掉头：目标 = 名义角 ± 90/180 + 转向×停位残差学习
void AGV_TurnLeft(void);
void AGV_TurnRight(void);
void AGV_TurnBack(void);
void AGV_Turn(uint8_t dir);

// 行进偏差观测对：AGV_Position_GoStraight/Translate 内部已成对调用，别处不用调
void Yaw_MovingOffsetStart(void);
void Yaw_MovingOffsetEnd(void);

// ---- 观测口（debugger watch / OLED / 上位机） ----
float Yaw_GetNominal(void);       /* 名义航向(°) */
float Yaw_GetTurnLag(void);       /* 停位残差学习值(°, 停早为正) */
float Yaw_GetLastResid(void);     /* 上次转弯停稳后的带符号残差(°) */
float Yaw_GetMovingDelta(void);   /* 自上次转弯以来的行进偏差累计(°) */

void FormatYaw(float yaw, char *buf);
// 主循环周期调用:OLED固定行显示当前航向(调试)
void Yaw_OLED_Show(void);

#endif