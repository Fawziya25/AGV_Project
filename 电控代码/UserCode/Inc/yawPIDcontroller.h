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

/* 转弯滞后学习（2026-11-17，修"残差符号"问题）
   转弯总是"差一点就停"：误差进死区 |e|<0.2° 就发停车，轮子再滑一小段才停住。
   这个"差一点"的**幅值**是机器属性（死区 + 停位延时 + 齿隙），跟转 90° 还是 180°
   无关；但它的**方向永远与本次转弯同向**——所以补偿时必须乘本次转向，不能直接用
   上一次残差的正负号。原来的写法把带符号的 RotatingOffset 直接减掉，符号对不对
   取决于"下一次转弯和这次是不是同向"，等于靠运气。
   现在只学幅值 RotatingMag，按本次转向加到转角上，与转弯顺序无关。
   ⚠ 但别期待它带来多少收益：主机仿真 12 次转弯，同向路线 0.11°→0.11°，
     左右交替路线 0.11°→0.15°（**反而略差**）。因为交替时前后两次的滞后本来就会
     互相抵消，没有可补偿的系统量，补偿只是把上一次的噪声搬到下一次。
     真正能把误差压到 0.04° 的是"拿名义航向做绝对反馈"（见改动记录）。 */
#define YAW_ROT_LAG_GAIN   1.0f    /* 幅值学习增益：1.0 = 直接用上一次的幅值当预测
                                      （对"每次转弯都差这么多"的常值滞后最准）。
                                      调小只是平滑，会让常值滞后收敛变慢、残余变大。 */
#define YAW_ROT_LAG_MAX    0.8f    /* 幅值上限(°)：超过判为异常(打滑/被撞)，不学、清零。
                                      0.8° ≈ 死区的 4 倍，正常停位不会到 */

typedef struct{
    float MovingOffset;      /* 平移期间的读数漂移(累加，转弯时消费掉) */
    float MovingOffsetStart;
    float MovingOffsetEnd;
    float RotatingOffset;    /* 上一次转弯的残差(带符号，只用于观察/判断方向) */
    float RotatingMag;       /* 学到的转弯滞后幅值(°，≥0)，按本次转向加进目标角 */
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

/* 小车朝向: 0:X+ 1:Y+ 2:X- 3:Y-，由 AGV_TurnLeft/Right 每次转弯后更新 */
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
void AGV_TurnBack(void);
void AGV_Turn(uint8_t dir);

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