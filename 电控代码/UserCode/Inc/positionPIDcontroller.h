#ifndef __POSITIONPIDCONTROLLER_H
#define __POSITIONPIDCONTROLLER_H

#include "main.h"
#include "Emm_V5.h"
#include "delay.h"
#include <stdint.h>

/**
 * @brief 位置修正PID控制器（X直行 / Y平移 两轴各一个）
 * @note  结构参考角度PID(yawPIDcontroller.c)：误差(px)→P+I+D→输出=轮速RPM
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float error;      /* 当前误差(px) */
    float prevError;  /* 上一次误差(px) */
    float integral;   /* 积分累加(px) */
    float output;     /* 输出：轮速RPM */
} PosPID_TypeDef;

/* 初始化位置修正PID（两轴共用一组参数，参数见 positionPIDcontroller.c 顶部 POS_PID_*） */
void PositionPID_Init(void);
/* 位置修正单拍：输入相机偏差 dx/dy(px)，算一轮PID并下发一轮轮速。
   dx→小车Y(平移)、dy→小车X(直行)，方向已按实测取反 */
void AGV_Position_Correct(int16_t dx, int16_t dy);
/* 视觉闭环修正主流程：阻塞持续修正直到"真对准"或超时，退出前自动停车。
   内部循环读 OpenMV 数据（依赖 openmv.h），按 AGV_CTRL_TICK_MS 节拍调 AGV_Position_Correct。
   timeout_ms = 本段修正允许的总时长，各阶段可自行传不同值；用 AGV_CORRECT_TMO_MS 作默认。 */
#define AGV_CORRECT_TMO_MS  3000U   /* 修正超时默认值(ms)，调用点可按阶段改写 */
void AGV_Correct(uint32_t timeout_ms);
/* 立即停车（对准/超时退出时调用） */
void AGV_Position_Stop(void);

#endif
