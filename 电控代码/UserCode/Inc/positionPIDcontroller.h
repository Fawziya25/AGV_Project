#ifndef __POSITIONPIDCONTROLLER_H
#define __POSITIONPIDCONTROLLER_H

#include "main.h"
#include "Emm_V5.h"
#include "delay.h"
#include <stdint.h>
#include "yawPIDcontroller.h"

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
/* 位置修正单拍：输入【已补偿】偏差 dx/dy(px)，算一轮PID并下发一轮轮速。
   dx→小车Y(平移)、dy→小车X(直行)，方向已按实测取反。
   ⚠ 输入应为 AGV_Correct_ApplyMountCorr() 补偿后的值；拿相机原始 dx/dy
   直接调会差一个相机安装偏置（相机中心后移20mm↔45px）。 */
void AGV_Position_Correct(int16_t dx, int16_t dy);
/* 视觉闭环修正主流程：阻塞持续修正直到"真对准"或超时，退出前自动停车。
   内部循环读 OpenMV 数据（依赖 camera.h），按 AGV_CTRL_TICK_MS 节拍调 AGV_Position_Correct。
   timeout_ms = 本段修正允许的总时长，各阶段可自行传不同值；用 AGV_CORRECT_TMO_MS 作默认。
   本入口用**放置侧**那套相机安装偏置（Place_Correct_* 等默认用它）。 */
#define AGV_CORRECT_TMO_MS  3000U   /* 修正超时默认值(ms)，调用点可按阶段改写 */
void AGV_Correct(uint32_t timeout_ms);

/* 相机安装偏置的两套选择（值在 positionPIDcontroller.c 顶部）：
     MOUNT_CORR_PLACE = 放置侧 CAM_MOUNT_CORR_X/Y（默认）
     MOUNT_CORR_PICK  = 取料侧 PICK_CAM_MOUNT_CORR_X/Y，**只有 Pick_Correct 用**
   取料时相机对着原料区、放置时对着圆环区，光轴落点不同，两套互不影响。 */
typedef enum {
    MOUNT_CORR_PLACE = 0,
    MOUNT_CORR_PICK  = 1,
} MountCorrSet;

/* 视觉闭环修正（指定偏置套）：AGV_Correct 的完整版。
   取料侧这样调：AGV_CorrectWithMountCorr(3000, MOUNT_CORR_PICK); */
void AGV_CorrectWithMountCorr(uint32_t timeout_ms, MountCorrSet set);

/* 相机安装偏差补偿：把相机原始 dx/dy(相对画面中心)原地换成相对夹爪落点的偏差。
   补偿量在 positionPIDcontroller.c 顶部（放置侧 / 取料侧两套）。
   AGV_Correct*() 内部已用；调试屏显示同用它，保证"屏上 x/y≈0 = 闭环真对准"。
   ⚠ 与视觉侧 DXDY_OFFSET 互斥，只能开一边（两边都开=净偏90px/40mm）。 */
/* 用"当前闭环那套"（由 AGV_CorrectWithMountCorr 设置，没进过闭环时为放置侧） */
void AGV_Correct_ApplyMountCorr(int16_t *pdx, int16_t *pdy);
/* 指定用哪一套：对准判定与 PID 必须和闭环本身用同一套 */
void AGV_Correct_ApplyMountCorrEx(int16_t *pdx, int16_t *pdy, MountCorrSet set);
/* 立即停车（对准/超时退出时调用） */
void AGV_Position_Stop(void);

#endif
