/**
 * @file pick_task.h
 * @brief 物料取放任务阶段：视觉闭环修正 + 各阶段取放动作
 * @note  颜色数组定义在 QRcode.c；RerunFlag 定义在 task.c
 */

#ifndef __PICK_TASK_H
#define __PICK_TASK_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "pick_ctrl.h"
#include "openmv.h"

/* ========================== 辅助宏定义 ========================== */

/* 发送颜色命令已由 openmv.c 的 OpenMV_SwitchTo() 承担(带回显确认),不再用宏 */

/** 移动到普通圆环位置 */
#define MOVE_TO_RING(ring) \
    do { \
        if ((ring) == 1) Pick_MoveTo POS_1; \
        else if ((ring) == 2) Pick_MoveTo POS_2; \
        else if ((ring) == 3) Pick_MoveTo POS_3; \
    } while(0)

/** 移动到高位圆环位置 */
#define MOVE_TO_RING_HIGH(ring) \
    do { \
        if ((ring) == 1) Pick_MoveTo POS_1_HIGE; \
        else if ((ring) == 2) Pick_MoveTo POS_2_HIGE; \
        else if ((ring) == 3) Pick_MoveTo POS_3_HIGE; \
    } while(0)

/** 从指定位置抓取并放置到圆环 */
#define GRAB_AND_PLACE_TO_RING(grab_pos, ring) \
    do { \
        Pick_MoveTo grab_pos; \
        Pick_Gripper_Grab(); \
        MOVE_TO_RING(ring); \
        Pick_Gripper_Release(); \
    } while(0)

/** 从指定位置抓取并放置到圆环（支持高位） */
#define GRAB_AND_PLACE_TO_RING_EX(grab_pos, ring, is_high) \
    do { \
        Pick_MoveTo grab_pos; \
        Pick_Gripper_Grab(); \
        if (is_high) { MOVE_TO_RING_HIGH(ring); } \
        else { MOVE_TO_RING(ring); } \
        Pick_Gripper_Release(); \
    } while(0)

/** 从圆环抓取并放置到指定位置 */
#define GRAB_FROM_RING_AND_PLACE(ring, place_pos) \
    do { \
        MOVE_TO_RING(ring); \
        Pick_Gripper_Grab(); \
        Pick_MoveTo place_pos; \
        Pick_Gripper_Release(); \
    } while(0)

/* ========================== 函数声明 ========================== */

void Pick_RAW(void);
void Pick_RAW_PLUS(void);
void Place_ROUGH(void);
void Pick_ROUGH(void);
void Place_TEMP(void);

#endif /* __PICK_TASK_H */