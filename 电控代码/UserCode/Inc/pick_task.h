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
#include "pick.h"
#include "camera.h"

/* ========================== 辅助宏定义 ========================== */

/* 视觉交互统一由 camera.h 的请求式 API 承担
   (Cam_RequestDisc / Cam_RequestRing / Cam_RequestBoxColor),不再用宏。
   请求超时统一用 camera.h 的 CAM_REQ_TMO_MS。 */

/** 移动到普通圆环位置 */
#define MOVE_TO_RING(ring) \
    do { \
        if ((ring) == 1) Pick_JumpTo POS_1; \
        else if ((ring) == 2) Pick_JumpTo POS_2; \
        else if ((ring) == 3) Pick_JumpTo POS_3; \
    } while(0)

/** 移动到高位圆环位置 */
#define MOVE_TO_RING_HIGH(ring) \
    do { \
        if ((ring) == 1) Pick_JumpTo POS_1_HIGE; \
        else if ((ring) == 2) Pick_JumpTo POS_2_HIGE; \
        else if ((ring) == 3) Pick_JumpTo POS_3_HIGE; \
    } while(0)

/* 槽位约定：车上是「同一个臂位姿 POS_CAR + 转盘转角」，不是三个臂位姿。
   slot 0/1/2 = A/B/C 槽 = 转盘 30°/150°/270°，由 Pick_TurnTable_Set(slot) 选。
   三个宏都遵守"先转盘、后摆臂"：转盘转动时臂不在车体上方。 */

/** 从车上 slot 槽抓取并放置到圆环 */
#define GRAB_AND_PLACE_TO_RING_SLOT(slot, ring) \
    do { \
        Pick_TurnTable_Set(slot); \
        Pick_JumpTo POS_CAR; \
        Pick_Gripper_Grab(); \
        MOVE_TO_RING(ring); \
        Pick_Gripper_Release(); \
    } while(0)

/** 从车上 slot 槽抓取并放置到圆环（支持高位） */
#define GRAB_AND_PLACE_TO_RING_SLOT_EX(slot, ring, is_high) \
    do { \
        Pick_TurnTable_Set(slot); \
        Pick_JumpTo POS_CAR; \
        Pick_Gripper_Grab(); \
        if (is_high) { MOVE_TO_RING_HIGH(ring); } \
        else { MOVE_TO_RING(ring); } \
        Pick_Gripper_Release(); \
    } while(0)

/** 从圆环抓取并放置到车上 slot 槽 */
#define GRAB_FROM_RING_AND_PLACE_SLOT(ring, slot) \
    do { \
        MOVE_TO_RING(ring); \
        Pick_Gripper_Grab(); \
        Pick_TurnTable_Set(slot); \
        Pick_JumpTo POS_CAR; \
        Pick_Gripper_Release(); \
    } while(0)

/* ========================== 函数声明 ========================== */

/* 槽位约定(slot)：0=A 槽, 1=B 槽, 2=C 槽，与 Place_Color[]/Pick_Color[] 下标一致
   圆环约定(ring)：1/2/3                                             */

void Pick_RAW(void);
/* 同 Pick_RAW，但每抓一块前都回观察位重问一次 0x03，按相机重新给的 flag=1 下手，
   不推算转盘走格。当前未调用，切换见 pick_task.c 里它的注释。 */
void Pick_RAW_PLUS(void);
void Place_ROUGH(void);
void Pick_ROUGH(void);
void Place_TEMP(void);

#endif /* __PICK_TASK_H */
