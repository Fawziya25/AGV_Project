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

/* 视觉交互统一由 camera.h 的 Cam_SwitchTo(mode) 承担 —— 一条命令一个模式号，
   没有出参，结果读全局量 dx/dy、box_color[]、cam_flag。不再用宏。
   mode 直接写命令码的八进制字面量：01 任务码 / 02 最近物块 / 03 三框颜色 /
   04 圆环 / 05 指定颜色。 */

/* 下面这些宏一律用 Pick_JumpTo（两轴绝对位置）。前提：上电前高度/半径
   两轴人工放在 0 位，Pick_Axis_Init() 会把该位置写成驱动器绝对零点。
   pick.c 里已没有相对位置版本，所有动作都依赖这个零点。 */

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

/* 原料区取料前的视觉对正：摆 POS_RAW_B → 开爪 → 0x02 拿偏差 → AGV_Correct(5000) → 收爪。
   拿不到偏差就不修正（免得用旧 dx/dy 把车带偏）。由 task.c 的 case Pick1 在
   Pick_RAW / Pick_RAW_PLUS 之前调用一次，那两个函数自己不再做这一步。 */
void Pick_Correct(void);

/* 原料区取料（当前路线）：摆 POS_RAW_B、开爪，反复发 0x02 直到相机报回来的颜色号
   等于任务码当前要抓的那一色（flag 即颜色号 1~6），才下手抓到车上对应槽。
   严格按任务码顺序 i=0/1/2，不匹配就一直等（无超时）。 */
void Pick_RAW(void);

/* 原料区三个抓取位各自的完整取料动作：抓一块 → 放到车上第 slot 槽。
   A/C 位比 B 位远，函数内部会先沿车头方向挪底盘 3cm 再抓、放完再挪回来；
   B 位底盘不动。拆成三个是因为三处以后可能各自加不同动作（现在 C 是 A 的镜像）。 */
void Pick_RAW_A(uint8_t slot);
void Pick_RAW_B(uint8_t slot);
void Pick_RAW_C(uint8_t slot);

/* 原料区取料（备用路线，当前未调用）：摆 POS_OBSERVE_RAW_PLUS 读一次三框颜色(0x03)，
   按任务码顺序把每一色分派给 Pick_RAW_A/B/C，并按 (初始格 + 已抓块数) % 3 推算转盘走格。
   切换方法见 pick_task.c 里它的注释。 */
void Pick_RAW_PLUS(void);

/* 粗加工区放置前的视觉对正：摆圆环观察位 → 开爪 → 0x04 拿偏差 → AGV_Correct(30000) → 收爪。
   拿不到偏差就不修正（免得用旧 dx/dy 把车带偏）。由 task.c 的 case Place1 在
   Place_ROUGH 之前调用一次，Place_ROUGH 自己不再做这一步。 */
void Place_Correct(void);

/* 粗加工区放置（不含视觉修正，那一步在 Place_Correct 里）：把车上三个槽
   依次放到二维码给的圆环上。 */
void Place_ROUGH(void);
void Pick_ROUGH(void);
void Place_TEMP(void);
/* 暂存区放置前的视觉对正【第二轮】：摆 POS_OBSERVE_TEMP → 开爪 → 0x05 指定颜色 → AGV_Correct(5000) → 收爪。
   第一轮别调它 —— 第一轮的观察位和粗加工区同一处，复用 Place_Correct()。
   task.c 的 case Place2 按 RerunFlag 二选一。 */
void Place_TEMP_Correct(void);

#endif /* __PICK_TASK_H */
