#ifndef __PICK_CTRL_H
#define __PICK_CTRL_H
/* 预设位置宏(括号宏)：task.c 里用不带括号方式调用 Pick_MoveTo POS_XXX;
   例如 Pick_MoveTo POS_A;  展开为 Pick_MoveTo(30.0f, 60.0f, 0.0f) */
//======================角度z    高度z   半径r=====================
#define POS_DEFAULT  (3.0f,   0.0f,    0.0f)//断电和上电前的复位
#define POS_START    (33.0f,  12.0f,   74.0f)//车运行前的位置
#define POS_RUN      (33.0f,  12.0f,   4.0f)//车移动时候位置

#define POS_OBSERVE       (184.0f, 10.0f,  4.0f)//下降100mm，距离地面约180mm
#define POS_OBSERVE_RAW   (184.0f, 10.0f,  4.0f)//观察原料上的物料块位置
#define POS_OBSERVE_CIRCLE (184.0f, 140.0f,  4.0f)//观察圆环位置
#define POS_OBSERVE_TEMP  (184.0f, 80.0f,  4.0f)//观察暂存区上物料块位置

#define POS_RAW       (184.0f, 67.0f,  4.0f )//原料区抓取位置正下方
#define POS_RAW_LEFT  (214.0f, 67.0f,  30.0f)//原料区抓取位置左前方
#define POS_RAW_RIGHT (154.0f, 67.0f,  30.0f)//原料区抓取位置右边前方

#define POS_A        (3.0f,   31.0f,  4.0f)//车上放置位置
#define POS_B        (43.0f,  31.0f,  4.0f)
#define POS_C        (325.0f, 31.0f,  4.0f)

#define POS_1        (230.0f, 141.0f,  58.0f)//圆环左边
#define POS_2        (184.0f, 141.0f,  4.0f)//圆环中间
#define POS_3        (138.0f, 141.0f,  58.0f)//圆环右边
#define POS_1_HIGE   (230.0f, 83.0f,  58.0f)//圆环物块左边
#define POS_2_HIGE   (184.0f, 83.0f,  4.0f)//圆环物块中间
#define POS_3_HIGE   (138.0f, 83.0f,  58.0f)//圆环物块右边

/* 夹爪舵机角度参数(°)：调整角度只需改这里，pick_ctrl.c 夹爪函数引用 */
#define PICK_OBSERVE    0.0f   /* 张开观察 */
#define PICK_GRAB      112.0f   /* 抓住物料 */
#define PICK_RELEASE   72.0f   /* 放开物料 */

#include <stdint.h>
#include "pick_ctrl.h"
#include "positionPIDcontroller.h"
#include "openmv.h"
#include "QRcode.h"
#include "chassis.h"
#include "usart.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>



/**********************************************************
 * 物料抓取模块（pick_ctrl）
 * 当前版本：视觉默认已对准，只控制 2 舵机 + 2 步进电机完成夹取动作。
 * 依赖：servo（PWM 舵机）、Emm_V5（CAN 步进电机，与四轮同协议）
 **********************************************************/

/* 模块初始化（在 AGV_Init 中调用一次） */
void Pick_Init(void);

/* 基本动作：Z舵机就位后，电机走门字形(Π形)轨迹到目标位置。
   servoZ: Z舵机 0~360°; motorZ_mm/motorR_mm: 高度/半径(mm)，越界自动钳位。
   夹爪不在此控制，由 Pick_Test_Transfer 或调用方按需开合。 */
void Pick_MoveTo(float servoZ, float motorZ_mm, float motorR_mm);
void Pick_Direct_MoveTo(float servoZ, float motorZ_mm, float motorR_mm);



void Pick_Test_Transfer(void);



void Pick_Gripper_Grab(void);
void Pick_Gripper_Release(void);
void Pick_Gripper_Observe(void);

/* 逐个模块测试（桌面调试，均为死循环） */
void Pick_Test_ZServo(void);    /* Z舵机: 0 -> +30 -> -30 循环 */
void Pick_Test_Gripper(void);   /* 夹爪: 0 -> 180 开合循环 */
void Pick_Test_MotorZ(void);    /* 高度电机: 往返 */
void Pick_Test_MotorR(void);    /* 半径电机: 往返 */

/* 桌面调试入口（当前指向 Pick_Test_ZServo） */
void Pick_DebugTest(void);

#endif
