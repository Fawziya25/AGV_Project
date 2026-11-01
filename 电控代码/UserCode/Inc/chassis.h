#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "Emm_V5.h"
#include "main.h"
#include "delay.h"
#include "yawPIDcontroller.h"
#include "radar_task.h"

/* ---------------- 底盘几何与传动（2026-10-27 用户确认） ----------------
   横向半轮距 114mm、纵向半轮距 95.5mm，麦轮**纵向安装**（滚动方向沿车身前后）。
   轮径 75mm → 周长 π×75 ≈ 235.62mm；驱动器 3200 脉冲/圈。
   原地旋转时的力臂 = **横向**半轮距 + **纵向**半轮距（= 209.5mm）：麦轮辊子轴线
   与车身成 45°，纵向偏移经辊子耦合进轮子行程（经典麦轮逆解的 (l1+l2)·ω）。
   （2026-11-01 更正：原文写"纵向半轮距不参与原地旋转、被 45° 辊子吃掉"是错的，
     据此算出的旋转量只有几何值的 0.54 倍，会让漂移走位偏掉 1/3 车程。）
   注意：下面这两个遗留的 a/b 宏没有引用（且 a/b 这种名字会污染变量名），真几何用上面这组。 */
#define CHASSIS_PI                3.14159265f
#define WHEEL_DIAMETER_MM         75.0f
#define WHEEL_CIRCUMFERENCE_MM    (CHASSIS_PI * WHEEL_DIAMETER_MM)   /* ≈235.62mm */
#define WHEEL_PULSES_PER_REV      3200.0f                            /* 脉冲/圈 */
#define WHEEL_HALF_TRACK_MM       114.0f   /* 横向半轮距（原地旋转力臂的一半） */
#define WHEEL_HALF_WHEELBASE_MM   95.5f    /* 纵向半轮距（另一半，别丢） */

#define a 100
#define b 115

/* ---------------- 场地坐标(mm) ----------------
   下面这些宏是"x, y"两个数打包成一个宏(带逗号)，用法：
       AGV_Position_Go(Raw, 0);   →  AGV_Position_Go(1200, 300, 0);
   所以只能整体当实参用，不能出现在赋值或运算里。 */
#define Origin_X 165                 // 初始X坐标
#define Origin_Y 165                 // 初始Y坐标

#define Vel 150                    // 默认行驶速度(RPM)

#define QRCode 360, 1110           // 二维码区坐标
#define Raw 1200, 300              // 原料区坐标
#define Raw_Depart 1200, 400       // 原料区出发点坐标
#define Rough 1200, 2150           // 粗加工区坐标
#define Temp 2090, 1230            // 暂存区坐标
#define Waypoint_LB 360, 300       // 左下角目标点坐标
#define Waypoint_RB 2090, 300      // 右下角目标点坐标
#define Waypoint_LF 400, 2150      // 左上角目标点坐标
#define Waypoint_RF 2090, 2150     // 右上角目标点坐标
#define Destination 165, 2285        // 目标点坐标

void AGV_Position_GoStraight(uint32_t distance, uint16_t vel, uint8_t dir);
void AGV_Position_Translate(uint32_t distance, uint16_t vel, uint8_t dir);
void AGV_Position_Rotate(uint8_t dir);
/* 坐标导航：Set 重新锚定小车当前坐标，Go 按 AGV_Direction 自动拆成直行/平移走到(x,y) */
void AGV_Position_Init();
void AGV_Position_Set(uint16_t x, uint16_t y);
void AGV_Position_Go(uint16_t x, uint16_t y, uint8_t priDir);
/* 漂移前进：平移与转向**同时**进行的一次动作，走到 (x,y) 时车头正好是 tarDir
   (0:X+ 1:Y+ 2:X- 3:Y-)，速度用宏 Vel。车头与运动方向全程不一致（这就是漂移）；
   目标朝向与当前相同时退化成纯平移/纯斜行。落点按弧线弦做修正，朝向末尾用
   AGV_SetYaw() 闭环收口。与 Go() 的区别：Go 是先直行再平移的两段折线、转向另做。
   1=收到到位帧，0=等到位超时（坐标仍按目标锚定，开环）。 */
uint8_t AGV_Position_Drift(uint16_t x, uint16_t y, uint8_t tarDir);
void AGV_Position_Depart();
void AGV_Position_Qcode_to_Raw();
void AGV_Position_Raw_to_Rough();
void AGV_Position_Rough_to_Temp();
void AGV_Position_Temp_to_Raw();
void AGV_Position_GoBack();

#endif