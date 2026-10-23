#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "Emm_V5.h"
#include "main.h"
#include "delay.h"
#include "yawPIDcontroller.h"
#include "radar_task.h"

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
void AGV_Position_Depart();
void AGV_Position_Qcode_to_Raw();
void AGV_Position_Raw_to_Rough();
void AGV_Position_Rough_to_Temp();
void AGV_Position_Temp_to_Raw();
void AGV_Position_GoBack();
/* 多段连成一条折线一次走完（段间拐点也圆角），只跑图时用；R=AGV_ARC_R_MM */
void AGV_Position_CruiseThroughRoute(const uint8_t *legOrder, uint8_t legCount, uint8_t endDir);

#endif