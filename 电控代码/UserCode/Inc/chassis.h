#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "Emm_V5.h"
#include "main.h"
#include "delay.h"
#include "yawPIDcontroller.h"

#define a 100
#define b 115 

#define Origin_X 0 // 初始X坐标
#define Origin_Y 0 // 初始Y坐标

#define Vel 100 // 行驶速度为100

#define QRCode 400, 1200           // 二维码区坐标
#define Raw 1200, 300              // 原料区坐标
#define Raw_Depart 1200, 400       // 原料区出发点坐标
#define Rough 1200, 2115           // 粗加工区坐标
#define Temp 2115, 1200            // 暂存区坐标
#define Waypoint_LB 400, 300       // 左下角目标点坐标
#define Waypoint_RB 2115, 300      // 右下角目标点坐标
#define Waypoint_LF 400, 2115      // 左上角目标点坐标
#define Waypoint_RF 2115, 2115     // 右上角目标点坐标

void AGV_Position_GoStraight(uint32_t distance, uint16_t vel, uint8_t dir);
void AGV_Position_Translate(uint32_t distance, uint16_t vel, uint8_t dir);
void AGV_Position_Rotate(uint8_t dir);
void AGV_Position_Depart_1();
void AGV_Position_Qcode_to_Raw();
void AGV_Position_Raw_to_Rough();
void AGV_Position_Rough_to_Temp();
void AGV_Position_Temp_to_Raw();
void AGV_Position_Depart_2();
void AGV_Position_GoBack_1();
void AGV_Position_GoBack_2();

#endif