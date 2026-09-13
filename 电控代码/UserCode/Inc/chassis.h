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