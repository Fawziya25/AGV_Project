#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "Emm_V5.h"
#include "main.h"
#include "delay.h"


#define a 100
#define b 115 

void AGV_Position_GoStraight(uint32_t distance, uint16_t vel, uint8_t dir);
void AGV_Position_Translate(uint32_t distance, uint16_t vel, uint8_t dir);
void AGV_Position_Rotate(uint8_t dir);
void AGV_Position_Depart_1();
void AGV_Position_GoBack();

#endif