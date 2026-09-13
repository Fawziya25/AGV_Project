#ifndef __YAW_PID_CONTROLLER_H__
#define __YAW_PID_CONTROLLER_H__   

#include "main.h"
#include "Emm_V5.h"
#include "delay.h"
#include "hwt101.h"
#include "oled.h"
#include "stdio.h"

#define IntegralLimit 80.0f

extern uint8_t TurnCpltFlag;

typedef struct {
	float kp;
    float ki;
    float kd;
    float target;
    float current;
    float error;
    float prevError;
    float integral;
    float output;
}YawPID_TypeDef;

extern YawPID_TypeDef yawPID;
void AGV_SetYaw(float yaw);
void AGV_SetOmega(float omega);
void YawPID_Init(float kp, float ki, float kd);

#endif