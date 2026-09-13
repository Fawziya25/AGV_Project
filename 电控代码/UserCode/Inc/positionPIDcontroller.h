#ifndef __POSITIONPIDCONTROLLER_H
#define __POSITIONPIDCONTROLLER_H

#include "main.h"
#include "Emm_V5.h"
#include "delay.h"

#define POSITIONPID_MAX_INTEGRAL 3000
#define POSITIONPID_MAX_OUTPUT 200000.0f

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
}PositionPID_TypeDef;

extern PositionPID_TypeDef positionXPID;
extern PositionPID_TypeDef positionYPID;

void PositionUpdate(void);
void PositionPID_Init(PositionPID_TypeDef *positionPID, float kp, float ki, float kd);
void PositionPID_Calculate(PositionPID_TypeDef *positionPID);
void AGV_Position_Set(float targetX, float targetY);
void AGV_Position_Loop(void);

#endif