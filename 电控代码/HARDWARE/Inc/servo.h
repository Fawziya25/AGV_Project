#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

/**********************************************************
 * PWM 舵机驱动（50Hz，脉宽 500us~2500us）
 * SERVO_Z   : Z轴旋转舵机, TIM2_CH1 = PA5, 量程0°~360°(500~2500us)
 * SERVO_GRIP: 夹爪舵机, TIM3_CH1 = PA6, 量程0°~270°(500~2500us)
 * 引脚/角度映射可在 servo.c 顶部配置区修改
 **********************************************************/

typedef enum
{
    SERVO_Z = 0,     /* Z轴旋转舵机(270°) */
    SERVO_GRIP = 1,  /* 夹爪舵机(180°) */
} ServoId;

void Servo_Init(void);
void Servo_SetAngle(ServoId id, float angle);   /* 角度范围按各舵机配置 */
/* 读取舵机当前命令角(°)：由比较寄存器反推，位置舵机无物理回读 */
float Servo_GetAngle(ServoId id);
/* 平滑转动：分步逼近目标角，模拟慢转。stepDeg=每步角度(°)，stepDelayMs=每步间隔(ms) */
void Servo_SetAngleSmooth(ServoId id, float angle, float stepDeg, uint16_t stepDelayMs);

#endif
