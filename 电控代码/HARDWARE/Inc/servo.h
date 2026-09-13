#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

/**********************************************************
 * PWM 舵机驱动（50Hz，脉宽 500us~2500us）
 * SERVO_Z   : Z轴旋转舵机, TIM2_CH1 = PA5, 量程0°~360°(500~2500us)
 * SERVO_GRIP: 夹爪舵机, TIM3_CH1 = PA6, 量程0°~270°(500~2500us)
 * 引脚/角度映射可在 servo.c 顶部配置区修改
 * 舵机平滑加减速参数见下方 SERVO_RAMP_* 配置
 **********************************************************/

/* ==================== 舵机平滑加减速(梯形曲线)配置 ====================
   开头/结尾各 SERVO_RAMP_STEPS 步缓慢加减速，中间按设定速度走；
   加速/减速段各自长度 = SERVO_RAMP_STEPS * stepDeg（°）；设 SERVO_RAMP_STEPS=0 即退化为匀速 */
#define SERVO_RAMP_STEPS  30      /* 加减速段步数：越多，加减速距离越长、越缓 */
#define SERVO_RAMP_START  0.1f   /* 起步/收尾首步角度占 stepDeg 的比例(0~1)，越小起步越慢、加速度越小 */

/* 平滑转动默认步长/间隔/到位裕量（原 pick_ctrl 的 PICK_SERVO_* 收敛于此） */
#define SERVO_STEP_DEG    0.8f   /* 平滑转动每步角度(°)，越小越慢 */
#define SERVO_STEP_MS     2      /* 平滑转动每步间隔(ms)，越大越慢 */
#define SERVO_SETTLE_MS   300    /* 平滑转动到位后的物理到位裕量(ms) */

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
