#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include <stdbool.h>

/**********************************************************
 * PWM 舵机驱动（50Hz，脉宽 500us~2500us）
 * SERVO_Z   : Z轴旋转舵机, TIM1_CH1 = PE9, 量程0°~270°(500~2500us)
 * SERVO_GRIP: 夹爪舵机, TIM3_CH1 = PA6, 量程0°~180°(500~2500us)
 *             【2026-10-14 换舵机：270°→180°。脉宽跨度没变，所以每度脉宽
 *               7.41→11.11us/°，**同一个命令角物理转 1.5 倍远** —— pick.h 里
 *               PICK_OBSERVE/PICK_GRAB/PICK_RELEASE 那三个夹爪角度这次没
 *               跟着缩，全偏了，必须重对。详见 servo.c 夹爪配置区那段。】
 * SERVO_TURN: 车上转盘舵机, TIM2_CH1 = PA5, 量程0°~270°(500~2500us)
 *             30°=A 槽, 150°=B 槽, 270°=C 槽，上电初值 0°=A
 * SERVO_RADAR: 雷达收放舵机, **TIM4_CH1 = PD12**, 量程0°~360°(500~2500us)
 *             **0° = 雷达伸出（扫描时用），340° = 雷达收入车内（行驶时用）**
 *             【2026-10-14：收起角 360°→180°(保险)→300°→**340°**(用户确认)；
 *               舵机量程没变，仍是 0~360°。0°↔340° 是 340° 大行程，单程约 0.95s。】
 *             上电初值 340°（收起 = 2389us），避免一上电雷达弹出来。
 *             角度语义在 radar_config.h 的 RADAR_SERVO_OUT_DEG / IN_DEG，
 *             时序封装在 radar_task 的 Radar_SetDeployed()。
 * 引脚/角度映射可在 servo.c 顶部配置区修改
 * 【2026-10-14：Z轴 与 转盘 两路接线对调过（原 Z=PA5/TIM2、转盘=PE9/TIM1）。
 *   这里只是注释，改引脚只改 servo.c 顶部配置区。】
 * 舵机平滑加减速参数见下方 SERVO_RAMP_* 配置
 **********************************************************/

/* ==================== 舵机运动参数（三路各自独立） ====================
   梯形速度曲线：开头/结尾各 RAMP_STEPS 步缓慢加减速，中间按 STEP_DEG 走；
   加速/减速段长度 = RAMP_STEPS * STEP_DEG（°）；RAMP_STEPS=0 即退化为匀速。
   STEP_DEG/STEP_MS 越小越慢；SETTLE_MS 是转到位后额外等待的物理裕量。
   三路舵机负载特性不同（Z轴行程长惯性大；夹爪行程短且要顶住物料；
   转盘带料一转 120°，没停稳臂就下去会撞），参数各调各的，互不影响。 */

/* --- Z轴旋转舵机 (TIM1_CH1 = PE9) --- */
#define SERVO_Z_RAMP_STEPS 30     /* 加减速段步数：越多，加减速距离越长、越缓 */
#define SERVO_Z_RAMP_START 0.15f   /* 起步/收尾首步占 STEP_DEG 的比例(0~1)，越小起步越慢 */
#define SERVO_Z_STEP_DEG   0.8f   /* 每步角度(°)，越小越慢 */
#define SERVO_Z_STEP_MS    2      /* 每步间隔(ms)，越大越慢 */
#define SERVO_Z_SETTLE_MS  300    /* 到位后的物理到位裕量(ms) */

/* --- 夹爪舵机 (TIM3_CH1 = PA6) --- */
#define SERVO_G_RAMP_STEPS 30     /* 同 Z轴：加减速段步数 */
#define SERVO_G_RAMP_START 0.1f   /* 同 Z轴：起步/收尾首步比例 */
#define SERVO_G_STEP_DEG   1.0f   /* 每步角度(°)，夹爪要慢，可调小 */
#define SERVO_G_STEP_MS    2      /* 每步间隔(ms)，夹爪要慢，可调大 */

/* --- 车上转盘舵机 (TIM2_CH1 = PA5) ---
   量程 0°~270°，30°/150°/270° = A/B/C 槽，上电初值 0°=A */
#define SERVO_T_RAMP_STEPS 0     /* 加减速段步数：0 = 匀速分步(无加减速)，见下面说明 */
#define SERVO_T_RAMP_START 0.8f   /* 起步/收尾首步比例(RAMP_STEPS=0 时不起作用) */
#define SERVO_T_STEP_DEG   0.8f   /* 每步角度(°) */
#define SERVO_T_STEP_MS    2.0f      /* 每步间隔(ms)：0.8°/1ms ≈ 800°/s，120° 走 150ms */
#define SERVO_T_SETTLE_MS  300    /* 到位后物理裕量(ms)：分步只保证命令发完，
                                     舵机追命令有滞后，没停稳臂就下去会撞料。
                                     §8 定的就是 300，中间被改成过 0，现改回来。 */

/* --- 雷达收放舵机 (TIM4_CH1 = PD12) ---
   方向/量程/上电行为这几项都在 servo.c 的雷达配置区：
     SERVO_R_INVERT     0=正常 1=反向（角度→脉宽翻转，判断"是不是装反了"用）
     SERVO_R_MAX_ANG    舵机**真实行程**（360/270/180…，按型号填，影响角度含义）
     SERVO_R_START_ANG  上电初值（收起位）
   0° = 伸出、340° = 收入（2026-10-14 用户确认 340°；量程仍 0~360°）；
   伸出/收起要慢一点，别把雷达甩出去/夹一下。 */
#define SERVO_R_RAMP_STEPS 20     /* 加减速段步数 */
#define SERVO_R_RAMP_START 0.2f   /* 起步/收尾首步比例 */
#define SERVO_R_STEP_DEG   1.5f   /* 每步角度(°)：0°↔340° 行程 ≈ 227 步(≈454ms) */
#define SERVO_R_STEP_MS    2      /* 每步间隔(ms)：1.5°/2ms ≈ 750°/s，整程约 720ms */
#define SERVO_R_SETTLE_MS  500    /* 到位后的物理裕量(ms)：雷达伸/收到位后再扫描或开跑 */

/* 上电时要不要初始化雷达舵机并锁到收起位：
   1 = 要（默认，上电即收起）
   0 = **上电先不动**（正在标定"收起角/方向"时用，避免一上电就往错的方向跑）；
       第一次 Radar_SetDeployed() 时仍会初始化并按目标角走。 */
#ifndef SERVO_R_INIT_AT_BOOT
#define SERVO_R_INIT_AT_BOOT  1
#endif

typedef enum
{
    SERVO_Z = 0,     /* Z轴旋转舵机(270°)：2026-10-14 由 360° 改过来(换舵机) */
    SERVO_GRIP = 1,  /* 夹爪舵机(180°)：2026-10-14 由 270° 改过来(换舵机) */
    SERVO_TURN = 2,  /* 车上转盘舵机(270°)：30°/150°/270° = A/B/C 槽 */
    SERVO_RADAR = 3, /* 雷达收放舵机(量程360°)：0°=伸出、340°=收入车内 */
} ServoId;

void Servo_Init(void);
/*
 * 只初始化**一路**舵机（不碰其他三路）。
 * 测试模式里 AGV_Init()/Pick_Init() 没跑、机械臂可能是任意姿态，
 * 这时只想动雷达舵机就不能调 Servo_Init()（那会顺手把 Z轴/夹爪/转盘
 * 一起复位到各自上电初值，机械臂会突然动）。重复调用是安全的。
 */
void Servo_InitId(ServoId id);
/* 该路舵机是否已初始化（PWM 已起）。1 = 已初始化。 */
bool Servo_IsReady(ServoId id);
void Servo_SetAngle(ServoId id, float angle);   /* 角度范围按各舵机配置 */
/* 读取舵机当前命令角(°)：由比较寄存器反推，位置舵机无物理回读 */
float Servo_GetAngle(ServoId id);
/* 平滑转动：分步逼近目标角，模拟慢转。stepDeg=每步角度(°)，stepDelayMs=每步间隔(ms) */
void Servo_SetAngleSmooth(ServoId id, float angle, float stepDeg, uint16_t stepDelayMs);

#endif
