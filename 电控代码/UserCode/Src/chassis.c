#include "chassis.h"
#include "Emm_V5.h"
#include "delay.h"
#include "radar_task.h"
#include "stm32f4xx_hal.h"
#include "yawPIDcontroller.h"
#include <stdint.h>
#include <stdlib.h>

/* 小车当前坐标(mm)，上电记作原点；每次 AGV_Position_Go() 走完自动锚定到目标点 */
uint16_t AGV_Position_X;
uint16_t AGV_Position_Y;

uint16_t Points_X[25];
uint16_t Points_Y[25];

void AGV_Position_Init(void)
{
	// 小车初始坐标，需要根据启停区设置
    AGV_Position_X = 150;
    AGV_Position_Y = 2250;

	// 所有拐点坐标(mm)
	Points_X[0] = 310;
	Points_Y[0] = 2090;
	// 2: QRCode
	Points_X[2] = 310;
	Points_Y[2] = 1200;
	Points_X[4] = 310;
	Points_Y[4] = 310;
	// 10: Rough
	Points_X[10] = 1200;
	Points_Y[10] = 2090;
	Points_X[12] = 1200;
	Points_Y[12] = 1200;
	// 14：Raw
	Points_X[14] = 1200;
	Points_Y[14] = 310;
	Points_X[20] = 2090;
	Points_Y[20] = 2090;
	// 22: Temp
	Points_X[22] = 2090;
	Points_Y[22] = 1200;
	Points_X[24] = 2090;
	Points_Y[24] = 310;
}

/**
 * @brief 直行
 * 
 * @param distance 距离
 * @param vel 速度
 * @param dir 方向
 * 0：向后移动
 * 1：向前移动
 */
void AGV_Position_GoStraight(uint32_t distance, uint16_t vel, uint8_t dir)
{
	float n = (float)distance / WHEEL_CIRCUMFERENCE_MM;  // 圈数（周长/脉冲数见 chassis.h 几何宏）
	uint32_t p = (uint32_t)(n * WHEEL_PULSES_PER_REV);
	dir^=1;
	Emm_V5_ClearRxFrame();						// 清历史回帧：后面等的是本次动作新到的到位帧
	Emm_V5_Pos_Control(1, Wheel1PositiveDir^dir, vel, 20, p, 2, 1); // 多机同步标志位置1
		HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_Pos_Control(2, Wheel2PositiveDir^dir, vel, 20, p, 2, 1); // 多机同步标志位置1
		HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_Pos_Control(3, Wheel3PositiveDir^dir, vel, 20, p, 2, 1); // 多机同步标志位置1
		HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_Pos_Control(4, Wheel4PositiveDir^dir, vel, 20, p, 2, 1); // 多机同步标志位置1
		HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_Synchronous_motion(0); 								 // 广播地址0触发
	HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_WaitArrive(EMM_V5_ARRIVE_TMO_MS);		// 等电机到位回帧(0xFD 0x9F)，超时兜底不卡死

	HAL_Delay(200);
}

/**
 * @brief 平移移动
 * 
 * @param distance 距离
 * @param vel 速度
 * @param dir 方向
 * 0：向右移动
 * 1：向左移动
 */
void AGV_Position_Translate(uint32_t distance, uint16_t vel, uint8_t dir)
{
	float n = (float)distance / WHEEL_CIRCUMFERENCE_MM;  // 圈数（周长/脉冲数见 chassis.h 几何宏）
	uint32_t p = (uint32_t)(n * WHEEL_PULSES_PER_REV);

	Emm_V5_ClearRxFrame();						// 清历史回帧：后面等的是本次动作新到的到位帧
	Emm_V5_Pos_Control(1, Wheel1PositiveDir^dir, vel, 20, p, 2, 1); // 多机同步标志位置1
		HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_Pos_Control(4, Wheel2PositiveDir^dir, vel, 20, p, 2, 1); // 多机同步标志位置1
		HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_Pos_Control(2, Wheel3PositiveDir^dir^1, vel, 20, p, 2, 1); // 多机同步标志位置1
		HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_Pos_Control(3, Wheel4PositiveDir^dir^1, vel, 20, p, 2, 1); // 多机同步标志位置1
		HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_Synchronous_motion(0); 								 // 广播地址0触发
	HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_WaitArrive(EMM_V5_ARRIVE_TMO_MS);		// 等电机到位回帧(0xFD 0x9F)，超时兜底不卡死

	HAL_Delay(200);
}

/**
 * @brief 重新锚定小车位置
 *
 * @param x X坐标
 * @param y Y坐标
 */
void AGV_Position_Set(uint16_t x, uint16_t y)
{
	AGV_Position_X = x;
	AGV_Position_Y = y;
}

/**
 * @brief 移动到目标位置
 *
 * @param x 目标X坐标
 * @param y 目标Y坐标
 * @param priDir 优先方向 0:X, 1:Y
 */
void AGV_Position_Go(uint16_t x, uint16_t y, uint8_t priDir)
{
	int32_t sdx = (int32_t)x - (int32_t)AGV_Position_X;
	int32_t sdy = (int32_t)y - (int32_t)AGV_Position_Y;
	uint16_t dx = (sdx < 0) ? (uint16_t)(-sdx) : (uint16_t)sdx;
	uint16_t dy = (sdy < 0) ? (uint16_t)(-sdy) : (uint16_t)sdy;
	uint8_t x_flag = (sdx >= 0) ? 1 : 0;
	uint8_t y_flag = (sdy >= 0) ? 1 : 0;
	Yaw_MovingOffsetStart();
	// X方向先行
	if(priDir == 0)
	{
		if(AGV_Direction == 0)
		{
			AGV_Position_GoStraight(dx, Vel, x_flag);
			AGV_Position_Translate(dy, Vel, y_flag);
		}
		else if(AGV_Direction == 1)
		{
			AGV_Position_Translate(dx, Vel, x_flag^1);
			AGV_Position_GoStraight(dy, Vel, y_flag);
		}
		else if (AGV_Direction == 2)
		{
			AGV_Position_GoStraight(dx, Vel, x_flag^1);
			AGV_Position_Translate(dy, Vel, y_flag^1);
		}
		else if (AGV_Direction == 3)
		{
			AGV_Position_Translate(dx, Vel, x_flag);
			AGV_Position_GoStraight(dy, Vel, y_flag^1);
		}
	}
	// Y方向先行
	else if(priDir == 1)
	{
		if(AGV_Direction == 0)
		{
			AGV_Position_Translate(dy, Vel, y_flag);
			AGV_Position_GoStraight(dx, Vel, x_flag);
		}
		else if(AGV_Direction == 1)
		{
			AGV_Position_GoStraight(dy, Vel, y_flag);
			AGV_Position_Translate(dx, Vel, x_flag^1);
		}
		else if (AGV_Direction == 2)
		{
			AGV_Position_Translate(dy, Vel, y_flag^1);
			AGV_Position_GoStraight(dx, Vel, x_flag^1);
		}
		else if (AGV_Direction == 3)
		{
			AGV_Position_GoStraight(dy, Vel, y_flag^1);
			AGV_Position_Translate(dx, Vel, x_flag);
		}
	}
	Yaw_MovingOffsetEnd();
	// 重新锚定小车位置
	AGV_Position_Set(x, y);
}

/* ==================== 漂移前进（边滑边转，一次到位） ====================
 * 漂移 = **车头朝向与实际运动方向不一致**的滑行：底盘在一次连续动作里同时完成
 * "平移"和"转向"，车头从当前朝向前转到 tarDir，到点时**位置和朝向同时**到位。
 * 它不是"先原地转到目标朝向、再斜着走"——那种走法转的时候车是停着的，是两段。
 *
 * —— 轮子上怎么叠加
 * 麦轮是线性的：轮子转角 = 平移分量 + 旋转分量，各算各的再相加。
 *   平移（前进 s、左移 t，单位 mm）：
 *       轮1 = t - s     轮2 = s + t     轮3 = s - t     轮4 = -(s + t)
 *     （与 GoStraight / Translate 的字节逐位一致，见下面的 FWD/LEFT 位）
 *   旋转（左转 Δθ）：四个轮子同向同量，每轮行程 = K × Δθ，K 见 DRIFT_MM_PER_DEG
 * 四个轮子的速度按**各自总行程**等比给，同时启动、同时停 —— 平移和转向才是同时发生的。
 *
 * —— 为什么平移分量要先转 -Δθ/2 再放大
 * 车体一边走一边转，世界系里的轨迹是一条圆弧，落点在**弦**上；弦的方向等于首尾
 * 两个车头朝向的平均值（即转到一半时的朝向）。所以要落在指定点上，车体系的平移
 * 方向必须提前转 Δθ/2，长度按弦长换算：
 *     车体系合位移 = |D| × (Δθ/2)/sin(Δθ/2)，方向 = D 提前转 -Δθ/2
 * Δθ=90° 时系数 1.1107（500mm 的弦，轮子上要走 555mm）；Δθ=0 时系数 1，
 * 退化成纯平移/纯斜行。不修正的话，90° 漂移的落点会偏掉约半个身位。
 *
 * —— 航向怎么保证
 * 旋转量按实车几何给（K = (横向半轮距 114 + 纵向半轮距 95.5)mm × π/180，见
 * DRIFT_MM_PER_DEG），但辊子打滑、地面摩擦都会让它有误差，所以到位后用
 * AGV_SetYaw(yaw0 + Δθ) **闭环收口**一次，
 * 把车头真正拉到目标朝向，并更新 AGV_Direction。
 *
 * ⚠ 本段**有意旋转**，绝不能进 Yaw_MovingOffset 窗口：那个量的语义是"本应为 0 的
 *   航向变化"，把 Δθ 记进去，下一次转弯就会少转一整段。航向误差由末尾闭环收口负责。
 * ⚠ 位置仍是开环：K 不准时弧线形状会变、落点也跟着偏（朝向由收口兜住）。K 必须标定。
 * ================================================================= */
/* ---- 漂移参数：函数体里不留裸数字，标定/调整只改这一块 ---- */
/* 轮子行程 → 脉冲。与 GoStraight / Translate 同一换算（几何宏在 chassis.h） */
#define DRIFT_PUL_PER_MM     (WHEEL_PULSES_PER_REV / WHEEL_CIRCUMFERENCE_MM)   /* ≈13.581 脉冲/mm */

/* 车体转 1° 时每个轮子的行程(mm)：
   麦轮辊子轴线与车身成 45°，绕中心转时接地点速度 v = ω×r，
   轮子行程 = (v·n)/(e_x·n)，而 e_x·n = ±1/√2 与约束式里的 1/√2 正好相消，于是
       每轮行程 = ω × (**横向**半轮距 + **纵向**半轮距)
   两项都进：纵向偏移经 45° 辊子耦合进了轮子行程，这正是经典麦轮逆解里的 (l1+l2)·ω。
       K = (114 + 95.5) × π/180 = 3.65646 mm/°   →   90° = 329.08mm/轮 ≈ 4469 脉冲
   ⚠ 2026-11-01 更正：这里原来只取横向半轮距（K = 1.98968mm/°，90° 只有 2432 脉冲），
   等于把麦轮当成"只能沿车身前后滚的普通轮"，旋转量只有几何值的 0.54 倍：
   弧线几乎走直、落点偏 1/3 车程（500mm 的 90° 漂移偏约 189mm，180° 偏约 456mm），
   朝向虽被末尾 AGV_SetYaw() 兜住，但"边滑边转"退化成"直着走 + 末尾原地拧"。
   ⚠ 别反过来向 AGV_SetOmega() 的 51/50 看齐：它反解出的力臂 229.5mm 只比这里的几何值
   209.5mm 高 9.6%，陀螺闭环本来就吸收掉了，那个值其实基本是对的。
   ⚠ 辊子打滑只会让实际转角偏小，标定只能在 209.5 之上再乘系数，不能比它小。 */
#define DRIFT_MM_PER_DEG     ((WHEEL_HALF_TRACK_MM + WHEEL_HALF_WHEELBASE_MM) * CHASSIS_PI / 180.0f)

/* 相邻朝向差 90°（AGV_Direction 一格），rotDeg 表由它推出来 */
#define DRIFT_TURN_STEP_DEG  90.0f

/* 弧线修正系数：A/B 已含弦长放大 (Δθ/2)/sin(Δθ/2)
     0°   → 1, 0；±90° → π/4, ±π/4；180° → 0, π/2 */
#define DRIFT_CHORD_45       (CHASSIS_PI / 4.0f)    /* 0.785398 */
#define DRIFT_CHORD_180      (CHASSIS_PI / 2.0f)    /* 1.570796 */

/* 阈值与时序 */
#define DRIFT_MIN_MOVE_MM    0.5f    /* 位移小于此值且不转向 → 视为已到位，不发帧 */
#define DRIFT_MIN_WHEEL_RPM  1.0f    /* 有行程的轮子最低转速（0 会被当成"不动"） */
#define DRIFT_ACC            20U     /* 位置模式加速度，与底盘其它走位一致 */
#define DRIFT_CMD_GAP_MS     1U      /* 逐轮发帧间隔，防粘包 */
#define DRIFT_SETTLE_MS      200U    /* 到位后的停稳裕量 */
#define DRIFT_YAW_WRAP_DEG   360.0f  /* 航向归一：一圈 360°，收口目标折到 ±180° 内 */

/* 每个轮子在"纯前进"和"纯左移"时的方向位（1/0 就是直接发出去的那个 dir 字节）。
   FWD 位 = GoStraight 前进时的字节：调用方 dir=1，函数内 dir^=1 得 0，故就是 WheelNPositiveDir。
   LEFT 位 = Translate 左移时的字节：调用方 dir=1，写法见 AGV_Position_Translate()。
   WheelNPositiveDir 四个宏已于 2026-10-27 确认正确 → 上面两条纯动作的字节可信，
   斜向只是它们的线性叠加（纯直行/纯平移时本函数逐位还原成原字节），符号无需再怀疑。 */
#define DRIFT_W1_FWD_BIT   (Wheel1PositiveDir)
#define DRIFT_W2_FWD_BIT   (Wheel2PositiveDir)
#define DRIFT_W3_FWD_BIT   (Wheel3PositiveDir)
#define DRIFT_W4_FWD_BIT   (Wheel4PositiveDir)
#define DRIFT_W1_LEFT_BIT  (Wheel1PositiveDir ^ 1U)
#define DRIFT_W2_LEFT_BIT  (Wheel3PositiveDir ^ 1U ^ 1U)
#define DRIFT_W3_LEFT_BIT  (Wheel4PositiveDir ^ 1U ^ 1U)
#define DRIFT_W4_LEFT_BIT  (Wheel2PositiveDir ^ 1U)
/* 该位为 1 表示"这个轮子在此方向上的正向就是 dir=1 那一侧"，反之取 -1 */
#define DRIFT_SGN(bit)     (((bit) != 0U) ? 1.0f : -1.0f)

/**
 * @brief 漂移前进：边平移边转向，一次动作走到 (x, y) 且车头停在 tarDir
 * @param x      目标点 X 坐标(mm)
 * @param y      目标点 Y 坐标(mm)
 * @param tarDir 目标车头朝向 0:X+ 1:Y+ 2:X- 3:Y-（越界自动 &3）
 * @retval 1 = 收到到位帧；0 = 等到位超时（坐标仍按目标点锚定，属开环）
 * @note  一次连续动作 = 平移 + 转向，车头与运动方向全程不一致（这就是漂移）；
 *        转向量 0 时自动退化成纯平移/纯斜行（车头保持不动）。
 *        位置与朝向在动作结束时同时到位：位置靠轮子脉冲（开环），朝向靠末尾
 *        AGV_SetYaw() 闭环收口（吸收 DRIFT_MM_PER_DEG 的误差）。
 *        **不在本函数里原地转向**——那样车会先停一下，就不是漂移了。
 *        夹爪/机械臂不归本函数管，调用方自己保证运输姿态（POS_RUN）。
 */
uint8_t AGV_Position_Drift(uint16_t x, uint16_t y, uint8_t tarDir)
{
	static const uint8_t fwdBit[4]  = { DRIFT_W1_FWD_BIT,  DRIFT_W2_FWD_BIT,  DRIFT_W3_FWD_BIT,  DRIFT_W4_FWD_BIT  };
	static const uint8_t leftBit[4] = { DRIFT_W1_LEFT_BIT, DRIFT_W2_LEFT_BIT, DRIFT_W3_LEFT_BIT, DRIFT_W4_LEFT_BIT };
	/* 弧线修正 (A,B)：车体系位移 ds' = A*ds + B*dt，dt' = -B*ds + A*dt。
	   下标 = (tarDir - 起始朝向) & 3 → 0:不转 1:左90° 2:掉头 3:右90°。
	   A/B 已含弦长系数 (Δθ/2)/sin(Δθ/2)：0→1、±90°→DRIFT_CHORD_45(π/4)、180°→DRIFT_CHORD_180(π/2)。 */
	static const float  chordA[4] = { 1.0f, DRIFT_CHORD_45, 0.0f,  DRIFT_CHORD_45  };
	static const float  chordB[4] = { 0.0f, DRIFT_CHORD_45, DRIFT_CHORD_180, -DRIFT_CHORD_45 };
	/* 有符号转向角(°，左正右负)，与 AGV_Direction 的 +1=左转 约定一致 */
	static const float  rotDeg[4] = { 0.0f, DRIFT_TURN_STEP_DEG, 2.0f * DRIFT_TURN_STEP_DEG, -DRIFT_TURN_STEP_DEG };

	int32_t  sdx, sdy;
	float    ds, dt;              /* 车体系合位移：ds 前后(+前) / dt 左右(+左)，mm */
	float    v[4];                /* 每轮总行程(mm，带符号) = 平移分量 + 旋转分量 */
	uint32_t pulses[4];
	uint16_t spd[4];
	float    vmax = 0.0f;
	float    yaw0, tgt;
	uint8_t  dth;
	uint8_t  i;
	uint8_t  ok;

	tarDir &= 3U;                 /* 归一化，越界值不往下传 */
	dth     = (uint8_t)((tarDir - AGV_Direction + 4U) & 3U);   /* 0/1/2/3 = 不转/左90/掉头/右90 */

	sdx = (int32_t)x - (int32_t)AGV_Position_X;
	sdy = (int32_t)y - (int32_t)AGV_Position_Y;

	/* 场地坐标 → 车体坐标（用**起始**朝向分解，随后再做弧线修正） */
	switch (AGV_Direction)
	{
		case 0:  ds =  (float)sdx;  dt =  (float)sdy;  break;  /* X+：前+X，左+Y */
		case 1:  ds =  (float)sdy;  dt = -(float)sdx;  break;  /* Y+：前+Y，左-X */
		case 2:  ds = -(float)sdx;  dt = -(float)sdy;  break;  /* X-：前-X，左-Y */
		default: ds = -(float)sdy;  dt =  (float)sdx;  break;  /* Y-：前-Y，左+X */
	}

	/* 弧线修正：世界系走的是圆弧、落点在弦上，车体系平移方向要提前转 -Δθ/2 并放大。
	   ⚠ 变量别取名 a / b：chassis.h 顶部有 #define a 100 / #define b 115，会被替换掉。 */
	{
		float ca = chordA[dth];
		float cb = chordB[dth];
		float ds2 = ca * ds + cb * dt;
		float dt2 = -cb * ds + ca * dt;
		ds = ds2;
		dt = dt2;
	}

	for (i = 0U; i < 4U; i++)
	{
		/* 旋转分量：四轮同向同量，左转为正 */
		v[i] = DRIFT_SGN(fwdBit[i]) * ds + DRIFT_SGN(leftBit[i]) * dt
		     + DRIFT_MM_PER_DEG * rotDeg[dth];
		float mag = (v[i] >= 0.0f) ? v[i] : -v[i];
		if (mag > vmax) vmax = mag;
	}

	if (vmax < DRIFT_MIN_MOVE_MM)   /* 已在目标点且不用转向：一帧都不发，只锚定 */
	{
		AGV_Position_Set(x, y);
		return 1U;
	}

	for (i = 0U; i < 4U; i++)
	{
		float mag = (v[i] >= 0.0f) ? v[i] : -v[i];
		float sp  = (float)Vel * mag / vmax;   /* 按总行程等比：平移与转向同时开始、同时结束 */

		pulses[i] = (uint32_t)(mag * DRIFT_PUL_PER_MM + 0.5f);
		if (sp < DRIFT_MIN_WHEEL_RPM) sp = DRIFT_MIN_WHEEL_RPM;   /* 有行程的轮子不能是 0，否则等于不动 */
		spd[i] = (uint16_t)sp;
	}

	/* 记录漂移前的航向，收口时用作绝对基准。**这一段有意旋转，不能进
	   Yaw_MovingOffset 窗口**（那个量是"本应为 0 的航向变化"，把 Δθ 记进去，
	   下一次 AGV_Turn() 就会少转一整段）。 */
	yaw0 = ZeroBias_GetYaw();

	Emm_V5_ClearRxFrame();        /* 清历史回帧：下面等的是本次动作新到的到位帧 */

	for (i = 0U; i < 4U; i++)
	{
		if (pulses[i] == 0U) continue;         /* 该轮净行程为 0（平移与旋转正好抵消），不发 */
		Emm_V5_Pos_Control((uint8_t)(i + 1U),
		                   (v[i] >= 0.0f) ? 1U : 0U,
		                   spd[i], DRIFT_ACC, pulses[i], 2U, 1U);   /* raF=2 相对当前实时位置，snF=1 多机同步 */
		HAL_Delay(DRIFT_CMD_GAP_MS);              /* 防粘包，与 GoStraight/Translate 一致 */
	}

	Emm_V5_Synchronous_motion(0U);            /* 广播触发四轮同时启动 */
	HAL_Delay(DRIFT_CMD_GAP_MS);

	ok = Emm_V5_WaitArrive(EMM_V5_ARRIVE_TMO_MS);   /* 超时兜底，不死等 */

	HAL_Delay(DRIFT_SETTLE_MS);   /* 停稳裕量，与 GoStraight/Translate 收尾一致 */

	/* 闭环收口：开环旋转量 K 有误差，用陀螺把车头真正拉到"漂移前航向 + Δθ"；
	   同时把打滑丢掉的转角补回来。这一步之后朝向由 IMU 保证，不依赖 K。 */
	tgt = yaw0 + rotDeg[dth];
	while (tgt >   DRIFT_YAW_WRAP_DEG * 0.5f) tgt -= DRIFT_YAW_WRAP_DEG;
	while (tgt <= -DRIFT_YAW_WRAP_DEG * 0.5f) tgt += DRIFT_YAW_WRAP_DEG;
	AGV_SetYaw(tgt);

	AGV_Direction = tarDir;       /* 逻辑朝向跟上车头，后续 Go/Turn 才按新朝向算 */
	AGV_Position_Set(x, y);       /* 开环锚定到目标点 */
	return ok;
}

void AGV_Position_Rotate(uint8_t dir)
{
	if(dir == 0)
	{
		Emm_V5_Vel_Control(1, 0, 40, 0, 1);
		HAL_Delay(10);
		Emm_V5_Vel_Control(2, 0, 40, 0, 1);
		HAL_Delay(10);
		Emm_V5_Vel_Control(3, 0, 40, 0, 1);
		HAL_Delay(10);
		Emm_V5_Vel_Control(4, 0, 40, 0, 1);
		HAL_Delay(10);
	}
	else if(dir == 1)
	{
		Emm_V5_Vel_Control(1, 1, 40, 0, 1);
		delay_us(250);
		Emm_V5_Vel_Control(2, 1, 40, 0, 1);
		delay_us(250);
		Emm_V5_Vel_Control(3, 1, 40, 0, 1);
		delay_us(250);
		Emm_V5_Vel_Control(4, 1, 40, 0, 1);
		delay_us(250);
	}
	
	Emm_V5_Synchronous_motion(0); 								 // 广播地址0触发
	delay_us(250);		
	
	Emm_V5_WaitRxFrame(EMM_V5_ACK_TMO_MS);		// 等命令回帧(原为死等，加超时防卡死)
	
	HAL_Delay(1910);
	
	Emm_V5_Stop_Now(1, 1);
	delay_us(250);
	Emm_V5_Stop_Now(2, 1);
	delay_us(250);
	Emm_V5_Stop_Now(3, 1);
	delay_us(250);
	Emm_V5_Stop_Now(4, 1);
	delay_us(250);
	
	Emm_V5_Synchronous_motion(0); 								 // 广播地址0触发
	delay_us(250);	
	
	Emm_V5_WaitRxFrame(EMM_V5_ACK_TMO_MS);   /* 等命令回帧(原为死等，加超时防卡死) */
}

void AGV_Position_Cruise(uint8_t RouteID, uint8_t tarDir)
{
	/* tarDir = 该段走完后的绝对朝向：0:X+ 1:Y+ 2:X- 3:Y-。
	   先归一化（4 当 0）：旧写法传 4 会一路传到 AGV_Turn()，而它只认 0~3，
	   非法值在那里会落到"没有任何分支命中" → 该转的时候不转。 */
	tarDir &= 3U;

	for(uint8_t i = 0; i < Radar_Path[RouteID].count; i++)
	{
		uint16_t tarX = Points_X[Radar_Path[RouteID].points[i]];
		uint16_t tarY = Points_Y[Radar_Path[RouteID].points[i]];
		if(abs(tarX - AGV_Position_X) >= abs(tarY - AGV_Position_Y))
		{
			if(AGV_Direction == 1 || AGV_Direction == 3)
			{
				/* 车头朝 ±Y、本段要改走 X：在 X+/X- 里选【离最终朝向 tarDir 更近】的那个。
				   原写法 abs(tarDir-2) >= abs(tarDir-4) + AGV_Turn(4) 有两个错：
				   ① tarDir=0（要朝 X+）时条件为假 → 选成 X-(反着走)；
				   ② 4 不是合法方向值（AGV_Turn 只认 0~3）。现已改成 0。 */
				if(abs((int)tarDir - 2) >= abs((int)tarDir - 0))AGV_Turn(0);
				else AGV_Turn(2);
			}
		}
		else 
		{
			if(AGV_Direction == 0 || AGV_Direction == 2)
			{
				/* 同理：车头朝 ±X、要改走 Y 时，在 Y+/Y- 里选离 tarDir 更近的。
				   这条原本就是对的，只补注释/显式 int 转换，逻辑不动。 */
				if(abs((int)tarDir - 1) >= abs((int)tarDir - 3))AGV_Turn(3);
				else AGV_Turn(1);
			}
		}
		AGV_Position_Go(tarX, tarY, 0);
	}
	AGV_Turn(tarDir);
}

void AGV_Position_Cruise_Fast(uint8_t RouteID, uint8_t tarDir)
{
	/* tarDir = 该段走完后的绝对朝向：0:X+ 1:Y+ 2:X- 3:Y-。
	   先归一化（4 当 0）：旧写法传 4 会一路传到 AGV_Turn()，而它只认 0~3，
	   非法值在那里会落到"没有任何分支命中" → 该转的时候不转。 */
	tarDir &= 3U;

	for(uint8_t i = 0; i < Radar_Path[RouteID].count; i++)
	{
		uint16_t tarX = Points_X[Radar_Path[RouteID].points[i]];
		uint16_t tarY = Points_Y[Radar_Path[RouteID].points[i]];
		if(abs(tarX - AGV_Position_X) >= abs(tarY - AGV_Position_Y))
		{
			if(AGV_Direction == 1 || AGV_Direction == 3)
			{
				/* 车头朝 ±Y、本段要改走 X：在 X+/X- 里选【离最终朝向 tarDir 更近】的那个。
				   原写法 abs(tarDir-2) >= abs(tarDir-4) + AGV_Turn(4) 有两个错：
				   ① tarDir=0（要朝 X+）时条件为假 → 选成 X-(反着走)；
				   ② 4 不是合法方向值（AGV_Turn 只认 0~3）。现已改成 0。 */
				if(abs((int)tarDir - 2) >= abs((int)tarDir - 0))AGV_Turn(0);
				else AGV_Turn(2);
			}
		}
		else 
		{
			if(AGV_Direction == 0 || AGV_Direction == 2)
			{
				/* 同理：车头朝 ±X、要改走 Y 时，在 Y+/Y- 里选离 tarDir 更近的。
				   这条原本就是对的，只补注释/显式 int 转换，逻辑不动。 */
				if(abs((int)tarDir - 1) >= abs((int)tarDir - 3))AGV_Turn(3);
				else AGV_Turn(1);
			}
		}
		AGV_Position_Go(tarX, tarY, 0);
	}
	AGV_Turn(tarDir);
}

/**
 * @brief 出发
 * 
 */
void AGV_Position_Depart()
{
	// AGV_Position_Go(QRCode, 0);
	AGV_Position_Go(Points_X[0], Points_Y[0], 0);
	AGV_Position_Cruise(0, 3);
}

void AGV_Position_Qcode_to_Raw()
{
	// AGV_Position_Go(Waypoint_LB, 0);
	// AGV_TurnLeft();
	// AGV_Position_Go(Raw, 0);
	AGV_Position_Cruise(1, 0);
}

void AGV_Position_Raw_to_Rough()
{
	// AGV_Position_Go(Raw_Depart, 0);
	// AGV_TurnRight();
	// AGV_Position_Go(Rough, 0);
	// AGV_TurnRight();
	AGV_Position_Cruise(2, 2);
}

void AGV_Position_Rough_to_Temp()
{
	// AGV_Position_Go(Waypoint_RF, 0);
    // AGV_TurnRight();
	// AGV_Position_Go(Temp, 0);
	AGV_Position_Cruise(3, 1);
}

void AGV_Position_Temp_to_Raw()
{
	// AGV_Position_Go(Waypoint_RB, 0);
    // AGV_TurnRight();
	// AGV_Position_Go(Raw, 0);
	AGV_Position_Cruise(4, 0);
}

/**
 * @brief 回到原点
 * 
 */
void AGV_Position_GoBack()
{
	// AGV_Position_Go(Waypoint_RF, 0);
	// AGV_TurnRight();
	// AGV_Position_Go(Destination, 0);
	AGV_Position_Cruise(5, 3);
	AGV_Position_Go(150, 2250, 0);
}