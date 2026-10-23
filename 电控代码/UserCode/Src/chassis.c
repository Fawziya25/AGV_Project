#include "chassis.h"
#include "Emm_V5.h"
#include "delay.h"
#include "radar_task.h"
#include "stm32f4xx_hal.h"
#include "yawPIDcontroller.h"
#include "positionPIDcontroller.h"  /* AGV_Position_Stop：圆弧停车 */
#include "test.h"                /* Dbg_Service：圆弧等待期间喂调试服务 */
#include <stdint.h>
#include <stdlib.h>
#include <math.h>      /* fabsf：圆弧结束判据 */

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
	Points_X[0] = 400;
	Points_Y[0] = 2000;
	// 2: QRCode
	Points_X[2] = 400;
	Points_Y[2] = 1200;
	Points_X[4] = 400;
	Points_Y[4] = 400;
	// 10: Rough
	Points_X[10] = 1200;
	Points_Y[10] = 2100;
	Points_X[12] = 1200;
	Points_Y[12] = 1200;
	// 14：Raw
	Points_X[14] = 1200;
	Points_Y[14] = 300;
	Points_X[20] = 2000;
	Points_Y[20] = 2000;
	// 22: Temp
	Points_X[22] = 2100;
	Points_Y[22] = 1200;
	Points_X[24] = 2000;
	Points_Y[24] = 400;
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
	/* 0 距离直接跳过：0 脉冲的位置命令不会回"到位帧"，
	   Emm_V5_WaitArrive() 要白等到 5s 超时（轴对齐移动里每段都有一个 0 轴） */
	if(distance == 0U) return;
	float n = distance/235.62;  // 轮子周长
	uint32_t p = (uint32_t)(n*3200);
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
	if(distance == 0U) return;   /* 同上：0 距离不发命令，省掉一次 5s 超时 */
	float n = distance/235.62;  // 轮子周长
	uint32_t p = (uint32_t)(n*3200);

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
	if(dx == 0U && dy == 0U) { AGV_Position_Set(x, y); return; }   /* 已在目标点：只校准坐标，不发命令 */
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

/* ======================== 圆弧过弯（小半径 R=300mm） ========================
 * 为什么要它：原来车要**开到拐点坐标、停稳、原地转 90°**再走。现在在拐点之前
 * T = R·tan(θ/2) = R 处（θ=90°）就起弧，圆弧切过去 —— **不经过拐点、不原地转**。
 *
 * 坐标锚定（关键，必须做）：车既然不经过拐点，那"车现在在哪"就必须显式重锚，
 * 否则后面所有绝对坐标全错。所以每个圆角：
 *     直线开到切点 A  ->  AGV_Position_Set(A)      ★ 转弯前锚定
 *     圆弧走完        ->  AGV_Position_Set(B)      ★ 转弯后锚定（车不在拐点上！）
 *     同时 AGV_Direction 改成新朝向（四态模型）
 *
 * 角度用量化后的 cardinal（0:X+ 1:Y+ 2:X- 3:Y-，逆时针递增），和 AGV_Position_Go 的
 * 四态模型一致；这样也**不需要 atan2/sqrt**（本工程没链 libm，fabsf 是内建指令）。
 */
#define AGV_ARC_R_MM         300.0f   /* 弯半径(mm)，用户指定 300 */
#define AGV_ARC_V_MM_S       350.0f   /* 过弯时车体线速度(mm/s)：比直线(≈589)慢一点减小侧滑 */
#define AGV_ARC_T_MAX_RATIO  0.50f    /* 切点距 T 最多占相邻段长的一半（同段两个拐角的切点才不会重叠） */
#define AGV_ARC_T_MIN_MM     200.0f   /* 段太短时自动缩半径；缩到这个值还放不下就退化成到点再转 */

/* 角度归一到 (-180,180]（参数名不能叫 a：chassis.h 里有 #define a 100） */
static float Chassis_Norm180(float angle)
{
	while(angle > 180.0f)  angle -= 360.0f;
	while(angle <= -180.0f) angle += 360.0f;
	return angle;
}

/* cardinal 方向 -> 单位向量 */
static void Chassis_DirVec(uint8_t dir, int8_t *ux, int8_t *uy)
{
	*ux = 0; *uy = 0;
	switch(dir)
	{
		case 0: *ux =  1; break;   /* X+ */
		case 1: *uy =  1; break;   /* Y+ */
		case 2: *ux = -1; break;   /* X- */
		default:*uy = -1; break;   /* Y- */
	}
}

/**
 * @brief 速度模式画一段圆弧：四轮 = 平移分量 ± 自转分量，同步触发，
 *        用 HWT101 航向作为结束判据（速度模式没有到位帧），最后用航向PID补残差。
 * @param angleDeg 圆弧转角：>0 左转(逆时针，AGV_Direction+1)，<0 右转
 * @param radiusMm 本次圆弧的实际半径(mm)；段短时会小于 AGV_ARC_R_MM
 * @param yawRef   本段起步时的参考航向(°)：圆弧结束**锚定到 yawRef±angleDeg**，
 *                 而不是"当前航向+90"——这样直线段的跑偏会被一次补掉，
 *                 和 AGV_TurnLeft/Right 用 yawOffset 补偿的做法等价。
 * @note  自转图案沿用 AGV_SetOmega() 的"四轮 raw 同向、靠左右电机镜像安装"这一套
 *        （现场已验证过能原地转）；平移分量 = 四轮同向，与 GoStraight 一致。
 */
static void Chassis_ArcVelocity(float angleDeg, float radiusMm, float yawRef)
{
	const float v     = AGV_ARC_V_MM_S;
	float rpm_t = v * 60.0f / 235.62f;            /* 平移分量(rpm)：轮周长 235.62mm */
	float omega = v / radiusMm * 57.29578f;       /* 车体角速度(°/s)：ω = v / 实际半径 */
	float rpm_r = omega * 51.0f / 50.0f;          /* 自转分量(rpm)：比例取自 AGV_SetOmega */
	float sgn   = (angleDeg >= 0.0f) ? 1.0f : -1.0f;
	float target;
	uint32_t t0, tmo;
	uint8_t i;

	const uint8_t pd[4] = { Wheel1PositiveDir, Wheel2PositiveDir, Wheel3PositiveDir, Wheel4PositiveDir };

	for(i = 0U; i < 4U; i++)
	{
		/* 物理轮速(+ = 车前进)：平移 + 自转。自转方向：pd=1 的轮子正转，pd=0 的反转 */
		float u = rpm_t + sgn * ((pd[i] != 0U) ? rpm_r : -rpm_r);
		/* 命令位：物理前进 <=> 命令位 == PositiveDir（与 GoStraight 的映射一致） */
		uint8_t dir = (u >= 0.0f) ? pd[i] : (uint8_t)(pd[i] ^ 1U);
		uint16_t vel = (uint16_t)((u >= 0.0f ? u : -u) + 0.5f);
		Emm_V5_Vel_Control((uint8_t)(i + 1U), dir, vel, 0, 1);   /* 最后参数=1：多机同步标志 */
		delay_us(250);
	}
	Emm_V5_Synchronous_motion(0);
	delay_us(250);
	(void)Emm_V5_WaitRxFrame(EMM_V5_ACK_TMO_MS);

	/* 结束判据：航向转过 (|角度|-2°) 就停；剩 2° 交给航向PID精确补 */
	target = (angleDeg >= 0.0f) ? angleDeg : -angleDeg;
	t0  = HAL_GetTick();
	tmo = (uint32_t)(target / omega * 1000.0f * 2.5f) + 3000U;   /* 兜底超时：陀螺没数据时别死等 */
	while((fabsf(Chassis_Norm180(ZeroBias_GetYaw() - yawRef)) < (target - 2.0f)))
	{
		Dbg_Service();                                           /* 喂调试服务：刷屏 / C8 暂停 */
		if((HAL_GetTick() - t0) > tmo) break;
	}
	AGV_Position_Stop();
	/* ★ 朝向重新锚定：目标是"本段起步参考航向 ± 角度"，不是"当前航向 ± 角度"。
	   这样直线段累积的跑偏（以及停车的几度残差）一次补掉，和原转弯函数的补偿等价。 */
	AGV_SetYaw(Chassis_Norm180(yawRef + angleDeg));
}

/**
 * @brief 沿一条折线走：每个**内部**拐点用 R=AGV_ARC_R_MM 圆弧切过去
 * @param xs, ys 航点绝对坐标(mm)，依次走
 * @param n      航点数(>=1)
 * @param endDir 走完后车头朝向(0:X+ 1:Y+ 2:X- 3:Y-)
 * @note 内部拐点**不经过拐点坐标**：先到切点 A（锚定 A）→ 圆弧 → 出弧点 B（锚定 B）。
 *       最后一点精确到达（那是要停车/干活的位置），再原地转到 endDir。
 *       拐角 0°/180°/段太短放不下圆弧时，退化成"开到拐点 + 原地转"。
 */
static void Chassis_DrivePolyline(const uint16_t *xs, const uint16_t *ys, uint8_t n, uint8_t endDir)
{
	uint8_t i;
	if(n == 0U) return;

	for(i = 0U; i < n; i++)
	{
		uint16_t cx = xs[i];
		uint16_t cy = ys[i];
		int32_t  ddx, ddy;
		uint8_t  dirSeg, dirOut, turn;
		int32_t  lenIn, lenOut, ndx, ndy, Tz;
		int8_t   ux, uy, ox, oy;
		uint16_t ax, ay, bx, by;
		float    yawSeg;

		/* ---- 第一个点：**不做"起点摆正"** ----
		   AGV_Position_Go() 本身会按当前 AGV_Direction 把位移拆成"直行 + 横移"
		   （需要时走倒车/横移），所以起点不需要先原地转到本段方向。
		   少一次 ~1s 的原地转，也避免起点附近空间不够时那一转的误差。 */
		if(i == 0U)
		{
			AGV_Position_Go(cx, cy, 0);
			AGV_Position_Set(cx, cy);
			if((uint8_t)(i + 1U) >= n)
			{
				if(AGV_Direction != endDir) AGV_Turn(endDir);
				break;
			}
			continue;   /* 第一个点的拐角不圆角：从第二个点起正常走"段首摆正/圆弧" */
		}

		ddx = (int32_t)cx - (int32_t)AGV_Position_X;
		ddy = (int32_t)cy - (int32_t)AGV_Position_Y;

		/* 本段方向：按主导轴量化成 cardinal */
		dirSeg = (abs(ddx) >= abs(ddy)) ? ((ddx >= 0) ? 0U : 2U) : ((ddy >= 0) ? 1U : 3U);
		lenIn  = (abs(ddx) >= abs(ddy)) ? abs(ddx) : abs(ddy);

		/* 段首把车头摆正（起点之后才需要；正常运行时前一时刻已对齐） */
		if(AGV_Direction != dirSeg) AGV_Turn(dirSeg);

		/* 记录本段起步参考航向：圆弧结束时用它锚定朝向（直线跑偏不累积） */
		yawSeg = ZeroBias_GetYaw();

		/* ---- 最后一点：精确到达，再原地转到 endDir ---- */
		if((uint8_t)(i + 1U) >= n)
		{
			AGV_Position_Go(cx, cy, 0);
			AGV_Position_Set(cx, cy);
			if(AGV_Direction != endDir) AGV_Turn(endDir);
			break;
		}

		/* 下一点，决定这个拐点要不要圆角 */
		ndx    = (int32_t)xs[i + 1U] - (int32_t)cx;
		ndy    = (int32_t)ys[i + 1U] - (int32_t)cy;
		dirOut = (abs(ndx) >= abs(ndy)) ? ((ndx >= 0) ? 0U : 2U) : ((ndy >= 0) ? 1U : 3U);
		lenOut = (abs(ndx) >= abs(ndy)) ? abs(ndx) : abs(ndy);
		turn   = (uint8_t)((dirOut + 4U - dirSeg) % 4U);   /* 0直行 1左90 2掉头 3右90 */
		Tz     = (int32_t)(AGV_ARC_R_MM + 0.5f);

		if((turn == 0U) || (turn == 2U))
		{
			/* 直行 / 180°掉头 / 段太短放不下圆弧 -> 沿老路：开到拐点再原地转 */
			AGV_Position_Go(cx, cy, 0);
			AGV_Position_Set(cx, cy);
			if(turn != 0U) AGV_Turn(dirOut);
			continue;
		}

		/* ---- 圆角：A = 拐点 - T*u_in，B = 拐点 + T*u_out ----
		   半径上限 AGV_ARC_R_MM=300；若相邻段太短放不下，就缩到能放下的最大半径（仍走圆弧），
		   缩到 AGV_ARC_T_MIN_MM 还不行才退化成"到点再转"（上面那个分支）。 */
		{
			int32_t t_lim_in  = (int32_t)(AGV_ARC_T_MAX_RATIO * (float)lenIn);
			int32_t t_lim_out = (int32_t)(AGV_ARC_T_MAX_RATIO * (float)lenOut);
			int32_t t_min     = (int32_t)(AGV_ARC_T_MIN_MM + 0.5f);
			if(Tz > t_lim_in)  Tz = t_lim_in;
			if(Tz > t_lim_out) Tz = t_lim_out;
			if(Tz < t_min)
			{
				/* 段真的挤不下最小半径：走老路（到点 + 原地转） */
				AGV_Position_Go(cx, cy, 0);
				AGV_Position_Set(cx, cy);
				AGV_Turn(dirOut);
				continue;
			}
		}
		Chassis_DirVec(dirSeg, &ux, &uy);
		Chassis_DirVec(dirOut, &ox, &oy);
		ax = (uint16_t)((int32_t)cx - Tz * (int32_t)ux);
		ay = (uint16_t)((int32_t)cy - Tz * (int32_t)uy);
		bx = (uint16_t)((int32_t)cx + Tz * (int32_t)ox);
		by = (uint16_t)((int32_t)cy + Tz * (int32_t)oy);

		AGV_Position_Go(ax, ay, 0);                       /* 直线开到切点 A（不经过拐点） */
		AGV_Position_Set(ax, ay);                         /* ★ 转弯前重新锚定 */
		Chassis_ArcVelocity((turn == 1U) ? 90.0f : -90.0f, (float)Tz, yawSeg);  /* 圆弧 90° + 朝向锚定 */
		AGV_Position_Set(bx, by);                         /* ★ 转弯后重新锚定位置（车不在拐点上！） */
		AGV_Direction = dirOut;                           /* ★ 朝向状态同步（四态模型） */
		Yaw_MovingOffsetClear();                          /* ★ 朝向已锚定，清掉直行累计跑偏，别重复补偿 */
	}
}

/**
 * @brief 按一段雷达路径巡航（生产流程用：段末必须精确到达，那里要机械臂干活）
 * @param RouteID 段号(0~RADAR_LEG_COUNT-1)
 * @param tarDir  本段走完后要朝的绝对朝向(0:X+ 1:Y+ 2:X- 3:Y-)
 *
 * 段内拐点用 R=AGV_ARC_R_MM 圆弧切过去（不经过拐点、不原地转）；
 * 段末那个点**精确到达**，再原地转到 tarDir —— 因为生产流程里段与段之间隔着
 * 取料/放料动作，车本来就要停在段末。
 * 数据不可用（段号越界 / status!=0 / count==0 / 非法格号 / 未标定格）一律不乱开。
 */
void AGV_Position_Cruise(uint8_t RouteID, uint8_t tarDir)
{
	uint16_t xs[25];
	uint16_t ys[25];
	uint8_t  n = 0U;
	uint8_t  i;

	if(RouteID >= RADAR_LEG_COUNT) return;                 /* 防御：段号越界 */
	if(Radar_Path[RouteID].status != 0U) return;           /* 防御：该段没规划成功（1~6） */
	if(Radar_Path[RouteID].count == 0U) return;            /* 防御：空路径（雷达没连上/失败） */

	for(i = 0U; (i < Radar_Path[RouteID].count) && (n < 25U); i++)
	{
		uint8_t cell = Radar_Path[RouteID].points[i];
		if(cell > 24U) continue;
		if((Points_X[cell] == 0U) && (Points_Y[cell] == 0U)) continue;
		xs[n] = Points_X[cell];
		ys[n] = Points_Y[cell];
		n++;
	}
	Chassis_DrivePolyline(xs, ys, n, tarDir);
}

/**
 * @brief 把**若干段**雷达路径按给定顺序连成一条折线一次走完（只跑图、不干活的场合，如模式1测试）
 * @param legOrder 段号顺序数组，例如完整流程两遍：
 *                 `{0,1,2,3,4,2,3,5}` → 起点→2→14→10→22→14→10→22→回起点
 *                 （第一遍 0~4，第二遍 2/3/5；每段 start 正好等于上一段 goal）
 * @param legCount 数组长度（<= RADAR_LEG_COUNT 次使用；同一段可重复出现）
 * @param endDir   整条路线走完后车头朝向
 *
 * 段与段之间按"上一段 goal == 本段 start"校验连得上才继续；连不上就停在那里，
 * 避免像以前那样把不相邻的两段硬串成一条大斜线/大横移。
 * 第一段的 start 格会作为出发点自动插到最前面（取代 Depart 里那句 Go(Points[0])）。
 */
void AGV_Position_CruiseThroughRoute(const uint8_t *legOrder, uint8_t legCount, uint8_t endDir)
{
	uint16_t xs[25];
	uint16_t ys[25];
	uint8_t  n = 0U;
	uint8_t  k, i;
	uint8_t  prevGoal = 0xFFU;

	if(legOrder == NULL || legCount == 0U) return;

	for(k = 0U; k < legCount; k++)
	{
		uint8_t leg = legOrder[k];

		if(leg >= RADAR_LEG_COUNT) break;              /* 防御：段号越界 */
		if(Radar_Path[leg].status != 0U) break;        /* 防御：该段没规划成功 */
		if(Radar_Path[leg].count == 0U)  break;        /* 防御：空路径 */

		/* 连续性校验：上一段终点必须等于本段起点，否则不再往下开（路径不可信） */
		if((prevGoal != 0xFFU) && (Radar_Path[leg].start != prevGoal)) break;
		prevGoal = Radar_Path[leg].goal;

		/* 第一段的出发点格插到最前面 */
		if(k == 0U)
		{
			uint8_t sc = Radar_Path[leg].start;
			if((sc <= 24U) && !((Points_X[sc] == 0U) && (Points_Y[sc] == 0U)))
			{
				xs[n] = Points_X[sc];
				ys[n] = Points_Y[sc];
				n++;
			}
		}

		for(i = 0U; (i < Radar_Path[leg].count) && (n < 25U); i++)
		{
			uint8_t cell = Radar_Path[leg].points[i];
			if(cell > 24U) continue;
			if((Points_X[cell] == 0U) && (Points_Y[cell] == 0U)) continue;
			xs[n] = Points_X[cell];
			ys[n] = Points_Y[cell];
			n++;
		}
	}
	Chassis_DrivePolyline(xs, ys, n, endDir);
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
	AGV_Position_Cruise(1, 0);   /* tarDir 恢复原值（方向控制逻辑不动） */
}

void AGV_Position_Raw_to_Rough()
{
	// AGV_Position_Go(Raw_Depart, 0);
	// AGV_TurnRight();
	// AGV_Position_Go(Rough, 0);
	// AGV_TurnRight();
	AGV_Position_Cruise(2, 2);   /* tarDir 恢复原值 */
}

void AGV_Position_Rough_to_Temp()
{
	// AGV_Position_Go(Waypoint_RF, 0);
    // AGV_TurnRight();
	// AGV_Position_Go(Temp, 0);
	AGV_Position_Cruise(3, 1);   /* tarDir 恢复原值 */
}

void AGV_Position_Temp_to_Raw()
{
	// AGV_Position_Go(Waypoint_RB, 0);
    // AGV_TurnRight();
	// AGV_Position_Go(Raw, 0);
	AGV_Position_Cruise(4, 0);   /* tarDir 恢复原值 */
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