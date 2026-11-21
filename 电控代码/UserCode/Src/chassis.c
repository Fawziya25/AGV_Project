#include "chassis.h"
#include "Emm_V5.h"
#include "delay.h"
#include "radar_config.h"   /* RADAR_ZONE_DEFAULT：上电先按哪个启停区摆 */
#include "radar_task.h"
#include "stm32f4xx_hal.h"
#include "yawPIDcontroller.h"
#include <stdint.h>
#include <stdlib.h>

/* 小车当前坐标(mm)，上电记作原点；每次 AGV_Position_Go() 走完自动锚定到目标点 */
uint16_t AGV_Position_X;
uint16_t AGV_Position_Y;

/* 当前启停区 1/2：AGV_Position_ApplyZone() 更新，AGV_Position_Depart() 用它选首点 */
static uint8_t AGV_Zone = 2U;

uint16_t Points_X[25];
uint16_t Points_Y[25];

/**
 * @brief 按启停区设定小车初始坐标与车头朝向
 * @param zone 启停区 1/2；其它值一律按区2（与 RADAR_ZONE_DEFAULT 一致）
 * @note  只动"车在哪、朝哪"，雷达那边的选区/位姿由 Radar_SetZone() 管；
 *        两区各自的坐标/朝向宏在 chassis.h 的 AGV_ZONEn_ORIGIN_*。
 */
void AGV_Position_ApplyZone(uint8_t zone)
{
	if(zone == 1U)
	{
		AGV_Zone       = 1U;
		AGV_Position_X = AGV_ZONE1_ORIGIN_X;
		AGV_Position_Y = AGV_ZONE1_ORIGIN_Y;
		AGV_Direction  = (uint8_t)(AGV_ZONE1_ORIGIN_DIR & 3U);
	}
	else
	{
		AGV_Zone       = 2U;
		AGV_Position_X = AGV_ZONE2_ORIGIN_X;
		AGV_Position_Y = AGV_ZONE2_ORIGIN_Y;
		AGV_Direction  = (uint8_t)(AGV_ZONE2_ORIGIN_DIR & 3U);
	}
}

void AGV_Position_Init(void)
{
	// 所有拐点坐标(mm)
	Points_X[0] = 350;
	Points_Y[0] = 2060;
	// 2: QRCode
	Points_X[2] = 350;
	Points_Y[2] = 1200;
	Points_X[4] = 350;
	Points_Y[4] = 350;
	// 10: Rough
	Points_X[10] = 1200;
	Points_Y[10] = 2060;
	Points_X[12] = 1200;
	Points_Y[12] = 1200;
	// 14：Raw
	Points_X[14] = 1200;
	Points_Y[14] = 350;
	Points_X[20] = 2060;
	Points_Y[20] = 2070;
	// 22: Temp
	Points_X[22] = 2060;
	Points_Y[22] = 1200;
	Points_X[24] = 2060;
	Points_Y[24] = 350;

	/* 小车初始坐标/朝向跟着启停区走，两区各有一套（chassis.h 的 AGV_ZONEn_ORIGIN_*）。
	   这里先按默认区摆好；菜单里选了别的区会再由 AGV_Position_ApplyZone() 覆盖。 */
	AGV_Position_ApplyZone((uint8_t)RADAR_ZONE_DEFAULT);
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
	float n = distance/235.62;  // 轮子周长
	uint32_t p = (uint32_t)(n*3200);

	/* 0 距离不下发：原来无条件发一条 Pos_Control(clk=0) 再 WaitArrive，
	   电机不一定回"到位"帧，白等 EMM_V5_ARRIVE_TMO_MS(5000ms) 才超时返回 */
	if(distance == 0U) return;

	dir^=1;
	Yaw_MovingOffsetStart();	/* 平移本不该改航向：期间读数变化就是航向漂移，下次转弯消费 */
	Emm_V5_ClearRxFrame();						// 清历史回帧：后面等的是本次动作新到的到位帧
	Emm_V5_Pos_Control(1, Wheel1PositiveDir^dir, vel, AGV_ACC_POS, p, 2, 1); // 多机同步标志位置1
		HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_Pos_Control(2, Wheel2PositiveDir^dir, vel, AGV_ACC_POS, p, 2, 1); // 多机同步标志位置1
		HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_Pos_Control(3, Wheel3PositiveDir^dir, vel, AGV_ACC_POS, p, 2, 1); // 多机同步标志位置1
		HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_Pos_Control(4, Wheel4PositiveDir^dir, vel, AGV_ACC_POS, p, 2, 1); // 多机同步标志位置1
		HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_Synchronous_motion(0); 								 // 广播地址0触发
	HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	/* 只认四个轮子(地址1~4)的到位帧，任意一个报到就算到位；地址5/6(机械臂)的
	   0x9F 不会被误当成底盘到位。超时兜底不卡死。 */
	Emm_V5_WaitArriveMask(0x0FU, EMM_V5_ARRIVE_TMO_MS);

	HAL_Delay(200);

	Yaw_MovingOffsetEnd();	/* 本次移动期间累计的航向漂移，攒到下一次转弯一起扣 */
}

/**
 * @brief 平移移动（横移）
 * 
 * @param distance 距离(mm)
 * @param vel 速度(RPM)，**上限是宏 Vel_Translate(80)**，传大了会被压下来
 * @param dir 方向
 * 0：向右移动
 * 1：向左移动
 * @note  横移全靠 45° 辊子侧滑传力，效率和重复性都远低于直行：同一档速度下它先打滑，
 *        而且滑多少每次都不太一样（表现为距离偏短 + 车体斜滑）。所以横移有**独立的速度上限**
 *        Vel_Translate(80RPM) 和**独立的加减速** AGV_ACC_TRANSLATE(50)；
 *        直行/倒车仍用 Vel(350RPM) / AGV_ACC_POS(150)。要改横移快慢/软硬只改那两个宏。
 */
void AGV_Position_Translate(uint32_t distance, uint16_t vel, uint8_t dir)
{
	float n = distance/235.62;  // 轮子周长
	uint32_t p = (uint32_t)(n*3200);

	/* 0 距离不下发：见 AGV_Position_GoStraight 的说明 */
	if(distance == 0U) return;

	/* 横移速度上限（只作用于横移，直行不受影响）：现有调用点全传 Vel(350)，
	   实测那档速度横移会飘，统一压到 Vel_Translate(80)；调用方想更慢可以传更小的值，
	   以后新加的调用点即使传 Vel 也不会再退回高速横移。 */
	if(vel > Vel_Translate) vel = Vel_Translate;

	Yaw_MovingOffsetStart();	/* 平移本不该改航向：期间读数变化就是航向漂移，下次转弯消费 */
	Emm_V5_ClearRxFrame();						// 清历史回帧：后面等的是本次动作新到的到位帧
	Emm_V5_Pos_Control(1, Wheel1PositiveDir^dir, vel, AGV_ACC_TRANSLATE, p, 2, 1); // 多机同步标志位置1
		HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_Pos_Control(4, Wheel2PositiveDir^dir, vel, AGV_ACC_TRANSLATE, p, 2, 1); // 多机同步标志位置1
		HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_Pos_Control(2, Wheel3PositiveDir^dir^1, vel, AGV_ACC_TRANSLATE, p, 2, 1); // 多机同步标志位置1
		HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_Pos_Control(3, Wheel4PositiveDir^dir^1, vel, AGV_ACC_TRANSLATE, p, 2, 1); // 多机同步标志位置1
		HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	Emm_V5_Synchronous_motion(0); 								 // 广播地址0触发
	HAL_Delay(1);																 // 每条命令后面延时10毫秒，防止粘包，CAN通讯速率调到1MHz，可以缩短至几十微秒即可

	/* 只认四个轮子(地址1~4)的到位帧，任意一个报到就算到位；地址5/6(机械臂)的
	   0x9F 不会被误当成底盘到位。超时兜底不卡死。 */
	Emm_V5_WaitArriveMask(0x0FU, EMM_V5_ARRIVE_TMO_MS);

	HAL_Delay(200);

	Yaw_MovingOffsetEnd();	/* 本次移动期间累计的航向漂移，攒到下一次转弯一起扣 */
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
	/* 航向漂移改成在 GoStraight/Translate 内部逐次测量（这样工位前那些单独调用的
	   短移也覆盖到了）；这里不再整段包一层，否则一趟 Go 会被累加两次 */
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
	// 重新锚定小车位置
	AGV_Position_Set(x, y);
}

void AGV_Position_Rotate(uint8_t dir)
{
	if(dir == 0)
	{
		Emm_V5_Vel_Control(1, 0, 40, AGV_ACC_VEL, 1);
		HAL_Delay(10);
		Emm_V5_Vel_Control(2, 0, 40, AGV_ACC_VEL, 1);
		HAL_Delay(10);
		Emm_V5_Vel_Control(3, 0, 40, AGV_ACC_VEL, 1);
		HAL_Delay(10);
		Emm_V5_Vel_Control(4, 0, 40, AGV_ACC_VEL, 1);
		HAL_Delay(10);
	}
	else if(dir == 1)
	{
		Emm_V5_Vel_Control(1, 1, 40, AGV_ACC_VEL, 1);
		delay_us(250);
		Emm_V5_Vel_Control(2, 1, 40, AGV_ACC_VEL, 1);
		delay_us(250);
		Emm_V5_Vel_Control(3, 1, 40, AGV_ACC_VEL, 1);
		delay_us(250);
		Emm_V5_Vel_Control(4, 1, 40, AGV_ACC_VEL, 1);
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

/* ==================== 每段"默认路径"与选择规则 ====================
 * 默认路径 = 无障碍时的常规走法（拐点 + 终点格，不含起点；起点按启停区自动取）。
 * 选择规则（用户要求）：
 *   1) 该段雷达规划失败              → 用默认路径；
 *   2) 规划成功 & 默认路径可用        → 用默认路径；
 *   3) 规划成功 & 默认路径不可用      → 用规划路径。
 * "默认可用" = 起点/拐点/终点，以及相邻两点之间同一行或同一列经过的格子，
 *              在最近一次方格扫描（Radar_GridIsObstacle）里都空闲。
 * 段号/终点与 radar_config.h 的 LAUNCH_ROUTE_GOALS 一致；改路线时这里要一起改。
 * ==================================================================== */

#define AGV_DEFAULT_MAX_POINTS  8U

/* 各段默认拐点（不含起点）；下标 = 段号 0..5 */
static const uint8_t AGV_DefLeg0[]     = { 2U };             /* 起点 → 2  (QRCode) */
static const uint8_t AGV_DefLeg1[]     = { 4U, 14U };        /* 2 → 4 → 14 (Raw) */
static const uint8_t AGV_DefLeg2[]     = { 10U };            /* 14 → 10 (Rough) */
static const uint8_t AGV_DefLeg3[]     = { 20U, 22U };       /* 10 → 20 → 22 (Temp) */
static const uint8_t AGV_DefLeg4[]     = { 24U, 14U };       /* 22 → 24 → 14 (Raw) */
static const uint8_t AGV_DefLeg5_Z1[]  = { 24U, 4U };        /* 22 → 24 → 4 (区1 停车格) */
static const uint8_t AGV_DefLeg5_Z2[]  = { 20U, 0U };        /* 22 → 20 → 0 (区2 停车格) */

/* 各段终点格（= LAUNCH_ROUTE_GOALS），第 1~4 段的起点 = 上一段终点 */
static const uint8_t AGV_LegGoals[5] = LAUNCH_ROUTE_GOALS;

/* 调试：每段实际用的是哪条路 0=规划 1=默认 2=无路径（debugger 直接 watch） */
volatile uint8_t AGV_LegSource[RADAR_LEG_COUNT];

/** 方格 → (行,列)：手册 §9 的编号规则（奇数行从右往左） */
static void AGV_CellRowCol(uint8_t cell, uint8_t *row, uint8_t *col)
{
    uint8_t r = (uint8_t)(cell / 5U);
    uint8_t k = (uint8_t)(cell % 5U);

    *row = r;
    *col = (uint8_t)(((r & 1U) != 0U) ? (4U - k) : k);
}

/** (行,列) → 方格 */
static uint8_t AGV_CellOf(uint8_t row, uint8_t col)
{
    if((row & 1U) != 0U) col = (uint8_t)(4U - col);
    return (uint8_t)(row * 5U + col);
}

/** 本段起点格：第 0 段按启停区取停车格；第 1~4 段 = 上一段终点；第 5 段 = RADAR_RETURN_FROM_GRID */
static uint8_t AGV_LegStartCell(uint8_t leg)
{
    if(leg == 0U) return (uint8_t)((AGV_Zone == 1U) ? AGV_ZONE1_DEPART_GRID : AGV_ZONE2_DEPART_GRID);
    if(leg == 5U) return (uint8_t)RADAR_RETURN_FROM_GRID;
    return AGV_LegGoals[leg - 1U];
}

/** 两格之间（同一行或同一列）经过的格子是否都空闲；不同行列（斜跨）保守判为不可用 */
static uint8_t AGV_DefSpanFree(uint8_t from, uint8_t to)
{
    uint8_t r0, c0, r1, c1, i, lo, hi;

    AGV_CellRowCol(from, &r0, &c0);
    AGV_CellRowCol(to,   &r1, &c1);

    if((r0 == r1) && (c0 == c1)) return (Radar_GridIsObstacle(to) ? 0U : 1U);

    if(r0 == r1)                                  /* 同一行：列方向遍历 */
    {
        lo = (c0 < c1) ? c0 : c1;
        hi = (c0 < c1) ? c1 : c0;
        for(i = lo; i <= hi; i++) { if(Radar_GridIsObstacle(AGV_CellOf(r0, i))) return 0U; }
        return 1U;
    }
    if(c0 == c1)                                  /* 同一列：行方向遍历 */
    {
        lo = (r0 < r1) ? r0 : r1;
        hi = (r0 < r1) ? r1 : r0;
        for(i = lo; i <= hi; i++) { if(Radar_GridIsObstacle(AGV_CellOf(i, c0))) return 0U; }
        return 1U;
    }
    return 0U;                                    /* 斜跨：默认路径不这么走，判不可用交给规划 */
}

/** 取一段的默认路径点（不含起点）；返回点数 */
static uint8_t AGV_DefaultLegPoints(uint8_t leg, uint8_t *out, uint8_t maxOut)
{
    const uint8_t *src = 0;
    uint8_t n = 0U, i;

    switch(leg)
    {
        case 0U: src = AGV_DefLeg0; n = (uint8_t)sizeof(AGV_DefLeg0); break;
        case 1U: src = AGV_DefLeg1; n = (uint8_t)sizeof(AGV_DefLeg1); break;
        case 2U: src = AGV_DefLeg2; n = (uint8_t)sizeof(AGV_DefLeg2); break;
        case 3U: src = AGV_DefLeg3; n = (uint8_t)sizeof(AGV_DefLeg3); break;
        case 4U: src = AGV_DefLeg4; n = (uint8_t)sizeof(AGV_DefLeg4); break;
        case 5U:
            if(AGV_Zone == 1U) { src = AGV_DefLeg5_Z1; n = (uint8_t)sizeof(AGV_DefLeg5_Z1); }
            else               { src = AGV_DefLeg5_Z2; n = (uint8_t)sizeof(AGV_DefLeg5_Z2); }
            break;
        default: return 0U;
    }
    if(n > maxOut) n = maxOut;
    for(i = 0U; i < n; i++) out[i] = src[i];
    return n;
}

/** 默认路径是否可用：起点 + 每个拐点/终点 + 相邻两点之间的格子都无遮挡 */
static uint8_t AGV_DefaultLegUsable(uint8_t leg, const uint8_t *pts, uint8_t n)
{
    uint8_t prev = AGV_LegStartCell(leg);
    uint8_t i;

    if(n == 0U) return 0U;
    if(Radar_GridIsObstacle(prev)) return 0U;

    for(i = 0U; i < n; i++)
    {
        if(pts[i] > 24U) return 0U;
        if(AGV_DefSpanFree(prev, pts[i]) == 0U) return 0U;
        prev = pts[i];
    }
    return 1U;
}

/**
 * @brief 选本段实际要走的路径点（默认优先）
 * @param leg    段号 0..RADAR_LEG_COUNT-1
 * @param out    输出：方格编号（行驶顺序，不含起点）
 * @param maxOut out 容量
 * @return 点数；0 = 没有可用路径（该段应该空转/停下）
 * @note 规则见文件头注释；同时把结果记到 AGV_LegSource[leg] 方便调试。
 */
static uint8_t AGV_SelectLegPoints(uint8_t leg, uint8_t *out, uint8_t maxOut)
{
    uint8_t def[AGV_DEFAULT_MAX_POINTS];
    uint8_t dn, pn, i;

    if(leg >= RADAR_LEG_COUNT) return 0U;

    dn = AGV_DefaultLegPoints(leg, def, (uint8_t)sizeof(def));
    pn = (Radar_Path[leg].count > maxOut) ? maxOut : Radar_Path[leg].count;

    /* 默认路径可用 → 优先默认（规划成功与否都一样） */
    if((dn > 0U) && (AGV_DefaultLegUsable(leg, def, dn) != 0U))
    {
        for(i = 0U; i < dn; i++) out[i] = def[i];
        AGV_LegSource[leg] = 1U;
        return dn;
    }

    /* 默认不可用：规划成功就用规划路径 */
    if((Radar_Path[leg].status == 0U) && (pn > 0U))
    {
        for(i = 0U; i < pn; i++) out[i] = Radar_Path[leg].points[i];
        AGV_LegSource[leg] = 0U;
        return pn;
    }

    /* 规划失败/无路径：退回默认路径（哪怕默认有遮挡，也比不动强） */
    if(dn > 0U)
    {
        for(i = 0U; i < dn; i++) out[i] = def[i];
        AGV_LegSource[leg] = 1U;
        return dn;
    }

    AGV_LegSource[leg] = 2U;
    return 0U;
}

/**
 * @brief 按方格序列走（不含段末转身）：每个点前按主导轴决定要不要先转 90°
 * @param cells  方格编号数组（行驶顺序）
 * @param n      点数
 * @param tarDir 本段走完后的最终朝向，用来在"两个候选方向"里选更接近的那个
 * @note  这段逻辑与原来 AGV_Position_Cruise() 里的循环完全一致，只是抽出来给
 *        正向/反向两个入口共用。
 */
static void AGV_FollowCells(const uint8_t *cells, uint8_t n, uint8_t tarDir, uint8_t faceMotion)
{
    uint8_t i;

    for(i = 0U; i < n; i++)
    {
        uint8_t  cell = cells[i];
        uint16_t tarX, tarY;

        if(cell > 24U) continue;
        tarX = Points_X[cell];
        tarY = Points_Y[cell];

        if(faceMotion != 0U)
        {
            /* faceMotion=1：按"这一段的运动方向"决定车头（反向路线用，保证始终正向行驶）。
               正常正向巡航仍走下面的原逻辑，行为不变。 */
            uint8_t want;
            if(abs(tarX - AGV_Position_X) >= abs(tarY - AGV_Position_Y))
            {
                want = ((int32_t)tarX >= (int32_t)AGV_Position_X) ? 0U : 2U;
            }
            else
            {
                want = ((int32_t)tarY >= (int32_t)AGV_Position_Y) ? 1U : 3U;
            }
            if(AGV_Direction != want) AGV_Turn(want);
        }
        else if(abs(tarX - AGV_Position_X) >= abs(tarY - AGV_Position_Y))
        {
            if(AGV_Direction == 1 || AGV_Direction == 3)
            {
                /* 车头朝 ±Y、本段要改走 X：在 X+/X- 里选离最终朝向 tarDir 更近的那个。 */
                if(abs((int)tarDir - 2) >= abs((int)tarDir - 0))AGV_Turn(0);
                else AGV_Turn(2);
            }
        }
        else
        {
            if(AGV_Direction == 0 || AGV_Direction == 2)
            {
                /* 同理：车头朝 ±X、要改走 Y 时，在 Y+/Y- 里选离 tarDir 更近的。 */
                if(abs((int)tarDir - 1) >= abs((int)tarDir - 3))AGV_Turn(3);
                else AGV_Turn(1);
            }
        }
        AGV_Position_Go(tarX, tarY, 0);
    }
}

void AGV_Position_Cruise(uint8_t RouteID, uint8_t tarDir)
{
    uint8_t pts[RADAR_PATH_MAX_POINTS];
    uint8_t n;

    /* tarDir = 该段走完后的绝对朝向：0:X+ 1:Y+ 2:X- 3:Y-。
       先归一化（4 当 0）：旧写法传 4 会一路传到 AGV_Turn()，而它只认 0~3，
       非法值在那里会落到"没有任何分支命中" → 该转的时候不转。 */
    tarDir &= 3U;

    if(RouteID >= RADAR_LEG_COUNT) return;

    /* ★ 默认路径优先：规划失败 → 默认；规划成功且默认可用 → 默认；默认不可用 → 规划。
       详见上面 AGV_SelectLegPoints() 的规则说明。 */
    n = AGV_SelectLegPoints(RouteID, pts, (uint8_t)sizeof(pts));
    if(n == 0U) return;

    AGV_FollowCells(pts, n, tarDir, 0U);      /* 按点走（转/直行逻辑原样） */
    if(RouteID != 5) AGV_Turn(tarDir);        /* 段末精确到达后原地转到目标朝向 */
}

/**
 * @brief 暂存区 → 粗加工区（第 4 段的反向），段末朝向 X-(2)
 * @note  函数名沿用需求里给的名字；实际走向 = 22(暂存) → 20 → 10(粗加工)，
 *        与 AGV_Position_Cruise(3, 1) 的路径**完全反向**。
 *        路径点同样先走 AGV_SelectLegPoints(3)（默认优先 / 规划兜底）再反向遍历，
 *        末尾补上本段起点格（第 4 段起点 = 10），所以终点就是粗加工区。
 */
void AGV_Position_Cruise_Jingjiagong_to_Temp(void)
{
    uint8_t pts[RADAR_PATH_MAX_POINTS];
    uint8_t rev[RADAR_PATH_MAX_POINTS];
    uint8_t n, i, rn = 0U;

    n = AGV_SelectLegPoints(3U, pts, (uint8_t)sizeof(pts));
    if(n == 0U) return;                                   /* 没选到路径：不动 */

    for(i = n; i > 0U; i--)                               /* 拐点/终点反向 */
    {
        if(rn < (uint8_t)sizeof(rev)) rev[rn++] = pts[i - 1U];
    }
    if(rn < (uint8_t)sizeof(rev))                         /* 最后一个点 = 本段起点格(10) */
    {
        rev[rn++] = AGV_LegStartCell(3U);
    }

    /* 第一个反向后点就是车现在所在的格（第 4 段终点 22）时跳过它：
       否则会先"原地转一次再走 0 距离"，白转两次 90°。 */
    if((rn > 0U) && (rev[0] <= 24U)
       && (Points_X[rev[0]] == AGV_Position_X) && (Points_Y[rev[0]] == AGV_Position_Y))
    {
        for(i = 0U; (uint8_t)(i + 1U) < rn; i++) rev[i] = rev[i + 1U];
        rn--;
    }
    if(rn == 0U) { AGV_Turn(2U); return; }

    AGV_FollowCells(rev, rn, 2U, 1U);                     /* faceMotion=1：按运动方向定车头，正向行驶 */
    AGV_Turn(2U);                                         /* 目标航向 X- */
}

#if Run_With_Radar

/**
 * @brief 出发：先对齐到本启停区的停车位格，再交给雷达规划的第一段
 * @note  首点按启停区取（区1 = Points[4] 左下角、区2 = Points[0] 左上角，见
 *        chassis.h 的 AGV_ZONEn_DEPART_GRID）；它必须和 radar_config.h 里该区的
 *        RADAR_ZONEn_START_GRID 是同一格，否则"先开到出发点"和"规划从哪格出发"会打架。
 */
void AGV_Position_Depart()
{
	uint8_t g = (AGV_Zone == 1U) ? (uint8_t)AGV_ZONE1_DEPART_GRID
	                             : (uint8_t)AGV_ZONE2_DEPART_GRID;

	AGV_Position_Go(Points_X[g], Points_Y[g], 0);

	// AGV_Position_Cruise(0, 3);

	if(AGV_Zone == 1U)
	{
		AGV_TurnLeft();
		AGV_Position_Go(Points_X[14], Points_Y[14], 0);
	}
	else if(AGV_Zone == 2U)
	{
		AGV_Position_Go(Points_X[4], Points_Y[4], 0);
		AGV_TurnLeft();
		AGV_Position_Go(Points_X[14], Points_Y[14], 0);
	}
}

void AGV_Position_Qcode_to_Temp()
{
	// AGV_Position_Cruise(1, 0);
	// AGV_Position_Translate(Raw_Approaching_L, Vel, 0);
	// if(Raw_Offset_L > 0)
	// {
	// 	AGV_Position_GoStraight(Raw_Offset_L, Vel, 1);
	// }
	// else if(Raw_Offset_L < 0)
	// {
	// 	AGV_Position_GoStraight(-Raw_Offset_L, Vel, 0);
	// }

	AGV_Position_Cruise(2, 2);
	AGV_Position_Translate(Temp_Approaching_L, Vel, 0);
}

void AGV_Position_Temp_to_Jingjiagong()
{
	// AGV_Position_Translate(Raw_Approaching_L, Vel, 1);
	// if(Raw_Offset_L > 0)
	// {
	// 	AGV_Position_GoStraight(Raw_Offset_L, Vel, 0);
	// }
	// else if(Raw_Offset_L < 0)
	// {
	// 	AGV_Position_GoStraight(-Raw_Offset_L, Vel, 1);
	// }
	// AGV_Position_Cruise(2, 2);
	// AGV_Position_Translate(Rough_Approaching_L, Vel, 0);

	AGV_Position_Translate(Temp_Approaching_L, Vel, 1);
	AGV_Position_Cruise(3, 1);
	AGV_Position_Translate(Jingjiagong_Approaching_L, Vel, 0);
}

/* 22(精加工/暂存区) -> 14(成品区)：函数体本来就是 22->14，
   原名写的是 _to_Temp，与 task.c 的 _to_Chengpin 调用不一致，改名对齐。 */
void AGV_Position_Jingjiagong_to_Temp()
{
	// AGV_Position_Translate(Rough_Approaching_L, Vel, 1);
	// AGV_Position_Cruise(3, 1);
	// AGV_Position_Translate(Temp_Approaching_L, Vel, 0);

    AGV_Position_Translate(Jingjiagong_Approaching_L, Vel, 1);
	AGV_Position_Cruise_Jingjiagong_to_Temp();
	AGV_Position_Translate(Temp_Approaching_L, Vel, 0);
}

/**
 * @brief 回到原点：走完雷达第 6 段后，再收回到**本启停区**的停车点
 * @note  末点原来写死 (150, 2250)（区2 的停车位），区1 会被拉回区2 的位置；
 *        现按 AGV_Zone 取本区停车坐标（chassis.h 的 AGV_ZONEn_ORIGIN_*）。
 */
void AGV_Position_GoBack()
{
	uint16_t ox = (AGV_Zone == 1U) ? (uint16_t)AGV_ZONE1_ORIGIN_X : (uint16_t)AGV_ZONE2_ORIGIN_X;
	uint16_t oy = (AGV_Zone == 1U) ? (uint16_t)AGV_ZONE1_ORIGIN_Y : (uint16_t)AGV_ZONE2_ORIGIN_Y;

	AGV_Position_Translate(Jingjiagong_Approaching_L, Vel, 1);
	AGV_Position_Cruise(5, 3);
	AGV_Position_Go(ox, oy, 0);
}

#else

/**
 * @brief 出发：先对齐到本启停区的停车位格，再交给雷达规划的第一段
 * @note  首点按启停区取（区1 = Points[4] 左下角、区2 = Points[0] 左上角，见
 *        chassis.h 的 AGV_ZONEn_DEPART_GRID）；它必须和 radar_config.h 里该区的
 *        RADAR_ZONEn_START_GRID 是同一格，否则"先开到出发点"和"规划从哪格出发"会打架。
 */
void AGV_Position_Depart()
{
	uint8_t g = (AGV_Zone == 1U) ? (uint8_t)AGV_ZONE1_DEPART_GRID
	                             : (uint8_t)AGV_ZONE2_DEPART_GRID;

	AGV_Position_Go(Points_X[g], Points_Y[g], 0);

	if(AGV_Zone == 1U)
	{
		AGV_TurnLeft();
		AGV_Position_Go(Points_X[14], Points_Y[14], 0);
	}
	else if(AGV_Zone == 2U)
	{
		AGV_Position_Go(Points_X[4], Points_Y[4], 0);
		AGV_TurnLeft();
		AGV_Position_Go(Points_X[14], Points_Y[14], 0);
	}
}

void AGV_Position_Qcode_to_Temp()
{
	AGV_Position_Go(Points_X[24], Points_Y[24], 0);
	AGV_TurnLeft();
	AGV_Position_Go(Points_X[20], Points_Y[20], 0);
	AGV_TurnLeft();
	AGV_Position_Go(Points_X[10], Points_Y[10], 0);
	AGV_Position_Translate(Temp_Approaching_L, Vel, 0);
}

void AGV_Position_Temp_to_Jingjiagong()
{
	AGV_Position_Translate(Temp_Approaching_L, Vel, 1);
	AGV_Position_Go(Points_X[20], Points_Y[20], 0);
	AGV_TurnRight();
	AGV_Position_Go(Points_X[22], Points_Y[22], 0);
	AGV_Position_Translate(Jingjiagong_Approaching_L, Vel, 0);
}

void AGV_Position_Jingjiagong_to_Temp()
{
	AGV_Position_Translate(Jingjiagong_Approaching_L, Vel, 1);
	AGV_Position_Go(Points_X[20], Points_Y[20], 0);
    AGV_TurnLeft();
	AGV_Position_Go(Points_X[10], Points_Y[10], 0);
	AGV_Position_Translate(Temp_Approaching_L, Vel, 0);
}

/**
 * @brief 回到原点：走完雷达第 6 段后，再收回到**本启停区**的停车点
 * @note  末点原来写死 (150, 2250)（区2 的停车位），区1 会被拉回区2 的位置；
 *        现按 AGV_Zone 取本区停车坐标（chassis.h 的 AGV_ZONEn_ORIGIN_*）。
 */
void AGV_Position_GoBack()
{
	uint16_t ox = (AGV_Zone == 1U) ? (uint16_t)AGV_ZONE1_ORIGIN_X : (uint16_t)AGV_ZONE2_ORIGIN_X;
	uint16_t oy = (AGV_Zone == 1U) ? (uint16_t)AGV_ZONE1_ORIGIN_Y : (uint16_t)AGV_ZONE2_ORIGIN_Y;

	if(AGV_Zone == 1U)
	{
		AGV_Position_Go(Points_X[24], Points_Y[24], 0);
	}
	else
	{
		AGV_Position_Go(Points_X[20], Points_Y[20], 0);
	}
	AGV_TurnRight();
	if(AGV_Zone == 1U)
	{
		AGV_Position_Go(ox, oy, 0);
	}
	else
	{
		AGV_Position_Go(ox-40, oy-60, 0);
	}
}

#endif