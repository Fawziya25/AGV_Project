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
	Points_X[0] = 400;
	Points_Y[0] = 2000;
	// 2: QRCode
	Points_X[2] = 400;
	Points_Y[2] = 1200;
	Points_X[4] = 400;
	Points_Y[4] = 400;
	// 10: Rough
	Points_X[10] = 1200;
	Points_Y[10] = 2000;
	Points_X[12] = 1200;
	Points_Y[12] = 1200;
	// 14：Raw
	Points_X[14] = 1200;
	Points_Y[14] = 400;
	Points_X[20] = 2000;
	Points_Y[20] = 2000;
	// 22: Temp
	Points_X[22] = 2000;
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
	AGV_Position_Go(150, 150, 0);
}