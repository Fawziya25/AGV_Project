#include "chassis.h"
#include "Emm_V5.h"
#include "delay.h"
#include "stm32f4xx_hal.h"
#include "yawPIDcontroller.h"

uint16_t AGV_Position_X = Origin_X;
uint16_t AGV_Position_Y = Origin_Y;

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
	float n = distance/251.33;
	uint32_t p = (uint32_t)(n*3200);
	dir^=1;
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

	while(can.rxData[0] != 0xFD || can.rxData[1] != 0x9F);
	can.rxFrameFlag = false;

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
	float n = distance/251.33;
	uint32_t p = (uint32_t)(n*3200);

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

	while(can.rxData[0] != 0xFD || can.rxData[1] != 0x9F);
	can.rxFrameFlag = false;

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
void AGV_Position_Go(uint16_t x, uint16_t y, uint16_t vel, uint8_t priDir)
{
	/* 差值必须用有符号数承接：用 uint16_t 会有两个致命问题——
	   1) (dx >= 0) 对无符号数恒真，x_flag/y_flag 永远是 1，下面的 ^1 反向分支全成死代码；
	   2) 目标坐标小于当前坐标时，负差值回绕成 65536-|Δ|，小车会按几十米去跑。
	   int32_t 是为了容纳 uint16_t 两端相减的最大跨度 ±65535，int16_t 会溢出。 */
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
			AGV_Position_GoStraight(dx, vel, x_flag);
			AGV_Position_Translate(dy, vel, y_flag);
		}
		else if(AGV_Direction == 1)
		{
			AGV_Position_Translate(dx, vel, x_flag^1);
			AGV_Position_GoStraight(dy, vel, y_flag);
		}
		else if (AGV_Direction == 2)
		{
			AGV_Position_GoStraight(dx, vel, x_flag^1);
			AGV_Position_Translate(dy, vel, y_flag^1);
		}
		else if (AGV_Direction == 3)
		{
			AGV_Position_Translate(dx, vel, x_flag);
			AGV_Position_GoStraight(dy, vel, y_flag^1);
		}
	}
	// Y方向先行
	else if(priDir == 1)
	{
		if(AGV_Direction == 0)
		{
			AGV_Position_Translate(dy, vel, y_flag);
			AGV_Position_GoStraight(dx, vel, x_flag);
		}
		else if(AGV_Direction == 1)
		{
			AGV_Position_GoStraight(dy, vel, y_flag);
			AGV_Position_Translate(dx, vel, x_flag^1);
		}
		else if (AGV_Direction == 2)
		{
			AGV_Position_Translate(dy, vel, y_flag^1);
			AGV_Position_GoStraight(dx, vel, x_flag^1);
		}
		else if (AGV_Direction == 3)
		{
			AGV_Position_GoStraight(dy, vel, y_flag^1);
			AGV_Position_Translate(dx, vel, x_flag);
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
	
	while(can.rxFrameFlag == false);
	can.rxFrameFlag = false;
	
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
	
	while(can.rxFrameFlag == false);
	can.rxFrameFlag = false;
}

/**
 * @brief 出发
 * 
 */
void AGV_Position_Depart_1()
{
	Yaw_MovingOffsetStart();
	Emm_V5_Pos_Control(1, Wheel1PositiveDir^1, 100, 50, 4265, 2, 1);
	HAL_Delay(1);
	Emm_V5_Pos_Control(3, Wheel3PositiveDir^1, 100, 50, 4265, 2, 1);
	HAL_Delay(1);
	Emm_V5_Synchronous_motion(0); 								 // 广播地址0触发
	HAL_Delay(1);
	HAL_Delay(2000);
	AGV_Position_GoStraight(800, 100, 0);
}

void AGV_Position_Depart_2()
{
	Yaw_MovingOffsetStart();
	Emm_V5_Pos_Control(2, Wheel2PositiveDir, 100, 50, 4265, 2, 1);
	HAL_Delay(1);
	Emm_V5_Pos_Control(4, Wheel4PositiveDir, 100, 50, 4265, 2, 1);
	HAL_Delay(1);
	
	Emm_V5_Synchronous_motion(0); 								 // 广播地址0触发
	HAL_Delay(1);
	
	HAL_Delay(2000);
	AGV_Position_GoStraight(1010, 100, 1);
}

void AGV_Position_Qcode_to_Raw()
{
	AGV_Position_Translate(50, 150, 1);
	AGV_Position_GoStraight(810, 150, 1);
	Yaw_MovingOffsetEnd();
	AGV_TurnLeft();
	HAL_Delay(500);
	Yaw_MovingOffsetStart();
	AGV_Position_GoStraight(815, 150, 1);
	AGV_Position_Translate(35, 150, 0);
	Yaw_MovingOffsetEnd();
}
void AGV_Position_Raw_to_Rough()
{
	AGV_TurnRight();
	Yaw_MovingOffsetStart();
	AGV_Position_GoStraight(1800, 200, 0);
	Yaw_MovingOffsetEnd();
	AGV_TurnRight();
}
void AGV_Position_Rough_to_Temp()
{
	Yaw_MovingOffsetStart();
	AGV_Position_GoStraight(890, 150, 0);
	Yaw_MovingOffsetEnd();
    AGV_TurnRight();
	Yaw_MovingOffsetStart();
    AGV_Position_GoStraight(885, 150, 0);

}
void AGV_Position_Temp_to_Raw()
{
	AGV_Position_GoStraight(900, 150, 0);
	Yaw_MovingOffsetEnd();
    AGV_TurnRight();
	Yaw_MovingOffsetStart();
    AGV_Position_GoStraight(800, 150, 0);
}
/**
 * @brief 回到原点
 * 
 */
void AGV_Position_GoBack_1()
{
	AGV_Position_GoStraight(910, 150, 0);
	HAL_Delay(200);
	Yaw_MovingOffsetEnd();
	AGV_TurnRight();
	AGV_Position_GoStraight(1865, 150, 0);
	HAL_Delay(200);
	AGV_Position_Translate(150, 150, 0);
}
void AGV_Position_GoBack_2()//未修改
{
	AGV_Position_GoStraight(910, 150, 1);
	HAL_Delay(200);
	AGV_TurnLeft();
	AGV_Position_GoStraight(1865, 150, 1);
	HAL_Delay(200);
	AGV_Position_Translate(150, 150, 0);
}