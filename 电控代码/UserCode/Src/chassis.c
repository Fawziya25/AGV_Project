#include "chassis.h"
#include "Emm_V5.h"
#include "delay.h"
#include "stm32f4xx_hal.h"

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
	Emm_V5_Pos_Control(1, Wheel1PositiveDir^1, 100, 50, 4265, 2, 1);
	HAL_Delay(1);
	Emm_V5_Pos_Control(3, Wheel3PositiveDir^1, 100, 50, 4265, 2, 1);
	HAL_Delay(1);
	
	Emm_V5_Synchronous_motion(0); 								 // 广播地址0触发
	HAL_Delay(1);
	
	HAL_Delay(1500);
}

/**
 * @brief 回到原点
 * 
 */
void AGV_Position_GoBack()
{
	AGV_Position_GoStraight(1870, 150, 0);
	HAL_Delay(200);
	AGV_Position_Translate(150, 150, 0);
}