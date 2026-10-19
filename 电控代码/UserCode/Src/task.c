#include "task.h"
#include "hwt101.h"
#include "stm32f4xx_hal.h"
#include "yawPIDcontroller.h"
#include "pick_task.h"
#include "positionPIDcontroller.h"
#include "menu.h"
#include "test.h"  
#include "radar_task.h" /* 雷达四段路径规划（Radar/ 模块） */
#include <stdint.h>


TaskcodeTypeDef TaskCode;

// 重复运行标志位 0->第一轮 1->第二轮
uint8_t RerunFlag = 0;
uint8_t Route_id=0;

/**
 * @brief 初始化AGV，包括DWT、CAN、OLED、HWT101、YawPID
 *        末尾阻塞在 Menu_Init() 等 A8 启动（期间 C6/C8 切换路线）
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
void AGV_Init(void)
{
    OLED_Init();
    Dbg_Init();           /* 开TIM7调试服务:周期刷相机数据 + A8短按暂停/继续 */
    Pick_Init();
    Cam_Init();           /* 等待相机启动，5秒超时，成功后oled显示参数，失败则不显示*/
    DWT_Init();// 初始化DWT,用于微秒级延时
    AGV_Can_Init(); // 初始化CAN,用于电机通信
    Pick_Axis_Init(); // 步进电机两轴：当前物理位置(上电前放好的0位)=绝对零点，并清堵转/使能
    AGV_Position_Init(); // 初始化位置坐标系
    YawPID_Init();  /* 航向PID,增益见 yawPIDcontroller.h 的 YAW_PID_KP/KI/KD */
    HWT101_Init(&huart5);// 初始化HWT101
    Radar_Init();           /* 雷达客户端初始化（不发帧）；在 Menu_Init 之前，菜单里要选起点方格 */
    Menu_Init();            /* 阻塞等待A8启动，期间C6/C8切换路线 */
    Radar_SetDeployed(true);             /* 伸出（0°） */
    Radar_Init();                        /* 选默认启停区 + 位姿 + 路线（不发帧） */
    (void)Radar_RunOnce();               /* 握手→选区→设位姿→方格扫描→六段规划→STOP */
    Radar_SetDeployed(false);            /* 扫描/规划完，收入车内（300°） */
    HAL_Delay(500);                      /* 摘要再看 0.5s，然后直接开跑 */
    TaskCode = Depart;  /* 开始 */
}

/**
 * @brief 主循环任务调度（由 main 的 while(1) 轮询调用，状态机推进整体流程）
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
/* 桌面调试开关已搬走：原来这里的 STANDALONE_TEST 各测试模式，
   2026-10-04 移到 UserCode/Src/test.c，由 main() 在 AGV_Init() 之前
   调 Test_Switch() 分发（模式表见 test.h）。本文件只剩正式流程。 */

void TaskProc(void)
{
    switch(TaskCode)
    {
        case Depart:
            Pick_JumpTo POS_RUN;
            AGV_Position_Depart();
            TaskCode = QRcode;
            break;
//===================================到达扫码位置==================================
        case QRcode:
            QRcode_Task();
            TaskCode = Move1;
            break;
        case Move1:
            AGV_Position_Qcode_to_Raw();
            TaskCode = Pick1;
            break;
//===================================到达原料区==================================
        case Pick1:
            Pick_Correct();
            Pick_RAW();
            //Pick_RAW_PLUS(); /* 备用路线：读一次0x03三框颜色，A/C 位带底盘补偿（已实现，未启用）*/
            TaskCode = Move2;
            break;
        case Move2:
            AGV_Position_Raw_to_Rough();
            TaskCode = Place1;
            break;
//===================================到达粗加工区=================================
        case Place1:
            Place_Correct();
            Place_ROUGH();
            TaskCode = Pick2;
            break;
        case Pick2:
            Pick_ROUGH();
            TaskCode = Move3;
            break;
        case Move3:
            AGV_Position_Rough_to_Temp();
            TaskCode = Place2;
            break;
//===================================到达暂存区==================================
        case Place2:
            if(RerunFlag == 0) Place_Correct();
            else               Place_TEMP_Correct();
            Place_TEMP();
            TaskCode = (RerunFlag == 0) ? Move4 : GoBack;
            RerunFlag = 1;
            break;
//====================================二周目====================================
        case Move4:
            AGV_Position_Temp_to_Raw();
            TaskCode = Pick1;
            break;
//====================================回到原点=================================== 
        case GoBack:
            AGV_Position_GoBack();
            TaskCode = IDLE;
            break;
        case IDLE:
            Pick_JumpTo POS_START;
            break;
    }
}
