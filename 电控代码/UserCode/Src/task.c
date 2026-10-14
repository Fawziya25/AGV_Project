#include "task.h"
#include "hwt101.h"
#include "stm32f4xx_hal.h"
#include "yawPIDcontroller.h"
#include "pick_task.h"
#include "positionPIDcontroller.h"
#include "menu.h"
#include "test.h"       /* Dbg_Init —— 原 dbg.h，已并进 test.h/test.c */
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
    Dbg_Init();           /* 开TIM7调试服务:周期刷相机数据 + A8短按暂停/继续(必须在OLED_Init之后) */
    Pick_Init();
    Cam_Init();           /* 等待相机启动，5秒超时，成功后oled显示参数，失败则不显示*/
    DWT_Init();// 初始化DWT,用于微秒级延时
    AGV_Can_Init(); // 初始化CAN,用于电机通信
    YawPID_Init();  /* 航向PID,增益见 yawPIDcontroller.h 的 YAW_PID_KP/KI/KD */
    HWT101_Init(&huart5);// 初始化HWT101
    Menu_Init();            /* 阻塞等待A8启动，期间C6/C8切换路线 */
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
            if(Route_id==0) AGV_Position_Depart_1();
            else AGV_Position_Depart_2();
            TaskCode = QRcode;
            break;
//===================================到达扫码位置==================================
        case QRcode:
            QRcode_Task(); //QRcode_Task() 内部：扫码 → 显示 → 把任务码下发给相机
            TaskCode = Move1;
            break;
        case Move1:
            AGV_Position_Qcode_to_Raw();
            TaskCode = Pick1;
            break;
//===================================到达原料区==================================
        case Pick1:
            Pick_RAW();
            //Pick_RAW_PLUS();两者选一个
            TaskCode = Move2;
            break;
        case Move2:
            AGV_Position_Raw_to_Rough();
            TaskCode = Place1;
            break;
//===================================到达粗加工区=================================
        case Place1:
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
            if(Route_id==0) AGV_Position_GoBack_1();
            else AGV_Position_GoBack_2();
            TaskCode = IDLE;
            break;
        case IDLE:
            Pick_JumpTo POS_START;
            break;
    }
}

/* 注：原TIM7节拍回调已移除——ZeroBias_Feed() 现由 hwt101.c 每解析一帧角度帧自动调用。
   TIM7 定时器不再启动，CubeMX 生成的 MX_TIM7_Init 仅占资源，可保留不影响运行。 */
