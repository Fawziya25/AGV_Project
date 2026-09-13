#include "task.h"
#include "hwt101.h"
#include "stm32f4xx_hal.h"
#include "yawPIDcontroller.h"
#include "pick_task.h"
#include "positionPIDcontroller.h"
#include "menu.h"
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
    OpenMV_Init();           /* 等待相机启动，5秒超时，成功后oled显示参数，失败则不显示*/
    DWT_Init();// 初始化DWT,用于微秒级延时
    AGV_Can_Init(); // 初始化CAN,用于电机通信
    YawPID_Init();  /* 航向PID,增益见 yawPIDcontroller.h 的 YAW_PID_KP/KI/KD */
    HWT101_Init(&huart5);// 初始化HWT101
    Pick_Init(); /* 机械臂上电+收臂到运输位(内部 Pick_MoveTo POS_START) */
    Menu_Init();            /* 阻塞等待A8启动，期间C6/C8切换路线 */
    TaskCode = Depart;  /* 开始 */
}

/**
 * @brief 主循环任务调度（由 main 的 while(1) 轮询调用，状态机推进整体流程）
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
/* 桌面调试开关: 1=机械臂测试, 2=OpenMV数据测试, 3=OLED测试, 4=扫码测试, 0=完整AGV流程 */
#define STANDALONE_TEST  0

void TaskProc(void)
{
    Yaw_OLED_Show();   /* OLED固定行显示当前航向(调试) */

#if STANDALONE_TEST == 1
    // 原有机械臂测试
    static int once_flag = 0;
if (!once_flag) {
    once_flag = 1;
    POS_RAW
        
    return;

#elif STANDALONE_TEST == 2
    //位置修正测试
    static int once_flag = 0;
if (!once_flag) {
    once_flag = 1;
    //位置修正测试
    Pick_MoveTo POS_OBSERVE_CIRCLE;
    Pick_Gripper_Observe();
    OpenMV_SwitchTo(0);
    AGV_Correct(8000);
    Pick_MoveTo POS_OBSERVE_RAW;
    OpenMV_SwitchTo(1);
    AGV_Correct(8000);
    Pick_MoveTo POS_DEFAULT;
}
    return;
#elif STANDALONE_TEST == 3
    //抓物块测试
    static int once_flag = 0;
if (!once_flag) {
    once_flag = 1;
    QRcode_Task();
    Place_ROUGH();
    //Pick_RAW();
    Pick_MoveTo POS_DEFAULT;
}
    return;
#elif STANDALONE_TEST == 4
    //路径测试
    Pick_MoveTo POS_RAW;
    AGV_Position_Depart_1();
    HAL_Delay(1000);
    AGV_Position_Qcode_to_Raw();
    HAL_Delay(1000);
    AGV_Position_Raw_to_Rough();
    HAL_Delay(1000);
    AGV_Position_Rough_to_Temp();
    HAL_Delay(1000);
    // AGV_Position_Temp_to_Raw();
    // HAL_Delay(1000);
    AGV_Position_GoBack_1();
    return;
#else
    switch(TaskCode)
    {
        case Depart:
            Pick_MoveTo POS_RUN;
            if(Route_id==0) AGV_Position_Depart_1();
            else AGV_Position_Depart_2();
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
            Pick_RAW();
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
            Pick_MoveTo POS_DEFAULT;
            break;
        }
#endif

}

/* 注：原TIM7节拍回调已移除——ZeroBias_Feed() 现由 hwt101.c 每解析一帧角度帧自动调用。
   TIM7 定时器不再启动，CubeMX 生成的 MX_TIM7_Init 仅占资源，可保留不影响运行。 */
