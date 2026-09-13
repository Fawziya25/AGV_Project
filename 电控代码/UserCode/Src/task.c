#include "task.h"


TaskcodeTypeDef TaskCode;

// 重复运行标志位 0->第一轮 1->第二轮
uint8_t RerunFlag = 0;

/**
 * @brief 初始化AGV，包括DWT、CAN、OLED、HWT101、YawPID
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
void AGV_Init(void)
{
    // 初始化DWT,用于微秒级延时
    DWT_Init();
    // 初始化CAN,用于电机通信
    AGV_Can_Init();
    // 初始化HWT101
    HWT101_Init(&huart5);
    HWT101_SetRate(100);
    /* 关闭自动零偏校准（AGV推荐）：运行中自动校准容易把震动误采为零偏 */
    HWT101_SetAutoCal(false);
    /* 上电静止时手动获取一次零偏，压低长时间运行的积分漂移率
       必须在小车完全静止时执行，静止时长可按现场漂移情况调整 */
    HWT101_SetYawZero();
    HWT101_ManualCalStart();
    HAL_Delay(10000);
    HWT101_ManualCalStop();
    // 初始化YawPID
    YawPID_Init(1.25f, 0.035f, 0.045f);
    Pick_Init();
    OpenMV_Init();           /* 启动openMV串口DMA接收 */
    TaskCode = Depart;
    //Pick_MoveTo POS_START; 之后整体运行的时候要打开，控制尺寸在范围之内
    AGV_OLED_ShowReady();
}

/**
 * @brief 主循环任务调度，TIM7节拍到来时在同一拍内执行完整位置闭环
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
/* 桌面调试开关: 1=机械臂测试, 2=OpenMV数据测试, 3=OLED测试, 4=扫码测试, 0=完整AGV流程 */
#define STANDALONE_TEST  0

void TaskProc(void)
{
#if STANDALONE_TEST == 1
    // 原有机械臂测试
    static uint8_t testStarted = 0;
    if(!testStarted)
    {
        testStarted = 1;
        Pick_DebugTest();
        Pick_MoveTo POS_DEFAULT;
    }
    return;

#elif STANDALONE_TEST == 2
    // OpenMV数据接收测试：接收并显示5个字段
    static uint8_t openmvStarted = 0;
    if(!openmvStarted)
    {
        openmvStarted = 1;
        OLED_Init();
        OLED_Clear();                        /* OpenMV_Init已在AGV_Init中调用 */
    }
    if(OpenMV_RxFlag)                        /* 收到一帧新数据 */
    {
        OpenMV_RxFlag = 0;
        OpenMV_ShowParams();   /* 3行显示全部5个视觉参数 */
    }
    return;
#else
    switch(TaskCode)
    {
        case Depart:
            Pick_MoveTo POS_RUN;
            HAL_Delay(2000);
            AGV_Position_Depart_1();
            HAL_Delay(200);
            AGV_Position_GoStraight(730, 100, 0);
            HAL_Delay(500);
            TaskCode = Qcode;
            break;
//===================================到达扫码位置==================================
         case Qcode:
            OLED_Clear();
            AGV_QRcode_StartScan();
            AGV_QRcode_WaitForScan();
            uint8_t sep = QRcode_Buffer[8];
            QRcode_Buffer[8] = '\0';
            OLED_ShowString(0, 0, (uint8_t *)QRcode_Buffer, 32, 1);
            QRcode_Buffer[8] = sep;
            OLED_ShowString(0, 32, (uint8_t *)&QRcode_Buffer[8], 32, 1);
            OLED_Refresh();
            TaskCode = Move1;
            break;
        case Move1:
            AGV_Position_Translate(50, 150, 1);
            HAL_Delay(200);
            AGV_Position_GoStraight(740, 150, 1);
            HAL_Delay(200);
            TaskCode = Turn1;
            break;
        case Turn1:
            AGV_SetYaw(90);
            if(TurnCpltFlag)
            {
                TurnCpltFlag = 0;
                TaskCode = Move2;
                HAL_Delay(200);
            }
            break;
        case Move2:
            AGV_Position_GoStraight(1215, 150, 1);
            HAL_Delay(200);
            AGV_Position_Translate(50, 150, 0);
            TaskCode = Pick1;
            break;
//===================================到达原料区==================================
    case Pick1:  
    {
        HWT101_ManualCalStart();
        Pick_RAW();
        HWT101_ManualCalStop();
        TaskCode = Move3;
        break;
    }
    case Move3:
        AGV_Position_GoStraight(385, 150, 0);
        HAL_Delay(200);
        TaskCode = Turn2;
        break;
    // 设置角度为0°
    case Turn2:
        AGV_SetYaw(0);
        if(TurnCpltFlag)
        {
            TurnCpltFlag = 0;
            TaskCode = Move4;
            HAL_Delay(200);
        }
        break;
    // 前往粗加工区
    case Move4:
        AGV_Position_GoStraight(1790, 200, 0);
        HAL_Delay(200);
        TaskCode = Turn3;
        break;
    // 设置角度为-90°
    case Turn3:
        AGV_SetYaw(-90);
        if(TurnCpltFlag)
        {
            TurnCpltFlag = 0;
            TaskCode = Place1;
            HAL_Delay(200);
        }
        break;
//======================到达粗加工区======================
    case Place1:
        HWT101_ManualCalStart();
        Place_ROUGH();
        TaskCode = Pick2;
        break;
    case Pick2:
        Pick_ROUGH();
        HWT101_ManualCalStop();
        TaskCode = Move5;
        break;
    
    // 离开粗加工区
    case Move5:
        AGV_Position_GoStraight(850, 150, 0);
        HAL_Delay(200);
        TaskCode = Turn4;
        break;
    // 设置角度为-180°
    case Turn4:
        AGV_SetYaw(-180);
        if(TurnCpltFlag)
        {
            TurnCpltFlag = 0;
            TaskCode = Move6;
            HAL_Delay(200);
        }
        break;
    // 移动至暂存区
    case Move6:
        AGV_Position_GoStraight(960, 150, 0);
        HAL_Delay(200);
        TaskCode = Place2;
        break;
//====================到达暂存区=====================
    case Place2:
        HWT101_ManualCalStart();
        Place_TEMP();
        HWT101_ManualCalStop();
        TaskCode = Move7;
        break;
    // 离开暂存区
    case Move7:
        AGV_Position_GoStraight(850, 150, 0);
        HAL_Delay(500);
        TaskCode = Turn5;
        break;
    // 设置角度为90°
    case Turn5:
        AGV_SetYaw(90);
        if(TurnCpltFlag)
        {
            TurnCpltFlag = 0;
            TaskCode = (RerunFlag == 0) ? Move8 : GoBack;
            HAL_Delay(200);
        }
        break;
    case Move8:
        AGV_Position_GoStraight(440, 150, 0);
        HAL_Delay(200);
        RerunFlag = 1;
        TaskCode = Pick1;
        break;
    case GoBack:
        AGV_Position_GoBack();
        TaskCode = IDLE;
    case IDLE:
        Pick_MoveTo POS_DEFAULT;
        break;
    }
#endif
}

// TIM7中断回调函数，10ms执行一次
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM7)
    {

    }
}


//==================================================
/*
 * @brief 视觉闭环修正：持续调整直到对准
 * @note  直接调用，执行完自动返回
 */
void AGV_Correct(void)
{
    extern volatile int16_t block_dx;
    extern volatile int16_t block_dy;
    extern volatile int16_t ring_dx;
    extern volatile int16_t ring_dy;
    extern volatile int16_t openmv_cmd;
    while(1)
    {
        /* 根据 openmv_cmd 选择偏差来源：8=圆环(ring)，1~6=物块(block) */
        int16_t dx = (openmv_cmd == 8) ? ring_dx : block_dx;
        int16_t dy = (openmv_cmd == 8) ? ring_dy : block_dy;
        if (dx == -999 || dy == -999)
        {
            HAL_Delay(50);
            break;
        }
        int16_t dx_correction = (int16_t)(dx * 2.0);
        int16_t dy_correction = (int16_t)(dy * 2.0);
        if (abs(dx_correction) <= 1 && abs(dy_correction) <= 1)
        {
            break;
        }
        if (dx_correction != 0)
        {
            uint8_t dir_x = (dx_correction > 0) ? 0 : 1;
            uint16_t dist_x = abs(dx_correction);
            AGV_Position_GoStraight(dist_x, 100, dir_x);
        }
        if (dy_correction != 0)
        {
            uint8_t dir_y = (dy_correction > 0) ? 1 : 0;
            uint16_t dist_y = abs(dy_correction);
            AGV_Position_Translate(dist_y, 100, dir_y);
        }
        OpenMV_ShowParams();   /* 3行显示全部5个视觉参数 */
    }
}

/*
 * @brief 任务阶段1：一次调用抓 3 个物块到车上 A/B/C 位置
 *        第1次发送 Pick_Color[0] 并调用一次修正；第2、3次直接抓取（不修正）
 *        第二次调用(第二轮)时 RerunFlag==1，自动改用 Pick_Color2
 */
void Pick_RAW(void)
{
    extern uint8_t Pick_Color1[3];
    extern uint8_t Pick_Color2[3];
    extern volatile int16_t block_dx;
    extern volatile int16_t block_dy;
    // 根据 RerunFlag 选择使用哪一组颜色（0=第一轮, 1=第二轮）
    uint8_t *Pick_Color = (RerunFlag == 0) ? Pick_Color1 : Pick_Color2;

    char cmd[4];
    /* 第1次：发送 Pick_Color[0]，修正，抓取到 POS_A */
    Pick_Gripper_Release();
    Pick_MoveTo POS_OBSERVE_RAW;
    sprintf(cmd, "%d", Pick_Color[0]);
    HAL_UART_Transmit(&huart1, (uint8_t *)cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(300);
    OpenMV_ShowParams();          /* 3行显示全部5个视觉参数 */
    AGV_Correct();                /* 调用一次修正（无数据则直接返回） */
    /* 等待物块对准：block_dx 和 block_dy 同时小于 5（像素）才进行抓取 */
    while (abs(block_dx) >= 5 || abs(block_dy) >= 5)
    {
        HAL_Delay(50);
        OpenMV_ShowParams();   /* 实时显示等待过程中的视觉参数 */
    }
    Pick_MoveTo POS_RAW;
    Pick_Gripper_Grab();
    Pick_MoveTo POS_A;
    Pick_Gripper_Release();
    /* 第2次：发送 Pick_Color[1]，直接抓取到 POS_B（不修正） */
    Pick_Gripper_Release();
    Pick_MoveTo POS_OBSERVE_RAW;
    sprintf(cmd, "%d", Pick_Color[1]);
    HAL_UART_Transmit(&huart1, (uint8_t *)cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(300);
    OpenMV_ShowParams();          /* 3行显示全部5个视觉参数 */
    /* 等待物块对准：block_dx 和 block_dy 同时小于 5（像素）才进行抓取 */
    while (abs(block_dx) >= 5 || abs(block_dy) >= 5)
    {
        HAL_Delay(50);
        OpenMV_ShowParams();   /* 实时显示等待过程中的视觉参数 */
    }
    Pick_MoveTo POS_RAW;
    Pick_Gripper_Grab();
    Pick_MoveTo POS_B;
    Pick_Gripper_Release();

    /* 第3次：发送 Pick_Color[2]，直接抓取到 POS_C */
    Pick_Gripper_Release();
    Pick_MoveTo POS_OBSERVE_RAW;
    sprintf(cmd, "%d", Pick_Color[2]);
    HAL_UART_Transmit(&huart1, (uint8_t *)cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(300);
    OpenMV_ShowParams();          /* 3行显示全部5个视觉参数 */
    /* 等待物块对准：block_dx 和 block_dy 同时小于 5（像素）才进行抓取 */
    while (abs(block_dx) >= 5 || abs(block_dy) >= 5)
    {
        HAL_Delay(50);
        OpenMV_ShowParams();   /* 实时显示等待过程中的视觉参数 */
    }
    Pick_MoveTo POS_RAW;
    Pick_Gripper_Grab();
    Pick_MoveTo POS_C;
    Pick_Gripper_Release();
    /* 全部抓完，回到运行位置 */
    Pick_MoveTo POS_RUN;
}


/*
 * @brief 任务阶段2：一次调用完成放置——修正后 C→圆环、B→圆环、A→圆环
 *        先发送"0"给相机并调用一次修正，然后依次放置，最后回到 POS_RUN
 */
void Place_ROUGH(void)
{
    extern uint8_t Place_Color1[3];
    extern uint8_t Place_Color2[3];

    // 根据 RerunFlag 选择放置颜色：0=第一轮 Place_Color1, 1=第二轮 Place_Color2
    uint8_t *Place_Color = (RerunFlag == 0) ? Place_Color1 : Place_Color2;

    uint8_t target_ring;

    /* 发送 0 给相机，调用一次修正 */
    HAL_UART_Transmit(&huart1, (uint8_t *)"0", 1, HAL_MAX_DELAY);
    Pick_MoveTo POS_OBSERVE_CIRCLE;
    HAL_Delay(300);
    OpenMV_ShowParams();          /* 3行显示全部5个视觉参数 */
    AGV_Correct();                /* 调用一次修正（无数据则直接返回） */

    Pick_Gripper_Release();
    /* 第1次：从 POS_C 放到圆环 */
    target_ring = Place_Color[2];
    Pick_MoveTo POS_C;
    Pick_Gripper_Grab();
    if (target_ring == 1)      Pick_MoveTo POS_1;
    else if (target_ring == 2) Pick_MoveTo POS_2;
    else if (target_ring == 3) Pick_MoveTo POS_3;
    Pick_Gripper_Release();

    /* 第2次：从 POS_B 放到圆环 */
    target_ring = Place_Color[1];
    Pick_MoveTo POS_B;
    Pick_Gripper_Grab();
    if (target_ring == 1)      Pick_MoveTo POS_1;
    else if (target_ring == 2) Pick_MoveTo POS_2;
    else if (target_ring == 3) Pick_MoveTo POS_3;
    Pick_Gripper_Release();

    /* 第3次：从 POS_A 放到圆环 */
    target_ring = Place_Color[0];
    Pick_MoveTo POS_A;
    Pick_Gripper_Grab();
    if (target_ring == 1)      Pick_MoveTo POS_1;
    else if (target_ring == 2) Pick_MoveTo POS_2;
    else if (target_ring == 3) Pick_MoveTo POS_3;
    Pick_Gripper_Release();

    /* 全部放完，回到运行位置 */
    Pick_MoveTo POS_RUN;
}

/*
 * @brief 任务阶段3：一次调用完成取回——环→A、环→B、环→C，最后回到 POS_RUN
 */
void Pick_ROUGH(void)
{
    extern uint8_t Place_Color1[3];

    uint8_t target_ring;

    /* 第1次：从圆环取回物块放到 POS_A */
    target_ring = Place_Color1[0];
    Pick_Gripper_Release();
    OpenMV_ShowParams();          /* 3行显示全部5个视觉参数 */
    if (target_ring == 1)      Pick_MoveTo POS_1;
    else if (target_ring == 2) Pick_MoveTo POS_2;
    else if (target_ring == 3) Pick_MoveTo POS_3;
    Pick_Gripper_Grab();
    Pick_MoveTo POS_A;
    Pick_Gripper_Release();

    /* 第2次：从圆环取回物块放到 POS_B */
    target_ring = Place_Color1[1];
    Pick_Gripper_Release();
    OpenMV_ShowParams();
    if (target_ring == 1)      Pick_MoveTo POS_1;
    else if (target_ring == 2) Pick_MoveTo POS_2;
    else if (target_ring == 3) Pick_MoveTo POS_3;
    Pick_Gripper_Grab();
    Pick_MoveTo POS_B;
    Pick_Gripper_Release();

    /* 第3次：从圆环取回物块放到 POS_C */
    target_ring = Place_Color1[2];
    Pick_Gripper_Release();
    OpenMV_ShowParams();
    if (target_ring == 1)      Pick_MoveTo POS_1;
    else if (target_ring == 2) Pick_MoveTo POS_2;
    else if (target_ring == 3) Pick_MoveTo POS_3;
    Pick_Gripper_Grab();
    Pick_MoveTo POS_C;
    Pick_Gripper_Release();

    /* 全部取回，回到运行位置 */
    Pick_MoveTo POS_RUN;
}

void Place_TEMP(void)
{
    extern uint8_t Place_Color1[3];
    extern uint8_t Place_Color2[3];
    // 根据 RerunFlag 选择放置颜色：0=第一轮 Place_Color1, 1=第二轮 Place_Color2
    uint8_t *Place_Color = (RerunFlag == 0) ? Place_Color1 : Place_Color2;
    uint8_t target_ring;

    /* 发送 0 给相机，调用一次修正 */
    HAL_UART_Transmit(&huart1, (uint8_t *)"0", 1, HAL_MAX_DELAY);
    Pick_MoveTo POS_OBSERVE;
    HAL_Delay(500);
    OpenMV_ShowParams();          /* 3行显示全部5个视觉参数 */
    AGV_Correct();                /* 调用一次修正（无数据则直接返回） */

    /* 第1次：从 POS_C 放到圆环 */
    target_ring = Place_Color[2];
    Pick_MoveTo POS_C;
    Pick_Gripper_Grab();
    if      (RerunFlag == 1 && target_ring == 1) Pick_MoveTo POS_1_HIGE;
    else if (RerunFlag == 1 && target_ring == 2) Pick_MoveTo POS_2_HIGE;
    else if (RerunFlag == 1 && target_ring == 3) Pick_MoveTo POS_3_HIGE;
    else if (target_ring == 1)                   Pick_MoveTo POS_1;
    else if (target_ring == 2)                   Pick_MoveTo POS_2;
    else if (target_ring == 3)                   Pick_MoveTo POS_3;
    Pick_Gripper_Release();

    /* 第2次：从 POS_B 放到圆环 */
    target_ring = Place_Color[1];
    Pick_MoveTo POS_B;
    Pick_Gripper_Grab();
    if      (RerunFlag == 1 && target_ring == 1) Pick_MoveTo POS_1_HIGE;
    else if (RerunFlag == 1 && target_ring == 2) Pick_MoveTo POS_2_HIGE;
    else if (RerunFlag == 1 && target_ring == 3) Pick_MoveTo POS_3_HIGE;
    else if (target_ring == 1)                   Pick_MoveTo POS_1;
    else if (target_ring == 2)                   Pick_MoveTo POS_2;
    else if (target_ring == 3)                   Pick_MoveTo POS_3;
    Pick_Gripper_Release();

    /* 第3次：从 POS_A 放到圆环 */
    target_ring = Place_Color[0];
    Pick_MoveTo POS_A;
    Pick_Gripper_Grab();
    if      (RerunFlag == 1 && target_ring == 1) Pick_MoveTo POS_1_HIGE;
    else if (RerunFlag == 1 && target_ring == 2) Pick_MoveTo POS_2_HIGE;
    else if (RerunFlag == 1 && target_ring == 3) Pick_MoveTo POS_3_HIGE;
    else if (target_ring == 1)                   Pick_MoveTo POS_1;
    else if (target_ring == 2)                   Pick_MoveTo POS_2;
    else if (target_ring == 3)                   Pick_MoveTo POS_3;
    Pick_Gripper_Release();

    /* 全部放完，回到运行位置 */
    Pick_MoveTo POS_RUN;
}