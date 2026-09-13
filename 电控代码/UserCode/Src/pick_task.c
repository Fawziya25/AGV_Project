/**
 * @file pick_task.c
 * @brief 物料取放任务阶段实现
 */
#include "pick_task.h"
#include "positionPIDcontroller.h"  // AGV_Correct()

/* ========================== 静态辅助函数 ========================== */

/**
 * @brief 抓取到指定位置 (POS_A/B/C)
 * @param index 0=POS_A, 1=POS_B, 2=POS_C
 * @param color 要抓取的物块颜色
 */
static void GrabToPosition(uint8_t index, uint8_t color)
{
    Pick_MoveTo POS_OBSERVE_RAW;
    OpenMV_SwitchTo(color);
    OpenMV_WaitForNearStill();
    Pick_MoveTo POS_RAW;
    Pick_Gripper_Grab();

    switch(index) {
        case 0: Pick_MoveTo POS_A; break;
        case 1: Pick_MoveTo POS_B; break;
        case 2: Pick_MoveTo POS_C; break;
    }

    Pick_Gripper_Release();
}

/**
 * @brief 从车上位置放置到圆环
 * @param index 0=POS_C, 1=POS_B, 2=POS_A
 * @param ring 目标圆环编号 (1/2/3)
 */
static void PlaceToRing(uint8_t index, uint8_t ring)
{
    switch(index) {
        case 0: Pick_MoveTo POS_C; break;
        case 1: Pick_MoveTo POS_B; break;
        case 2: Pick_MoveTo POS_A; break;
    }
    Pick_Gripper_Grab();
    MOVE_TO_RING(ring);
    Pick_Gripper_Release();
}

/* ========================== 公共函数实现 ========================== */

void Pick_RAW(void)
{
    extern uint8_t Pick_Color1[3];
    extern uint8_t Pick_Color2[3];
    extern uint8_t RerunFlag;
    uint8_t *Pick_Color = (RerunFlag == 0) ? Pick_Color1 : Pick_Color2;
    // 预定位修正（使用颜色[2]）
    //等待用任意颜色识别出静止之后，切换最近的颜色，进行位置修正
    Pick_MoveTo POS_OBSERVE_RAW;
    Pick_Gripper_Observe();
    OpenMV_SwitchTo(Pick_Color[2]);
    OpenMV_WaitForNearStill();
    //OpenMV_SwitchCloseColor();
    AGV_Correct(3000);   /* 视觉闭环修正，超时见 positionPIDcontroller.h */
    // 三次抓取
    for (int i = 0; i < 3; i++) {
        GrabToPosition(i, Pick_Color[i]);
    }
    Pick_MoveTo POS_RUN;
}

void Pick_RAW_PLUS(void)
{
    extern uint8_t Pick_Color1[3];
    extern uint8_t Pick_Color2[3];
    extern uint8_t RerunFlag;
    uint8_t *Pick_Color = (RerunFlag == 0) ? Pick_Color1 : Pick_Color2;
    extern volatile int16_t target_dx;
    extern volatile int16_t target_dy;

    // 预定位修正（使用颜色[2]）
    //等待用任意颜色识别出静止之后，切换最近的颜色，进行位置修正
    Pick_MoveTo POS_OBSERVE_RAW;
    OpenMV_SwitchTo(Pick_Color[2]);
    OpenMV_WaitForStill();
    OpenMV_SwitchCloseColor();
    AGV_Correct(AGV_CORRECT_TMO_MS);   /* 视觉闭环修正，超时见 positionPIDcontroller.h */
    // 三次抓取
    for (int i = 0; i < 3; i++) {
        OpenMV_SwitchTo(Pick_Color[i]);
        Openmv_OLED_Show(); 
        if(target_dy <20) Pick_MoveTo POS_RAW;
        else if(target_dx>0) Pick_MoveTo POS_RAW_LEFT;
        else if(target_dx<0) Pick_MoveTo POS_RAW_RIGHT;
        Pick_Gripper_Grab();
        switch(i) {
        case 0: Pick_MoveTo POS_A; break;
        case 1: Pick_MoveTo POS_B; break;
        case 2: Pick_MoveTo POS_C; break;
        }
        Pick_Gripper_Release();   /* 移到位(A/B/C)后再松开夹爪，勿放在 switch 内 */
    }

    Pick_MoveTo POS_RUN;
}

void Place_ROUGH(void)
{
    extern uint8_t Place_Color1[3];
    extern uint8_t Place_Color2[3];
    extern uint8_t RerunFlag;
    uint8_t *Place_Color = (RerunFlag == 0) ? Place_Color1 : Place_Color2;

    // 视觉修正
    OpenMV_SwitchTo(0);
    Pick_MoveTo POS_OBSERVE_CIRCLE;
    Pick_Gripper_Observe();
    AGV_Correct(5000);   /* 视觉闭环修正，超时见 positionPIDcontroller.h */
    Pick_Gripper_Release();

    // 三次放置：C→环, B→环, A→环
    for (int i = 0; i < 3; i++) {
        PlaceToRing(i, Place_Color[2 - i]);  // 索引: 2→C, 1→B, 0→A
    }
    /* 由上层状态机控制返回 POS_RUN */
}

void Pick_ROUGH(void)
{
    extern uint8_t Place_Color1[3];

    GRAB_FROM_RING_AND_PLACE(Place_Color1[0], POS_A);
    GRAB_FROM_RING_AND_PLACE(Place_Color1[1], POS_B);
    GRAB_FROM_RING_AND_PLACE(Place_Color1[2], POS_C);

    Pick_MoveTo POS_RUN;
}

void Place_TEMP(void)
{
    extern uint8_t Place_Color1[3];
    extern uint8_t Pick_Color1[3];
    extern uint8_t Place_Color2[3];
    extern uint8_t RerunFlag;

    uint8_t *Place_Color = (RerunFlag == 0) ? Place_Color1 : Place_Color2;
    uint8_t is_high = (RerunFlag == 1);

    // 视觉修正
    if (RerunFlag == 0) {
        Pick_MoveTo POS_OBSERVE_CIRCLE;
        HAL_UART_Transmit(&huart1, (uint8_t *)"0", 1, HAL_MAX_DELAY);
    } else {
        Pick_MoveTo POS_OBSERVE_TEMP;
        uint8_t color_to_send = 0;
        if (Place_Color1[0] == 2) color_to_send = Pick_Color1[0];
        else if (Place_Color1[1] == 2) color_to_send = Pick_Color1[1];
        else if (Place_Color1[2] == 2) color_to_send = Pick_Color1[2];
        OpenMV_SwitchTo(color_to_send);
    }

    Pick_Gripper_Observe();
    AGV_Correct(3000);   /* 视觉闭环修正，超时见 positionPIDcontroller.h */
    Pick_Gripper_Release();

    // 三次放置：C→环, B→环, A→环
    GRAB_AND_PLACE_TO_RING_EX(POS_C, Place_Color[2], is_high);
    GRAB_AND_PLACE_TO_RING_EX(POS_B, Place_Color[1], is_high);
    GRAB_AND_PLACE_TO_RING_EX(POS_A, Place_Color[0], is_high);

    Pick_MoveTo POS_RUN;
}