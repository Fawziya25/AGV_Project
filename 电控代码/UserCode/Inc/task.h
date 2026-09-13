#ifndef __TASK_H
#define __TASK_H

#include "main.h"
#include "usart.h"
#include "Emm_V5.h"
#include "positionPIDcontroller.h"
#include "yawPIDcontroller.h"
#include "hwt101.h"
#include "oled.h"
#include "chassis.h"
#include "yawPIDcontroller.h"
#include <stdio.h>
#include <stdint.h>
#include "pick.h"
#include "QRcode.h"
#include "openmv.h"

typedef enum
{
    // 从出发点离开并扫码,到达原料区
    Depart = 0,
    // 取料
    Pick = 1,
    // 到达粗加工区
    Turn1 = 2,
    Move1 = 3,
    Turn2 = 4,
    // 到达暂存区
    Move2 = 5,
    Turn3 = 6,
    Move3 = 7,
    // 放料
    Place1 = 8,
    // 回到出发点
    Move4 = 9,
    Turn4 = 10,
    GoBack = 11,
    IDLE = 12,
    Qcode= 13,
    Correct=14,
    Move5 = 15,
    Turn5 = 16,
    Move6 = 17,
    Place2 = 18,
    Move7 = 19,
    Turn6 = 20,
    Move8 = 21,
    Pick1 = 22,
    Pick2 = 23,
}TaskcodeTypeDef;

extern TaskcodeTypeDef TaskCode;

void AGV_Init(void);
void TaskProc(void);

void AGV_Correct(void);
void Pick_RAW(void);
void Place_ROUGH(void);
void Pick_ROUGH(void);
void Place_TEMP(void);

#endif