#ifndef __TASK_H
#define __TASK_H

#include "main.h"
#include "task.h"
#include "usart.h"
#include "Emm_V5.h"
#include "yawPIDcontroller.h"
#include "hwt101.h"
#include "oled.h"
#include "chassis.h"
#include "yawPIDcontroller.h"
#include <stdio.h>
#include <stdint.h>
#include "pick_ctrl.h"
#include "QRcode.h"
#include "openmv.h"
#include "pick_task.h"

typedef enum
{
    Depart = 0,
    QRcode = 1,
    Move1 = 2,
    Pick1 = 3,
    Move2 = 4,
    Place1 = 5,
    Pick2 = 6,
    Move3 = 7,
    Place2 = 8,
    Move4 = 9,
    GoBack = 10,
    IDLE = 11,
}TaskcodeTypeDef;

extern TaskcodeTypeDef TaskCode;
extern uint8_t Route_id;   /* 路线选择 0/1，定义在 task.c，menu.c 切换 */

void AGV_Init(void);
void TaskProc(void);

#endif