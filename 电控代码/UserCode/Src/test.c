
/**
 * @file test.c
 * @brief 桌面测试入口 + 运行时调试服务（原 dbg.c 已并进本文件）
 *
 * 两个互相独立的开关，都在 test.h 顶部：
 *   TEST_MODE   上电跑哪个测试（0=不测试，跑正式流程）
 *   DBG_ENABLE  运行时调试服务：TIM7 周期刷 OLED + C8 短按暂停/继续
 *   正式比赛两个都设 0。DBG_ENABLE=0 时 Dbg_Init()/Dbg_Service() 变成空语句，
 *   别的模块里的调用点一个字不用删 —— 这正是把调试服务跟测试入口放一起的原因：
 *   调试和测试本来就是一伙的，正式版一起关掉。
 *
 * ── 第一部分：运行时调试服务（DBG_ENABLE） ──────────────────────────
 * 为什么中断里只扫键不刷屏、为什么接管 HAL_Delay，见 test.h 里的说明。
 *
 * ── 第二部分：测试模式分发（TEST_MODE） ────────────────────────────
 * main() 是在 AGV_Init() **之前** 调 Test_Switch() 的，那时 OLED / 相机 / CAN /
 * HWT101 / DWT 一个都还没起来。所以 Test_HwInit() 只做**测试之间共用**的那几样
 * （OLED / DWT / 航向PID），剩下的**由每个测试函数自己按需初始化** ——
 * 只测臂的别去等相机，只测路径的别去初始化机械臂。这是有意的分工：
 * 某个测试自己那几行初始化就写在它函数开头，看得见、好改。
 *
 * 测试模式下 Test_Switch() 跑完进死循环、不返回，所以程序不会走到 AGV_Init()，
 * 也不会进正式流程。这是有意的，不是漏了 return。
 *
 * 加新测试的三步：
 *   1) test.h 的模式表里给新测试占一个号；
 *   2) 本文件上面「测试函数声明」加一条，下面写实现；
 *   3) Test_Switch() 的 switch 里加一个 case。
 */

#include "test.h"
#include "chassis.h"
#include "stm32f4xx_hal.h"
#include "task.h"                   /* 正式流程状态机 + AGV_Position_* */
#include "pick_task.h"              /* Pick_RAW / Place_ROUGH / Place_TEMP */
#include "pick.h"                   /* Pick_JumpTo / POS_* / Pick_Gripper_* */
#include "camera.h"                 /* cam_mode/cam_flag/dx/dy + Cam_Init/Cam_SwitchTo */
#include "positionPIDcontroller.h"  /* AGV_Correct / AGV_Position_Stop */
#include "QRcode.h"                 /* Pick_Color1[3] 等 12 位任务码 */
#include "oled.h"                   /* OLED_Clear / OLED_ShowString / OLED_Refresh */
#include "servo.h"                  /* Servo_InitId(SERVO_RADAR)：上电先把雷达舵机复位到收起位 */
#include "radar_task.h"             /* 模式 10：Radar_Init / Radar_RunOnce / Radar_Path[5] */
#include <stdint.h>
#include <stdio.h>                  /* sprintf */

#define START_KEY_GPIO   GPIOA
#define START_KEY_PIN    GPIO_PIN_8      /* A8 启动 */
#define DBG_KEY_GPIO     GPIOC
#define DBG_KEY_PIN      GPIO_PIN_8      /* C8 暂停 */

#if DBG_ENABLE

extern TIM_HandleTypeDef htim7;      /* Core/Src/tim.c，MX_TIM7_Init 已初始化过 */

/* 刷屏周期：一整屏(软件I2C约30ms)刷太勤纯浪费，还会跟主程序的 OLED 抢屏闪得厉害 */
#define DBG_REFRESH_MS   200U

/* 消抖：TIM7 10ms 一拍，连续 3 拍同电平算稳定（约 30~40ms，和 menu.c 一致） */
#define DBG_DEBOUNCE_CNT 3U

static volatile uint8_t s_paused      = 0;   /* 1=已暂停，主程序卡在 Dbg_Service */
static volatile uint8_t s_refresh_req = 0;   /* 1=到刷屏节拍了 */
static volatile uint8_t s_in_service  = 0;   /* 重入保护，见 Dbg_Service 注释 */
static volatile uint8_t s_suspended   = 0;   /* 1=别刷屏：屏幕被别的模块占着(启动菜单) */

/* 按键消抖状态（只在 TIM7 中断里动） */
static uint8_t s_key_prev   = 0;
static uint8_t s_key_cnt    = 0;
static uint8_t s_key_stable = 0;
static uint8_t s_key_armed  = 0;   /* 见过一次"松开"之后才允许触发 */

static uint16_t s_tick = 0;

static void Dbg_DelayRaw(uint32_t ms);

/**
 * @brief 启动调试服务：清状态 + 开 TIM7 中断
 * @note  必须在 OLED_Init() 之后调 —— 中断要起来，之后刷屏要驱动软件 I2C。
 */
void Dbg_Init(void)
{
    s_paused      = 0;
    s_refresh_req = 0;
    s_in_service  = 0;
    s_suspended   = 0;
    s_key_prev    = 0;
    s_key_cnt     = 0;
    s_key_stable  = 0;
    s_key_armed   = 0;
    s_tick        = 0;

    HAL_TIM_Base_Start_IT(&htim7);   /* MX_TIM7_Init 已配好：84MHz/84/10000 = 10ms 一拍 */
}


void Dbg_Suspend(void)
{
    s_suspended = 1U;
}

/**
 * @brief 恢复调试刷屏，并立刻重画一次（返回后第一处 HAL_Delay 就会刷出来）
 */
void Dbg_Resume(void)
{
    s_suspended   = 0U;
    s_refresh_req = 1U;   /* 菜单退出时 OLED_Clear 过，别等下一个 200ms 节拍 */
}

/**
 * @brief TIM7 中断，10ms 一拍：只做两件轻活 —— 扫键 + 置刷屏标志
 * @note  刷屏（约 30ms）不在这里做，原因见 test.h 顶部。
 *        工程里目前只有 TIM7 用周期中断，其它定时器若是以后开了中断，在这里
 *        按 Instance 再加分支即可（默认直接 return，不会误触发）。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t raw;

    if(htim->Instance != TIM7) return;

    /* ---------------- 1) 扫 C8(暂停键)：消抖 + 按下沿触发一次 ---------------- */
    raw = (HAL_GPIO_ReadPin(DBG_KEY_GPIO, DBG_KEY_PIN) == GPIO_PIN_RESET) ? 1U : 0U;

    if(raw == s_key_prev)
    {
        if(s_key_cnt < DBG_DEBOUNCE_CNT) s_key_cnt++;
    }
    else
    {
        s_key_prev = raw;
        s_key_cnt  = 0;
    }

    if(s_key_cnt >= DBG_DEBOUNCE_CNT && s_key_stable != raw)
    {
        s_key_stable = raw;
        if(raw == 1U)
        {
            /* 必须"见过一次松开"才认这次按下。否则**上电时就按着 C8**
               （或者按键抖动被读成常低）会被这里当成一次新按键 ——
               程序刚起来就停在暂停画面。 */
            if(s_key_armed) s_paused = (uint8_t)(!s_paused);
        }
        else
        {
            s_key_armed = 1U;
        }
    }

    /* ---------------- 2) 刷屏节拍 ---------------- */
    if(++s_tick >= (DBG_REFRESH_MS / 10U))
    {
        s_tick = 0;
        s_refresh_req = 1U;
    }
}

/**
 * @brief 调试服务：暂停判断 + 到点刷屏
 * @note  重入保护 s_in_service 是必需的，而且有两个来源：
 *          1) 暂停时要调 AGV_Position_Stop()，它内部的 Emm_V5_WaitRxFrame()
 *             被手工插了 Dbg_Service() —— 没保护就无限递归；
 *          2) 被盖掉的 HAL_Delay() 本身就调 Dbg_Service()，嵌套 HAL_Delay
 *             也会走回来。
 *        中断上下文直接返回：不该在 ISR 里进那个暂停死循环。
 *        s_suspended（启动菜单占着屏幕）时整个服务让路：既不刷屏也不响应暂停。
 */
void Dbg_Service(void)
{
    if(s_in_service || s_suspended || __get_IPSR() != 0U) return;
    s_in_service = 1;

    if(s_paused)
    {
        /* 步进电机收到的是"走 Xmm @ Yrpm"，轨迹是电机自己跑的 —— MCU 冻住
           车照走。所以暂停必须显式发一条停车。
           这句要求 CAN 已经起来了（Test_Switch() 开头那句 AGV_Can_Init()，现在无条件
           执行）—— CAN 没 Start 过的话，调用会卡在 can_SendCmd() 的
           `while(HAL_CAN_AddTxMessage(...) != HAL_OK);` 上，按一次 C8 就死循环。 */
        AGV_Position_Stop();

        while(s_paused)                 /* 死在这儿，等 C8 再按一下 */
        {
            Dbg_ShowScreen(1);          /* 暂停画面：数据照样显示，方便对着读数 */
            Dbg_DelayRaw(DBG_REFRESH_MS);
        }
        s_refresh_req = 1U;             /* 恢复后立刻重画一次正常画面 */
    }

    if(s_refresh_req)
    {
        s_refresh_req = 0U;
        Dbg_ShowScreen(0);
    }

    s_in_service = 0;
}

/**
 * @brief 裸延时：纯等，不跑服务
 * @note  不能用 HAL_Delay —— 那个被本文件盖掉后会回调 Dbg_Service()。
 *        虽然重入保护挡得住不会真递归，但语义上这里就该是纯等待。
 */
static void Dbg_DelayRaw(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    while((HAL_GetTick() - t0) < ms) { }
}

/**
 * @brief 盖掉 HAL 里的 __weak HAL_Delay：等时间的同时把调试服务跑起来
 * @note  这是整个方案的关键一步 —— 主程序经常一卡好几秒（等相机 10s、等
 *        电机 5s、菜单无限久），主循环那点时间根本指望不上。接管 HAL_Delay
 *        以后，**全工程每一处 HAL_Delay 都自动成了服务点，一处不用改**。
 *        只有几个纯 HAL_GetTick 自旋、不走 HAL_Delay 的循环需要手工补一句
 *        Dbg_Service()，已经补好了：positionPIDcontroller.c 的 AGV_Correct()、
 *        Emm_V5.c 的 Emm_V5_WaitArrive / Emm_V5_WaitRxFrame。
 *
 *        对外语义不变（还是"阻塞等这么久"，精度仍是 1ms 级 SysTick），
 *        只是偶尔一次 HAL_Delay(1) 会因为赶上刷屏节拍变长约 30ms。
 *        SysTick 在 HAL_Init() 里就起来了，早于任何一次 HAL_Delay 调用。
 */
void HAL_Delay(uint32_t Delay)
{
    uint32_t t0 = HAL_GetTick();

    while((HAL_GetTick() - t0) < Delay)
    {
        Dbg_Service();
    }
}

#endif  /* DBG_ENABLE */

/* ==========================================================================
 * 屏幕渲染：唯一一份整屏显示，**不随 DBG_ENABLE 走**
 * ==========================================================================
 * 为什么放在开关外面：Cam_Init() 里也要用它（原来那份 Cam_OLED_Show() 已经被它
 * 取代删掉了）。相机模块要显示"链路通了/超时了"是上电诊断，正式流程一样需要，
 * 跟"要不要周期刷新、要不要暂停"是两回事。开关只关周期刷新和暂停。
 * 所以 DBG_ENABLE=0 时这函数还在、`Cam_Init()` 里那句调用还在，只是没人周期调它。
 */

/**
 * @brief 整屏覆盖显示：相机 mode/flag/dx/dy + 任务码 12 个数字
 * @param paused 1=暂停画面（最后一行换成暂停提示）；0=正常画面
 *
 * 四行 16 号字正好铺满 128x64 屏：
 *   y=0   C:xx F:x            相机帧模式 + 是否识别到目标
 *   y=16  x:ddd y:ddd         偏差(px)
 *   y=32  542+312+452+123     任务码：一轮取/放 二轮取/放，各 3 位
 *   y=48  C8:pause / --PAUSE-- C8:GO
 *
 * 一次清屏、一次画完、一次 Refresh —— 调用方不要再跟着 Refresh，也别拆成两段画。
 * 扫码前四个数组还是 BSS 初值 0，显示出来就是 000+000+000+000（正常现象）。
 */
void Dbg_ShowScreen(uint8_t paused)
{
    char buf[24];

    OLED_Clear();

    sprintf(buf, "C:%02X F:%d", cam_mode, cam_flag);
    OLED_ShowString(0, 0, (uint8_t *)buf, 16, 0);

    sprintf(buf, "x:%d y:%d", dx, dy);
    OLED_ShowString(0, 16, (uint8_t *)buf, 16, 0);

    /* ---- 任务码：4 组各 3 位，组间 '+'，共 15 字符(120px < 128) ----
       手拼不用 sprintf：这 12 个值都该是 0~9 的一位数，拼出来长度固定；
       用 %d 的话编译器不知道值域，会按最长 3 位算长度警告可能溢出。 */
    {
        const uint8_t *grp[4] = { Pick_Color1, Place_Color1, Pick_Color2, Place_Color2 };
        char code[16];                  /* 12 位 + 3 个 '+' + '\0' */
        uint8_t i, k = 0U;

        for(i = 0U; i < 4U; i++)
        {
            if(i != 0U) code[k++] = '+';
            code[k++] = (char)('0' + (grp[i][0] % 10U));
            code[k++] = (char)('0' + (grp[i][1] % 10U));
            code[k++] = (char)('0' + (grp[i][2] % 10U));
        }
        code[k] = '\0';
        OLED_ShowString(0, 32, (uint8_t *)code, 16, 0);
    }

    OLED_ShowString(0, 48, (uint8_t *)(paused ? "--PAUSE-- C8:GO" : "C8:pause"), 16, 0);

    OLED_Refresh();
}

/* ==========================================================================
 * 第二部分：测试模式分发（TEST_MODE）
 * ========================================================================== */
#if TEST_MODE == 0
/* ==========================================================================
   不测试：立刻返回，后面交给 AGV_Init() + TaskProc() 走正式流程。
   下面整块实现用 #if 挡住，不然 TEST_MODE=0 时每个测试函数都没人引用，
   -Wall 会对每个函数报一条 -Wunused-function。
   注意：上面的调试服务不受这里影响，DBG_ENABLE=1 时正式流程照样有
   OLED 刷新和 C8 暂停。
   ========================================================================== */
void Test_Switch(void)
{
    return;
}

#else   /* TEST_MODE != 0 */

/* ========================== 测试函数声明 ========================== */
static void Test_AGV(void);          /* 1 跑一遍位置 */
static void Test_Correct(void);      /* 2 夹爪到观察位 + 0x02 位置修正 */
static void Test_Turn(void);         /* 3 航向PID：左转/右转 90° 往复(不返回) */
static void Test_Pick_Raw_PLUS(void);/* 4 原料 ABC → 车上 123(纯动作，不看任务码) */
static void Test_Pick_Place(void);   /* 5 车上 123 → 圆环 123(纯动作) */
static void Test_Pick_Raw(void);     /* 6 原料 B → 车上 123(走相机 0x02，不用任务码) */
static void Test_Camera(void);       /* 7 相机指令轮询(0x00~0x05 每5秒一条) */
static void Test_Motor(void);        /* 9 臂测试：高度/半径步进电机 + 转盘/Z轴舵机(都不返回) */
static void Test_Radar(void);        /* 10 雷达：一次方格扫描 + 四段路径规划，结果分页显示(不返回) */

static void Test_AGV(void);       /* 1 跑一遍位置 */
static void Test_Correct(void);   /* 2 夹爪到观察位 + 0x02 位置修正 */
static void Test_Pick_Scale(void);/* 3 夹爪开合死循环(调夹爪角度用) */
static void Test_Pick_Raw(void);  /* 4 原料 ABC → 车上 123(纯动作) */
static void Test_Pick_Place(void);/* 5 车上 123 → 圆环 123(纯动作) */
static void Test_Func(void);      /* 6 空壳,留给以后单步测带视觉的流程 */
static void Test_Camera(void);    /* 7 相机指令轮询(0x00~0x05 每5秒一条) */
static void Test_OLED(void);      /* 8 屏幕自检(甲段不带TIM7 / 乙段带TIM7，不返回) */
static void Test_Motor(void);     /* 9 臂测试：高度/半径步进电机 + 转盘/Z轴舵机(都不返回) */
static void Test_Radar(void);     /* 10 雷达：一次方格扫描 + 六段路径规划，结果分页显示(不返回) */
static void Test_Place_Ring(void);/* 11 POS_CAR → 圆环 1/2/3 简单放置(纯动作，不扫码不碰相机) */

static void Test_GrabRawToCar(uint8_t box, uint8_t slot);  /* 测试4 用的单步时序 */
static void Test_WaitStartKey(void);                       /* 跑测试前阻塞等 A8 短按 */


//    故意不调 Menu_Init()：那会在等 A8 按键时把测试卡在这里，而且测试本来就不需要菜单。
//    正式流程的 AGV_Init() 里才有。 */
static void Test_HwInit(void)
{
    OLED_Init();
#if TEST_MODE == 8
    /* 模式 8 测的就是"TIM7 刷屏把屏冲没了没有"，所以这里**绝对不能**先把 TIM7
       起来 —— 甲段要的正是一个没有 TIM7 的干净环境。TIM7 由 Test_OLED() 自己
       在乙段开头手工 Dbg_Init()。 */
#else
    Dbg_Init();           /* 开TIM7调试服务:周期刷OLED + C8短按暂停/继续(必须在OLED_Init之后) */
#endif
#if TEST_CAN_ENABLE
    AGV_Can_Init();       /* CAN 放这儿，不放各测试里 —— 两个原因：
                             1) C8 暂停在**任何**调试模式下都会生效，而暂停第一件事就是
                                调 AGV_Position_Stop() 往 CAN 上发四轮停车帧。哪个模式
                                漏了这句，暂停就会每帧空等到 EMM_V5_ACK_TMO_MS(200ms)
                                才返回，按下去要愣一下才停。
                             2) 剩下几个模式里除了模式7(只发串口)、模式8(只碰屏幕)
                                和模式1、2 之外的，本来也都要 CAN 驱动机械臂的步进电机。
                             里面自带 HAL_Delay(500) 等 CAN 稳定，放在最前面正好。
                             【TEST_CAN_ENABLE=0(台架没接电机)时整句编掉：CAN_MODE_NORMAL
                               下总线没第二个节点就同步不上，HAL_CAN_Start 超时返回
                               HAL_ERROR → Error_Handler() → while(1) 卡死。详见 test.h】 */
#else
    /* 台架没接步进电机：跳过 CAN。此时 Dbg_Service() 的停车也一起编掉了
       （见那边的 TEST_CAN_ENABLE），否则按 A8 会在 can_SendCmd() 的 while 上死循环。 */
#endif
    DWT_Init();           /* 初始化DWT,用于微秒级延时 —— pick/chassis 的 delay_us 靠它 */
    YawPID_Init();        /* 航向PID,增益见 yawPIDcontroller.h 的 YAW_PID_KP/KI/KD
                             测试1(走路线)和测试2(AGV_Correct)要用;后面几个纯臂测试用不上,
                             但它的开销只有几个赋值,没必要再分一次。 */

    /* ---- 雷达收放舵机：上电先复位到 340°(收起) ---- 
       **为什么必须放在这里**：`MX_TIM4_Init()`（main.c 里）只是把 TIM4_CH1=PD12 的 PWM
       配置好，**并没有启动输出**；测试模式的上电流程里也没有 `Servo_Init()`。
       不初始化的话，上电后雷达舵机**根本没有 PWM 信号**（松脱/保持未知位置），
       万一停在"伸出"位置，车一开就会撞。
       `Servo_InitId(SERVO_RADAR)` 只初始化**雷达这一路**（不碰机械臂三路），
       按 servo.c 的配置把初值定在 **340° = 收起**，并立刻起 PWM 让舵机锁在收起位。
       模式 1（Test_AGV）随后会用 Radar_SetDeployed(true) 再伸出去。 */
#if SERVO_R_INIT_AT_BOOT
    Servo_InitId(SERVO_RADAR);      /* 宏置 0 则上电先不动，仅第一次 Radar_SetDeployed() 时初始化 */
#endif
}

#define TEST_KEY_SCAN_MS   15U   /* 扫描周期，同 menu.c 的 KEY_SCAN_PERIOD_MS */
#define TEST_KEY_DEBOUNCE  3U    /* 连续同电平次数，同 menu.c 的 KEY_DEBOUNCE_CNT */

static void Test_WaitStartKey(void)
{
    uint8_t prev = 0U, cnt = 0U, stable = 0U, armed = 0U, raw;
    char    buf[20];

    Dbg_Suspend();                       /* 屏幕归这里，别被 200ms 的刷屏节拍盖掉 */

    OLED_Clear();
    sprintf(buf, "MODE %d", (int)TEST_MODE);
    OLED_ShowString(8, 16, (uint8_t *)buf,          16, 1);   /* 模式号：知道自己跑的哪个 */
    OLED_ShowString(8, 32, (uint8_t *)"press A8",   16, 1);
    OLED_ShowString(8, 48, (uint8_t *)"to start",   16, 1);
    OLED_Refresh();

    for(;;)
    {
        raw = (HAL_GPIO_ReadPin(START_KEY_GPIO, START_KEY_PIN) == GPIO_PIN_RESET) ? 1U : 0U;

        if(raw != prev) { prev = raw; cnt = 0U; }
        else
        {
            if(cnt < TEST_KEY_DEBOUNCE) cnt++;
            if(cnt >= TEST_KEY_DEBOUNCE && stable != raw)
            {
                stable = raw;
                if(raw == 0U)      armed = 1U;   /* 先见过"松开"，这次按下才算数 */
                else if(armed)     break;        /* 稳定为按下 → 启动 */
            }
        }
        HAL_Delay(TEST_KEY_SCAN_MS);         /* 挂起期间这句不会再刷屏(见上面说明) */
    }

    OLED_Clear();                        /* 清掉提示，别和测试自己的显示叠在一起 */
    Dbg_Resume();                        /* 交还屏幕，并补一次重画 */
}

/* ========================== 1 路径测试 ==========================
   **2026-10-14 改动**：前面先跑一遍雷达（**舵机伸出 → 一次 14 圈方格扫描 + 六段路径规划
   → 舵机收回车内**），然后**直接开跑**下面原有的路线测试（段间停 2s 看车走到哪了）。

   雷达收放舵机是第 4 路 `SERVO_RADAR`（TIM4_CH1 = PD12）：
   **0° = 雷达伸出、340° = 收入车内**（上电初值 340° 收起）。
   整个顺序：按 A8 → 雷达伸出（0°）→ 握手/选区/位姿/方格扫描/六段规划（屏上进度+摘要）
            → 雷达收回（340°）→ 直接开始跑车 → 各段依次执行。

   两点说明：
   1) 测试模式下 `AGV_Init()` 没被调用，所以雷达要在这里自己 `Radar_Init()`；
      `Radar_RunOnce()` 内部会 Dbg_Suspend/Resume，跑完把屏交还调试画面。
      收放舵机也由 `Radar_SetDeployed()` 自己初始化，**不会碰机械臂那三路**。
   2) **开跑用的是原有的固定路线函数**（AGV_Position_xxx），不是把雷达规划出的方格路径喂给底盘
      —— 那还需要"方格 → 场地坐标"的映射表（尚未定，见 改动记录 第 60 节）。
      想用规划结果驱车，接在 `Radar_RunOnce()` 之后调 AGV_Position_Go(x,y,dir) 即可。

   台架只验底盘时把 `TEST_AGV_WITH_RADAR` 置 0：跳过整段雷达（含伸出/收回舵机），
   免得先等雷达连接超时（8s）或没有雷达时白转一圈舵机。 */
#ifndef TEST_AGV_WITH_RADAR
#define TEST_AGV_WITH_RADAR  1
#endif

static void Test_AGV(void)
{
#if TEST_AGV_WITH_RADAR
    /* 雷达收放（舵机第 4 路 SERVO_RADAR = TIM4_CH1/PD12）：
       **0° = 伸出、340° = 收入车内**（角度在 radar_config.h 的 RADAR_SERVO_*）。
       按要求顺序：先伸出雷达 → 扫描 + 六段规划 → 收回车内 → 再开跑。
       Radar_SetDeployed() 内部先初始化这一路舵机（不碰机械臂三路），
       并阻塞到"转完 + 到位裕量"，所以返回时雷达确实到位了。 */
    Radar_SetDeployed(true);             /* 伸出（0°） */
    Radar_Init();                        /* 选默认启停区 + 位姿 + 路线（不发帧） */
    (void)Radar_RunOnce();               /* 握手→选区→设位姿→方格扫描→六段规划→STOP */
    Radar_SetDeployed(false);            /* 扫描/规划完，收入车内（340°） */
    HAL_Delay(500);                      /* 摘要再看 0.5s，然后直接开跑 */
#endif

    AGV_Position_Init();
    HWT101_Init(&huart5); /* 初始化HWT101 */

    /* 完整流程**跑两遍**（和 task.c 一致），按真实段序连成一条折线：
         第一遍：起点→2→14→10→22→14      (leg 0,1,2,3,4)
         第二遍：14→10→22→回出发点         (leg 2,3,5)
       每段 start == 上一段 goal，整条连续；所有拐点用 R=300mm 圆弧切过去，
       不经过拐点坐标、不停车原地转。生产流程里段末要干活所以会停，测试这里连续跑。 */
    static const uint8_t routeTwoLaps[] = { 0U, 1U, 2U, 3U, 4U, 2U, 3U, 5U };
    AGV_Position_CruiseThroughRoute(routeTwoLaps, (uint8_t)sizeof(routeTwoLaps), 3U);
}

/* ========================== 2 视觉修正测试 ==========================*/

static void Test_Correct(void)
{
    Cam_Init();           /* 等待相机启动，5秒超时，成功后oled显示参数，失败则不显示*/
    HWT101_Init(&huart5); /* 初始化HWT101 */

    Cam_SwitchTo(02);             /* 发 0x02 拿偏差，落到全局 dx/dy */
    AGV_Correct(20000);           /* 闭环到对准或 20s 超时，退出时自动停车 */
}

static void Test_Turn(void)
{
    HWT101_Init(&huart5);   /* 陀螺仪 + 零偏标定；转弯全靠它，CAN 由 Test_Switch 起好 */

    for(;;)
    {
        AGV_TurnLeft();     /* +90°：内部阻塞到对准并停车才返回 */
        AGV_TurnRight();    /* -90°：转回原朝向 */
        HAL_Delay(3000);    /* 歇 3s；DBG_ENABLE=1 时这句里也在刷屏、C8 也能暂停 */
    }
}



static void Test_GrabRawToCar(uint8_t box, uint8_t slot)
{
    switch(box)
    {
        case 0:  Pick_JumpTo POS_RAW_A; break;
        case 1:  Pick_JumpTo POS_RAW_B; break;
        default: Pick_JumpTo POS_RAW_C; break;
    }
    Pick_TurnTable_Set(slot);   /* 先选槽：转盘转动时臂还在原料区，不会碰臂 */
    Pick_Gripper_Grab();
    Pick_JumpTo POS_CAR;        /* 再摆臂：往车体上方去一律抬起来走 */
    Pick_Gripper_Release();
}

static void Test_Pick_Raw_PLUS(void)
{
    Pick_Init();          /* 机械臂上电+收臂到运输位(内部 Pick_JumpTo POS_START) */

    //Pick_Gripper_Release();   /* 先到已知爪位，免得第一次抓的时候夹爪卡在半途 */

    Test_GrabRawToCar(0, 0);  /* 原料 A 框 → 车上 1 槽(转盘   0°) */
    Test_GrabRawToCar(1, 1);  /* 原料 B 框 → 车上 2 槽(转盘 150°) */
    Test_GrabRawToCar(2, 2);  /* 原料 C 框 → 车上 3 槽(转盘 270°) */

    Pick_JumpTo POS_DEFAULT;  /* 摆臂到离开位，免得挡着相机 */
}



static void Test_Pick_Raw(void)
{
    Cam_Init();           /* 0x02 要它：等相机启动，5秒超时，成功后屏上显示参数 */
    Pick_Init();          /* 机械臂上电+收臂到运输位(内部 Pick_JumpTo POS_START) */           /* 等待相机启动，5秒超时，成功后oled显示参数，失败则不显示*/
    HWT101_Init(&huart5); /* 初始化HWT101 */
    
    QRcode_Task(); 
    Pick_Correct();
    Pick_RAW_PLUS();
    //Place_Correct();
    //Place_ROUGH();

    Pick_JumpTo POS_DEFAULT;  /* 摆臂到离开位，免得挡着相机 */
}


static void Test_Pick_Place(void)
{
    Pick_Init();          /* 机械臂上电+收臂到运输位(内部 Pick_JumpTo POS_START) */
    Place_Correct();
    Place_ROUGH();
    GRAB_AND_PLACE_TO_RING_SLOT(0, 1);   /* 车上 1 槽 → 圆环 1 号 */
    GRAB_AND_PLACE_TO_RING_SLOT(1, 2);   /* 车上 2 槽 → 圆环 2 号 */
    GRAB_AND_PLACE_TO_RING_SLOT(2, 3);   /* 车上 3 槽 → 圆环 3 号 */

    Pick_JumpTo POS_DEFAULT;  /* 摆臂到离开位，免得挡着相机 */
}

/* 模式 11：最简单的放置过程 —— 只验"车上取料位 → 三个圆环"这条动作链，
   不扫码、不碰相机、不做视觉修正。转盘固定在 A 槽位不动，所以三次都从
   同一个 POS_CAR 取料：
     循环 3 次：POS_CAR 取 → 夹紧 → 对应圆环(POS_1/2/3) → 放开
   ⚠ 前提：上电前高度/半径两轴已人工放在 0 位(Pick_Axis_Init 会
     把该位置写成绝对零点)，否则绝对坐标整体偏移。
   ⚠ 半径电机(地址 6)不回到位帧，Pick_JumpTo 只发目标不等回执；
     看动作是否到位以实物为准，不要以为没回帧就是没动。 */
static void Test_Place_Ring(void)
{
    Pick_Init();              /* 机械臂上电+收臂到运输位 */
    Pick_Gripper_Release();   /* 先到已知爪位，免得第一次抓时夹爪卡在半途 */

    for(uint8_t ring = 1U; ring <= 3U; ring++)
    {
        Pick_JumpTo POS_CAR;   /* 先回车上取料位 */
        Pick_Gripper_Grab();        /* 夹住物料 */
        HAL_Delay(300);             /* 停一下，肉眼确认夹稳了再走 */

        switch(ring)
        {
            case 1U:  Pick_JumpTo POS_1; break;
            case 2U:  Pick_JumpTo POS_2; break;
            default:  Pick_JumpTo POS_3; break;
        }

        Pick_Gripper_Release();     /* 放到圆环上 */
        HAL_Delay(300);             /* 等爪子完全张开，臂再抬走 */
    }

    Pick_JumpTo POS_RUN;       /* 跑完收到车体上方运输位 */
}



#define CAM_TEST_STEP_MS   5000U   /* 每条命令停留多久 */
#define CAM_TEST_TICK_MS    100U   /* 等待切片：每片插一次 Cam_Service() 做接收自愈 */

static void Test_Camera(void)
{
    /* 命令表：0x00~0x06 七条全发。
       带参数的只有 0x01(三色) 和 0x05(单色)，其余都是无参数帧。 */
    static const uint8_t cmd_tbl[7] = { 0x00U, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U };
    static const uint8_t plen_tbl[7] = { 0U, 3U, 0U, 0U, 0U, 1U, 0U };
    static const uint8_t arg_tbl[7][3] = {
        { 0U,   0U, 0U },   /* 0x00 空闲 —— 无参数 */
        { 1U,   2U, 3U },   /* 0x01 任务码 —— 三色填 1 2 3 */
        { 0U,   0U, 0U },   /* 0x02 最近物块 —— 无参数 */
        { 0U,   0U, 0U },   /* 0x03 三框颜色 —— 无参数 */
        { 0U,   0U, 0U },   /* 0x04 圆环 —— 无参数 */
        { 2U,   0U, 0U },   /* 0x05 指定颜色 —— 填颜色号 2，只用第一个字节 */
        { 0U,   0U, 0U },   /* 0x06 修正角度 —— 无参数 */
    };

    Cam_Init();           /* 等相机启动(5秒超时)，并起 DMA 接收 */

    for(;;)
    {
        for(uint8_t i = 0U; i < 7U; i++)
        {
            Cam_SendCommand(cmd_tbl[i], arg_tbl[i], plen_tbl[i]);

            /* 等 5 秒再看下一条。切成 100ms 一小段是为了中间插 Cam_Service()：
               串口一旦出错，HAL 会中止 DMA 接收且不会自动重挂，不补这一句的话
               一次错误就让 OLED 上的数据永远定住，看着像相机坏了。 */
            for(uint8_t t = 0U; t < (CAM_TEST_STEP_MS / CAM_TEST_TICK_MS); t++)
            {
                Cam_Service();
                HAL_Delay(CAM_TEST_TICK_MS);
            }
        }
    }
}


static void Test_Motor(void)
{
    Pick_Init();      /* 机械臂上电：起三个舵机(臂保持姿势) + 收臂到运输位。
                         不自己调 Servo_Init 是为了跟其他测试一个路子；它内部那句
                         Pick_JumpTo POS_START 只动高度(到 10mm)，半径目标 0mm
                         等于不动 —— 所以三个子测试都不受它干扰。
                         转盘子测试就靠这里起的舵机。 */
    

    /* 五个子测试一次只能跑一个 —— 换就换注释（同一时刻只留一行没注释）。
       判读方法见 pick.c 里各自函数头上那段。高度/半径现在都是绝对位置模式。 */
    Pick_Test_MotorZ();               /* 高度轴：死循环，不返回 */
    Pick_Test_MotorR();             /* 半径轴：把上面那行注释掉、这行打开 */
    Pick_Test_TurnTable();          /* 转盘舵机：A(30°)/B(150°)/C(270°)，不需要 CAN */
    Pick_Test_ZServo();             /* Z轴舵机：0°/100°/50°，不需要 CAN（臂先抬高）*/
    Pick_Test_Gripper();            /* 夹爪：观察/抓紧/放开 走 3 轮停在抓紧，不需要 CAN */

}

/* ========================== 10 雷达测试 ==========================
   只碰雷达一条链路：UART4(PC10=TX / PC11=RX, 115200) + OLED。
   相机、机械臂、陀螺仪都不初始化；CAN 仍由 Test_HwInit 统一初始化（雷达测试本身
   不用 CAN，那 500ms 是白等的，但测试初始化一律走同一套，不再分模式）。

   一次 A8 之后自动跑完整个流程（不返回）：
     Radar_Init()    —— 选默认启停区(RADAR_ZONE_DEFAULT) + 装该区的雷达位姿与出发点方格，不发帧
     Radar_RunOnce() —— 握手 → 选区 → 设位姿 → **一次 14 圈方格扫描** → 六段路径规划 → STOP
   跑完在屏上分页显示（按键：C6 = 下一页 / C8 = 上一页 / A8 = 重跑一次）：
     页0  障碍方格：5×5 图，"XX" = 有障碍、数字 = 空闲（方格扫描的直接结果）
     页1  位姿对照：雷达自报的位姿（= 上位机标定的那份）vs 本机 config 那份，
                    末行 diff 是两者朝向之差（归一到 ±180°）—— 扫出来格子不对先看这页
     页1  六段概览：标题 "legs n/6 f失败码"，下面每段一行 Lx 起点>终点 s状态 n拐点数
     页2~7 第 1~6 段的**逐格**路线（6x8 小字，方格编号，每行 7 个）
     （原"页1 位姿对照"已按要求去掉，见 改动记录 第 67 节）

   六段路线：起点(启停区出发点方格) → 2 → 14 → 10 → 22 → 14，第 6 段**从 22 回起点格**
   （非链式：第 5 段结束在 14，回程段却从 22 出发，见 radar_config.h 的 RADAR_RETURN_FROM_GRID）。
   段数只由 RADAR_LEG_COUNT（radar_task.h）与 LAUNCH_ROUTE_GOALS（radar_config.h）决定，
   本文件的分页/循环全部按这两个宏走，不写死段数。

   ⚠ 页k 为什么要"展开"：雷达回的**不是逐格序列**，而是"拐点 + 终点"
   （说明书 §7：直线路径只返回终点；移植示例 §6：不含起点、含拐点和终点）。
   所以一段 4→14 只回 1 个点（14）是**正常**的 —— 两点同列，本来就是一条直线。
   屏上先用 Radar_ExpandLeg() 按 §9 的方格几何展开成逐格，才看得清走哪几格；
   页k 第 2 行左边是雷达原样回的拐点数，右边是展开后的格数。

   判读：s0 成功；s1 无有效方格、s2 起点被占用、s3 终点被占用、s4 不可达、s5 参数、
   s6 容量超限。整页 "Lx --" 表示这段没跑到（前面就失败了，看第 3 行 f 码：
   f1 连接 / f2 位姿（多半是 radar_config.h 的"待填"值没填或越界） / f3 方格扫描 /
   f4 读保存状态 / f5 雷达里没有有效方格 / f6 路径 / f7 雷达拒绝或协议错）。

   结果数据本体在全局数组 Radar_Path[5]（见 radar_task.h），本函数只负责显示；
   要拿数据直接用 Radar_Path[i].points[] / .count / .status（那是雷达原样的拐点），
   要逐格路线用 Radar_ExpandLeg(段号, 缓冲, 容量)。

   注意：C8 在这里是"上一页"，而 TIM7 中断里 C8 还是全局暂停键 —— 按下会顺带
   悄悄翻一次 s_paused（test.h 第二节记过这条），测试模式下没有动作，无副作用。 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       prev;
    uint8_t       cnt;
    uint8_t       stable;
} RadarTestKey;

/* 消抖后的"按下沿"，逻辑同 menu.c 的 Key_Scan（15ms 一拍 + 连续 3 次同电平） */
static uint8_t RadarTest_KeyEdge(RadarTestKey *k)
{
    uint8_t raw = (HAL_GPIO_ReadPin(k->port, k->pin) == GPIO_PIN_RESET) ? 1U : 0U;
    uint8_t press = 0U;

    if(raw == k->prev)
    {
        if(k->cnt < TEST_KEY_DEBOUNCE) k->cnt++;
        if(k->cnt >= TEST_KEY_DEBOUNCE && k->stable != raw)
        {
            k->stable = raw;
            if(raw == 1U) press = 1U;
        }
    }
    else
    {
        k->prev = raw;
        k->cnt = 0U;
    }
    return press;
}

/* 页2：六段概览。6 段用 6x8 小字每行一段（16 像素字体一屏只能放 4 行，放不下 6 段）；
   标题行给出实际拿到的段数与失败码，一眼看出"是不是 5 段都成了"。 */
static void RadarTest_ShowOverview(void)
{
    char buf[32];                 /* 最坏 "L5 24>24 s6 n24" 约 16 字符，留够余量给 -Wformat-overflow */
    uint8_t i;

    OLED_Clear();
    sprintf(buf, "legs %u/%u f%u", (unsigned)Radar_PathCount,
            (unsigned)RADAR_LEG_COUNT, (unsigned)Radar_GetFailure());
    OLED_ShowString(0, 0, (uint8_t *)buf, 16, 1);

    for(i = 0U; i < (uint8_t)RADAR_LEG_COUNT; i++)
    {
        if(Radar_Path[i].status == RADAR_PATH_INVALID)
            sprintf(buf, "L%u --", (unsigned)(i + 1U));
        else
            sprintf(buf, "L%u %u>%u s%u n%u", (unsigned)(i + 1U),
                    (unsigned)Radar_Path[i].start, (unsigned)Radar_Path[i].goal,
                    (unsigned)Radar_Path[i].status, (unsigned)Radar_Path[i].count);
        OLED_ShowString(0, (uint8_t)(16U + i * 8U), (uint8_t *)buf, 8, 1);
    }
    OLED_Refresh();
}

/* 页k(1..5)：第 k 段。雷达只回**拐点+终点**（直线段只回终点），这里展开成逐格编号显示 —— 
   否则一条直线段只显示 1 个点，看着像丢数据（其实是协议设计如此，见说明书 §7/§9）。 */
static void RadarTest_ShowLeg(uint8_t page)
{
    char buf[32];
    uint8_t idx = (uint8_t)(page - 1U);
    uint8_t cells[RADAR_PATH_MAX_POINTS];
    uint8_t n;
    uint8_t i;

    OLED_Clear();
    sprintf(buf, "L%u %u>%u s%u", (unsigned)page, (unsigned)Radar_Path[idx].start,
            (unsigned)Radar_Path[idx].goal, (unsigned)Radar_Path[idx].status);
    OLED_ShowString(0, 0, (uint8_t *)buf, 16, 1);

    n = Radar_ExpandLeg(idx, cells, (uint8_t)sizeof(cells));

    if(Radar_Path[idx].status != 0U)
    {
        sprintf(buf, "no path n=%u", (unsigned)Radar_Path[idx].count);
        OLED_ShowString(0, 24, (uint8_t *)buf, 16, 1);
    }
    else if(n == 0U)
    {
        /* 理论上不该发生：要么起点=终点，要么折点不共行不共列 */
        sprintf(buf, "raw %u expand FAIL", (unsigned)Radar_Path[idx].count);
        OLED_ShowString(0, 16, (uint8_t *)buf, 8, 1);
    }
    else
    {
        sprintf(buf, "raw %u -> %u cells", (unsigned)Radar_Path[idx].count, (unsigned)n);
        OLED_ShowString(0, 16, (uint8_t *)buf, 8, 1);

        /* 逐格编号：每行 7 个（7×18=126 像素）；y=24/32/40/48 四行，够放 24 格 */
        for(i = 0U; i < n && i < 28U; i++)
        {
            OLED_ShowNum((uint8_t)((i % 7U) * 18U), (uint8_t)(24U + (i / 7U) * 8U),
                         (u32)cells[i], 2U, 8U, 1);
        }
    }

    OLED_ShowString(0, 56, (uint8_t *)"C6> C8< A8:run", 8, 1);
    OLED_Refresh();
}

/* 页0：障碍方格 5×5 图（方格扫描的直接结果，也就是路径规划的输入）。
   "XX" = 该格有障碍；数字 = 空闲格。按说明书 §9 的摆法画：行4 在上、行0 在下，
   和手册那张图方向一致（若现场 Y 轴方向相反，把 (4U - row) 改成 row 即可）。 */
static void RadarTest_ShowObstacles(void)
{
    char buf[24];
    uint8_t id;

    OLED_Clear();
    if(Radar_GetFailure() != 0U)
        sprintf(buf, "obs n=%u f%u", (unsigned)Radar_GetObstacleCount(),
                (unsigned)Radar_GetFailure());
    else
        sprintf(buf, "obs grid n=%u", (unsigned)Radar_GetObstacleCount());
    OLED_ShowString(0, 0, (uint8_t *)buf, 16, 1);

    for(id = 0U; id <= 24U; id++)
    {
        uint8_t x = (uint8_t)(4U + Radar_GridCol(id) * 20U);
        uint8_t y = (uint8_t)(16U + (4U - Radar_GridRow(id)) * 8U);

        if(Radar_GridIsObstacle(id))
            OLED_ShowString(x, y, (uint8_t *)"XX", 8, 1);
        else
            OLED_ShowNum(x, y, (u32)id, 2U, 8U, 1);
    }

    OLED_ShowString(0, 56, (uint8_t *)"C6> C8< A8:run", 8, 1);
    OLED_Refresh();
}

/* 页0 障碍方格 / 页1 六段概览 / 页2..7 第 1..6 段逐格路线。
   （原来的"位姿对照"页已按要求去掉，见 改动记录 第 67 节；位姿仍可读
     Radar_GetRadarPose() / Radar_GetConfigPose()，只是不再上屏。）
   段数变化时这里全自动跟着变（只改了 RADAR_LEG_COUNT 与 LAUNCH_ROUTE_GOALS 两处）。 */
#define RADAR_TEST_PAGES  (2U + (uint8_t)RADAR_LEG_COUNT)

static void RadarTest_ShowPage(uint8_t page)
{
    if(page == 0U)      RadarTest_ShowObstacles();
    else if(page == 1U) RadarTest_ShowOverview();
    else                RadarTest_ShowLeg((uint8_t)(page - 1U));
}

static void Test_Radar(void)
{
    RadarTestKey key_next = { GPIOC, GPIO_PIN_6, 0U, 0U, 0U };   /* C6 下一页 */
    RadarTestKey key_prev = { GPIOC, GPIO_PIN_8, 0U, 0U, 0U };   /* C8 上一页 */
    RadarTestKey key_run  = { GPIOA, GPIO_PIN_8, 0U, 0U, 0U };   /* A8 重跑 */

    for(;;)
    {
        uint8_t page = 0U;
        uint8_t rerun = 0U;

        Radar_Init();               /* 每次重跑都重新初始化（Launch_IsFinished 是粘滞的） */
        (void)Radar_RunOnce();      /* 进度页 + 六段摘要页由它自己显示（内部管 Dbg 挂起） */

        Dbg_Suspend();              /* 结果页归本函数，直到下次重跑 */
        do
        {
            RadarTest_ShowPage(page);   /* 页0 障碍方格 → 页1 概览 → 页2..7 六段逐格 */

            for(;;)
            {
                if(RadarTest_KeyEdge(&key_run))  { rerun = 1U; break; }
                if(RadarTest_KeyEdge(&key_next))
                {
                    page = (uint8_t)((page + 1U) % RADAR_TEST_PAGES);
                    break;
                }
                if(RadarTest_KeyEdge(&key_prev))
                {
                    page = (page == 0U) ? (uint8_t)(RADAR_TEST_PAGES - 1U) : (uint8_t)(page - 1U);
                    break;
                }
                HAL_Delay(TEST_KEY_SCAN_MS);   /* 挂起期间这句不会再刷屏 */
            }
        } while(rerun == 0U);

        Dbg_Resume();               /* 交还屏幕，然后重跑 */
    }
}

/* ========================== 分发入口 ==========================
   main() 在 AGV_Init() 之前调它。测试模式下不返回。 */
void Test_Switch(void)
{
    OLED_Init();
    Dbg_Init();          
    AGV_Can_Init();                    
    DWT_Init();         
    YawPID_Init(); 

    Test_WaitStartKey();
    Pick_Axis_Init();
    switch(1)
    {
        case 1:  Test_AGV();         break;  //跑一遍位置
        case 2:  Test_Correct();     break;  //夹爪到观察位置 发送0x02 位置修正
        case 3:  Test_Turn();        break;  //转向PID：左转90/右转90往复，增益在 yawPIDcontroller.h 的 KP/KI/KD
        case 4:  Test_Pick_Raw_PLUS();    break;  //依次抓取ABC到车上123位置(不看任务码、不碰相机，纯动作)
        case 5:  Test_Pick_Place();  break; //    依次抓取车上123位置到圆环123位置
        case 6:  Test_Pick_Raw();        break;   //依次抓取B到车上123位置(走相机：0x04 等到静止的块才抓)
        case 7:  Test_Camera();      break;  //定时器自带oled显示接收数据 ，然后每隔5秒轮流发送所有模式指令 比如任务0x04的可以就123，0x05就用颜色数字2
        case 8:  QRcode_Task();       break;  //调用任意的测试函数
        case 9:  Test_Motor();       break;  //臂测试：高度/半径步进电机(0<->30mm往返)或转盘舵机(A/B/C槽)，靠看+尺子验方向和比例(不返回)
        case 10: Test_Radar();       break;  //雷达：一次方格扫描+六段路径规划，结果分页显示；C6/C8 翻页，A8 重跑(不返回)
        case 11: Test_Place_Ring();  break;  //简单放置：POS_CAR 取料 → 圆环 1/2/3 依次放(纯动作，不扫码不碰相机)
        default: break;                      /* 号还没写实现的：初始化完就停住 */
    }

    for(;;) { } 
}

#endif  /* TEST_MODE == 0 */
