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
#include "stm32f4xx_hal.h"
#include "task.h"                   /* 正式流程状态机 + AGV_Position_* */
#include "pick_task.h"              /* Pick_RAW / Place_ROUGH / Place_TEMP */
#include "pick.h"                   /* Pick_JumpTo / POS_* / Pick_Gripper_* */
#include "camera.h"                 /* cam_mode/cam_flag/dx/dy + Cam_Init/Cam_SwitchTo */
#include "positionPIDcontroller.h"  /* AGV_Correct / AGV_Position_Stop */
#include "QRcode.h"                 /* Pick_Color1[3] 等 12 位任务码 */
#include "oled.h"                   /* OLED_Clear / OLED_ShowString / OLED_Refresh */
#include <stdint.h>
#include <stdio.h>                  /* sprintf */

/* ==========================================================================
 * 第一部分：运行时调试服务（DBG_ENABLE=1 才有）
 * ========================================================================== */

/* --- 两个键，各管各的（2026-10-14 拆开）---
   START_KEY_* = A8：**启动键**。menu.c 的菜单和本文件的 Test_WaitStartKey()
                 都等它，只负责"开始跑"。
   DBG_KEY_*   = C8：**暂停键**，只在运行时的 TIM7 中断里扫（HAL_TIM_PeriodElapsedCallback）。
   以前是同一个 A8 兼两个角色，一按就既"启动"又"暂停"，启动瞬间就打架。

   **两个都故意放在 #if DBG_ENABLE 外面**：START_KEY_* 要给
   Test_WaitStartKey() 用，那段在 DBG_ENABLE=0 时照样编进去；塞进开关里面的话，
   -DDBG_ENABLE=0 会报 "START_KEY_GPIO undeclared"。DBG_KEY_* 放外面只是跟着
   一起挪，方便对照着看。 */
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

/**
 * @brief 挂起调试刷屏：屏幕交给别的模块用（只有 Menu_Init 会调）
 * @note  为什么需要：AGV_Init() 是 OLED_Init → Dbg_Init → ... → Menu_Init，
 *        菜单起来时 TIM7 已经在跑了，而 Menu_Init 里那个 HAL_Delay(15) 被本
 *        文件盖掉、内部会调 Dbg_Service() —— 200ms 一到就把菜单整个刷成相机
 *        数据画面，而 Menu_Display() 只在首次和按 C6 时重画，被刷掉就再也
 *        不回来（现象：菜单闪一下消失、看不到 Route）。
 *        挂起期间只是不刷屏，TIM7 照旧扫键、暂停照旧被挡在外面（见下面那句
 *        s_suspended 的判断）—— 菜单期间 A8 是启动键，C8 是"还没跑起来"的
 *        暂停键，两个都不该在菜单里起作用。
 *
 *        ⚠ 注意挡的是"按下去当场生效"，**不是"拦住 s_paused 被翻转"**：
 *        中断里那道 C8 消抖扫描并没有看 s_suspended（见 HAL_TIM_PeriodElapsedCallback
 *        的开头），所以挂起期间按 C8 照样会把 s_paused 翻成 1，只是 Dbg_Service()
 *        整个让路、当场看不出来。等 Dbg_Resume() 之后，第一个服务点就会
 *        立刻进暂停画面 —— 表现成"菜单一退出车就停住不动"。Menu_Init() 里
 *        A8/C6 与 C8 分家（见 menu.c 文件头）已经把最容易碰到的那次堵掉了，
 *        但真正稳妥的做法是在中断的扫键段前面也加一句 s_suspended 判断。
 */
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
        s_suspended（启动菜单占着屏幕）时整个服务让路：既不刷屏也不响应暂停。
 */
void Dbg_Service(void)
{
    if(s_in_service || s_suspended || __get_IPSR() != 0U) return;
    s_in_service = 1;

    if(s_paused)
    {
        /* 步进电机收到的是"走 Xmm @ Yrpm"，轨迹是电机自己跑的 —— MCU 冻住
           车照走。所以暂停必须显式发一条停车。 */
#if TEST_CAN_ENABLE
        AGV_Position_Stop();
#endif
        /* TEST_CAN_ENABLE=0（台架没接电机）时上面那句不发。**必须和 Test_HwInit()
           里的 AGV_Can_Init() 同进同出**：CAN 没启动时 AGV_Position_Stop() 会走到
           can_SendCmd() 里 `while(HAL_CAN_AddTxMessage(...) != HAL_OK);` —— CAN 没
           Start 过，这句恒不成立，按一次 C8 就死循环，再按也救不回来（已经不在
           下面这个 while 里了）。没接电机本来也没东西可停。 */

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
static void Test_HwInit(void);    /* 所有测试共用的那一小撮初始化 */

static void Test_AGV(void);       /* 1 跑一遍位置 */
static void Test_Correct(void);   /* 2 夹爪到观察位 + 0x02 位置修正 */
static void Test_Pick_Scale(void);/* 3 夹爪开合死循环(调夹爪角度用) */
static void Test_Pick_Raw(void);  /* 4 原料 ABC → 车上 123(纯动作) */
static void Test_Pick_Place(void);/* 5 车上 123 → 圆环 123(纯动作) */
static void Test_Func(void);      /* 6 空壳,留给以后单步测带视觉的流程 */
static void Test_Camera(void);    /* 7 相机指令轮询(0x00~0x05 每5秒一条) */
static void Test_OLED(void);      /* 8 屏幕自检(甲段不带TIM7 / 乙段带TIM7，不返回) */
static void Test_Motor(void);     /* 9 臂测试：高度/半径步进电机 + 转盘/Z轴舵机(都不返回) */

static void Test_GrabRawToCar(uint8_t box, uint8_t slot);  /* 测试4 用的单步时序 */
static void Test_WaitStartKey(void);                       /* 跑测试前阻塞等 A8 短按 */

/* ========================== 测试共用的初始化 ==========================
   只有**每个测试都要**的才放这里。其余外设由各测试函数在自己开头按需初始化
   （见下面每个函数前几行）：只测臂的不用白等相机、只测路径的不用初始化机械臂。

   故意不调 Menu_Init()：那会在等 A8 按键时把测试卡在这里，而且测试本来就不需要菜单。
   正式流程的 AGV_Init() 里才有。 */
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
}

/* ========================== 一键启动等待 ==========================
   跑任何测试之前先在这儿停住，等 A8 短按才放行。目的：手上还没准备好时（还没
   把臂放到行程中段、还没把车摆正、尺子还没拿）不用赶着上电，上完电从容按一下
   再跑。

   放在 Test_HwInit() **之后**：那会儿 OLED 已经亮着，能显示出模式号和提示；
   CAN 也已经起完（自带 500ms 等稳定）。早于 Test_HwInit 的话屏是黑的，只能盲按。

   **按键就是 menu.c 里的启动键 A8**（`START_KEY_GPIO`/`START_KEY_PIN` =
   GPIOA PIN_8，上拉输入，按下=低）。
   `MX_GPIO_Init()` 在 main() 里就把它配好了（早于 Test_Switch），这里不用再初始化。
   **暂停键是 C8，跟这里无关** —— 2026-10-14 两个键分家之后，等待期间按 C8
   除了让中断里的 s_paused 悄悄翻一次之外没有任何可见效果，也不会再抢 A8。

   两个和 Menu_Init() 一样必须做的事：
     1) **Dbg_Suspend()/Dbg_Resume() 包住**。DBG_ENABLE=1 时 TIM7 早跑起来了
        （Test_HwInit 里起的）。不挂起的话：本函数里的 HAL_Delay 被本文件盖掉、
        内部会调 Dbg_Service()，200ms 一到就把"press A8 / to start"刷成相机数据
        画面，提示消失、也不再回来。
        （以前 A8 兼暂停键时这里还多一条理由"按下去会先被当成暂停"，现在分成
        两个键了，那条不成立了；挂起照样必须留着，为的是上面刷屏那条。）
        挂起期间 Dbg_Service 整个让路（见其开头那句 s_suspended 判断）。
        DBG_ENABLE=0 时这两个是空语句，编出来不占空间。
     2) **消抖 + "先松后按"**：15ms 一扫、连续 3 次同电平算稳定（≈45ms），和
        menu.c 的 Key_Scan 同一套。多加的 armed 是防"上电时就按着 A8" ——
        menu.c 那时会立刻启动，这里不行：开机瞬间就按着的键不算数，得先松手再按，
        否则等于没有这道关卡。 */
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
   把整条路线的每一段依次跑一遍，段间停 2s 方便看车走到哪了。
   原 task.c 里 STANDALONE_TEST==1 那段，原样搬过来。 */
static void Test_AGV(void)
{
    HWT101_Init(&huart5); /* 初始化HWT101 */

    AGV_Position_Depart_1();       HAL_Delay(2000);
    AGV_Position_Qcode_to_Raw();   HAL_Delay(2000);
    AGV_Position_Raw_to_Rough();   HAL_Delay(2000);
    AGV_Position_Rough_to_Temp();  HAL_Delay(2000);
    AGV_Position_Temp_to_Raw();    HAL_Delay(2000);
}

/* ========================== 2 视觉修正测试 ==========================
   只测「问最近物块 → 闭环对齐」这一环，不取放物料。
   和正式流程一样：夹爪先张开、臂摆到取料位，免得挡着相机；然后请 0x02
   拿偏差（落到全局 dx/dy），交给 AGV_Correct() 闭环。
   拿不到偏差就不修正 —— 不然会拿上一帧的旧 dx/dy 把车带偏。

   ⚠ 0x02 的语义 2026-10-16 改过：现在报的是"最近物块"的位置偏差与边沿判断，
     它的 flag 就是那个物块的颜色号(1~6)，不再是"圆盘中心、flag 只有 0/1"。
     正式流程那条路的位姿是 POS_RAW_B（见 pick_task.c 的 Pick_Correct），
     本测试下面用的是 POS_OBSERVE_RAW —— 想跟正式流程完全一致就改成它。 */
static void Test_Correct(void)
{
    Pick_Init();          /* 机械臂上电+收臂到运输位(内部 Pick_JumpTo POS_START) */
    Cam_Init();           /* 等待相机启动，5秒超时，成功后oled显示参数，失败则不显示*/
    HWT101_Init(&huart5); /* 初始化HWT101 */

    Pick_JumpTo POS_OBSERVE_RAW;  /* 臂摆到圆盘观察位 */
    Pick_Gripper_Observe();          /* 夹爪张开，别挡住观察位 */

    // if(Cam_SwitchTo(02))
      AGV_Correct(20000);           /* 闭环到对准或 5s 超时，退出时自动停车 */
 
    Place_ROUGH();
    HAL_Delay(2000);
}

/* ========================== 3 夹爪开合测试 ==========================
   臂停在运输位 POS_RUN 不动，只让夹爪在三个状态之间循环(闭合→放开→张开)，
   **死循环不出**。

   用途：对着看夹爪转到位没有、会不会顶到料。要改角度就调 pick.h 里的
   PICK_GRAB / PICK_RELEASE / PICK_OBSERVE，烧一次就能一直看，不用反复上电。
   所以这个函数永远不返回 —— 外面 Test_Switch() 的 for(;;) 到不了，是有意的。 */
static void Test_Pick_Scale(void)
{
    Pick_Init();          /* 机械臂上电+收臂到运输位(内部 Pick_JumpTo POS_START) */

    Pick_JumpTo POS_RUN;  /* 摆到运输位，让开视线 */

    for(;;)
    {
        Pick_Gripper_Grab();     HAL_Delay(3000);   /* 闭合：对着看 PICK_GRAB 角度 */
        Pick_Gripper_Release();  HAL_Delay(3000);   /* 释放：对着看 PICK_RELEASE 角度 */
        Pick_Gripper_Observe();  HAL_Delay(3000);   /* 张开：对着看 PICK_OBSERVE 角度 */
    }
}

/* ========================== 4 用的单步时序 ==========================
   原料框 box → 车上槽 slot，纯动作。

   照 pick_task.c 的静态函数 GrabRawBoxToCar() 抄的，两处差别：
     * 那边 box 是 0/1/2 推算出来的动态值，这里测试固定传 0/1/2；
     * 那边有个 direct 参数分"门字形/直接下"两版走法，这里只用门字形
       （安全、能跨过料台上的东西，桌面调试不需要抢那点时间）。
   改 pick_task.c 那段时序时记得同步这里 —— 测试比的就是它。 */
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

/* ========================== 4 原料区抓取测试 ==========================
   原料 A/B/C 三个框 → 车上 1/2/3 三个槽，**纯动作、不碰相机、不看任务码**。

   和正式流程 Pick_RAW() 的区别：那个要发 0x02 对正圆盘、发 0x03 问三框颜色，
   再按任务码决定哪一格去哪一槽；这里全跳过，固定 A→1、B→2、C→3，
   就为了单独看臂的走位对不对、转盘转角对不对。

   前提：车已经停在原料区，且摆位大致对得上 —— 没做视觉对正。 */
static void Test_Pick_Raw(void)
{
    Pick_Init();          /* 机械臂上电+收臂到运输位(内部 Pick_JumpTo POS_START) */

    Pick_Gripper_Release();   /* 先到已知爪位，免得第一次抓的时候夹爪卡在半途 */

    Test_GrabRawToCar(0, 0);  /* 原料 A 框 → 车上 1 槽(转盘   0°) */
    Test_GrabRawToCar(1, 1);  /* 原料 B 框 → 车上 2 槽(转盘 150°) */
    Test_GrabRawToCar(2, 2);  /* 原料 C 框 → 车上 3 槽(转盘 270°) */

    Pick_JumpTo POS_DEFAULT;  /* 摆臂到离开位，免得挡着相机 */
}

/* ========================== 5 车上取料放圆环测试 ==========================
   车上 1/2/3 三个槽 → 圆环 1/2/3 号位，**纯动作、不碰相机、不看任务码**。

   用的宏是 pick_task.h 里现成的 GRAB_AND_PLACE_TO_RING_SLOT() —— 它展开出来
   就是 pick_task.c 那个静态函数 PlaceToRing()，和正式流程 Place_ROUGH() 走的是
   同一段时序，只是环号写死 1/2/3（正式流程是拿 Place_Color[i] 当环号）。

   前提：三个槽里已经放好了料（先跑测试 4 就行），且车停在圆环区、摆位大致对得上。 */
static void Test_Pick_Place(void)
{
    Pick_Init();          /* 机械臂上电+收臂到运输位(内部 Pick_JumpTo POS_START) */

    GRAB_AND_PLACE_TO_RING_SLOT(0, 1);   /* 车上 1 槽 → 圆环 1 号 */
    GRAB_AND_PLACE_TO_RING_SLOT(1, 2);   /* 车上 2 槽 → 圆环 2 号 */
    GRAB_AND_PLACE_TO_RING_SLOT(2, 3);   /* 车上 3 槽 → 圆环 3 号 */

    Pick_JumpTo POS_DEFAULT;  /* 摆臂到离开位，免得挡着相机 */
}

/* ========================== 6 功能测试（空壳） ==========================
   留着给带视觉的完整流程做单步验证（Pick_RAW / Place_ROUGH / Pick_ROUGH /
   Place_TEMP 这些）。现在只起硬件就停住，等你想测什么往里加。

   往里加的时候注意：这几个函数都依赖任务码数组 Pick_Color1/2、Place_Color1/2，
   不先跑一次 QRcode_Task() 灌进去的话，Pick_Color_List() 读到的全是 0，
   颜色匹配永远不中，表现是"臂动了两下就收工"。想绕开扫码就得自己先把
   这几个数组填死。第二轮那几个还要先把 task.c 的 RerunFlag 置 1。 */
static void Test_Func(void)
{
    Pick_Init();          /* 机械臂上电+收臂到运输位(内部 Pick_JumpTo POS_START) */
    /* 往里加测试内容 */
}

/* ========================== 7 相机指令轮询测试 ==========================
   每 5 秒换一条命令，0x00~0x05 六条循环发，OLED 上看相机回什么。
   屏上内容是 Dbg_Service 在刷（还是原来那四行，本测试没改屏），怎么读：

     C:xx   回帧的命令码 —— **这是本测试的主要判据**。每 5 秒换一条发，
            正常情况下它应该跟着变；一直停在某个值不动 = 相机不认新命令。
     F:x    flag —— 0x02 时是最近物块的颜色号(1~6)、0=没识别到或还没静止；
            0x04 时就是圆环号(1/2/3)；
            0x01/0x03 时 1=干完了。
     x: y:  dx/dy 偏差(px) —— 只有 0x02 / 0x04 / 0x05 的帧会更新，别的命令时是旧值。

   **为什么不用 Cam_SwitchTo()**：那个是阻塞的，第一段等相机回本命令固定 10s、
   第二段（CAM_REQ_TMO_MS）又 20s，相机没接时一条命令卡 30 秒，"每 5 秒换一条"根本做不到，
   而且卡住期间 cam_mode 不刷新、屏看起来像死机。这里用 Cam_SendCommand()
   只发不等（几十微秒），相机会自己持续回发当前模式的数据。

   注意 0x00 是让相机 sleep 的那条，每轮都会把它打回空闲 —— 这是有意的，
   正好能看出来"相机被唤醒之后还会不会继续回"。 */
#define CAM_TEST_STEP_MS   5000U   /* 每条命令停留多久 */
#define CAM_TEST_TICK_MS    100U   /* 等待切片：每片插一次 Cam_Service() 做接收自愈 */

static void Test_Camera(void)
{
    /* 命令表：0x00~0x05 六条全发。
       带参数的只有 0x01(三色) 和 0x05(单色)，其余都是无参数帧。 */
    static const uint8_t cmd_tbl[6] = { 0x00U, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U };
    static const uint8_t plen_tbl[6] = { 0U, 3U, 0U, 0U, 0U, 1U };
    static const uint8_t arg_tbl[6][3] = {
        { 0U,   0U, 0U },   /* 0x00 空闲 —— 无参数 */
        { 1U,   2U, 3U },   /* 0x01 任务码 —— 三色填 1 2 3 */
        { 0U,   0U, 0U },   /* 0x02 圆盘中心 —— 无参数 */
        { 0U,   0U, 0U },   /* 0x03 三框颜色 —— 无参数 */
        { 0U,   0U, 0U },   /* 0x04 圆环 —— 无参数 */
        { 2U,   0U, 0U },   /* 0x05 指定颜色 —— 填颜色号 2，只用第一个字节 */
    };

    Cam_Init();           /* 等相机启动(5秒超时)，并起 DMA 接收 */

    for(;;)
    {
        for(uint8_t i = 0U; i < 6U; i++)
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

/* ========================== 8 屏幕自检 ==========================
   用来分开两件完全不同的事：**屏/接线坏了**，和**TIM7 刷屏把屏冲没了**。
   分甲乙两段，段首都在屏上打出标记字，一眼能看出卡在哪一段。

     甲段：TIM7 **不启动**（Test_HwInit 里对模式 8 特意跳过了 Dbg_Init），
           全是手写画面 —— 先全黑↔全白闪 3 下（最直观的"屏到底活没活"），
           再打 4 行标记字停 5 秒。
             * 黑白闪了、字也出来了 → 屏本身、软件 I2C、PB6/PB7 引脚全好，
               问题在乙段，也就是 TIM7 / 调试服务那条路。
             * 一点动静都没有 → 跟 TIM7 无关，去查硬件：屏的 VCC/GND、
               SCL(PB6)/SDA(PB7) 接反没接反、模块是不是 3.3V 的、上拉电阻。

     乙段：手工调 Dbg_Init() 把 TIM7 起起来（和正式流程一模一样），先写一个
           "B1" 标记屏，然后 HAL_Delay(5s)。这 5 秒里 Dbg_Service() 每 200ms
           会整屏覆盖成调试画面（C:xx F:x / x: y: / 任务码 / C8:pause）。
             * 5 秒后屏上是调试画面 → 整条 TIM7 刷屏链路是好的。
             * 还停在自己写的 "B1" → TIM7 中断压根没进来（时钟没使能 /
               HAL_TIM_Base_Start_IT 没成）。
             * 变成全黑或雪花 → 刷屏这条路有问题。多半是在 TIM7 里动软件 I2C
               被打断（CAN / 串口接收中断优先级比它低会被压住），先把 test.h 的
               DBG_REFRESH_MS 调大、或者把 TIM7 的 NVIC 优先级调低试试。

   最后死循环来回切"自己写的画面 ↔ Dbg_ShowScreen"，方便一直盯着对比，
   顺便看长时间跑会不会自己坏掉。所以这个函数和模式 3 一样永不返回。

   **开头必须先 DWT_Init()**：oled.c 是软件 I2C，每一位的时序靠 delay_us(1)
   撑着，而 delay_us 读的是 DWT->CYCCNT。DWT 没初始化时 usTick==0，
   delayTicks 算出来是 0，那个 while 一次都不转 —— 等于没有延时，SCL/SDA 会
   飙到几 MHz，SSD1306 收不到就整屏不亮。而 Test_HwInit() 里 OLED_Init() 排在
   DWT_Init() **前面**，所以这里再初始化一次屏（这次 DWT 肯定已经起来了），
   把这个初值顺序的变量彻底排除掉。DWT_Init() 幂等，重复调没有副作用。 */
static void OLED_TestFill(uint8_t on)
{
    uint8_t x, y;

    for(y = 0U; y < 64U; y++)
    {
        for(x = 0U; x < 128U; x++)
        {
            OLED_DrawPoint(x, y, on);   /* 只动显存，下面统一刷一次 */
        }
    }
    OLED_Refresh();                     /* 整屏一次，约 30ms */
}

static void Test_OLED(void)
{
    /* ---------------- 甲段：TIM7 关着，纯手写画面 ---------------- */
    DWT_Init();      /* 幂等，见函数头说明：确保后面的 delay_us 真的在延时 */
    OLED_Init();     /* 再初始化一次，这次 DWT 已经在跑，I2C 时序是准的 */

    for(uint8_t i = 0U; i < 3U; i++)    /* 全黑 ↔ 全白，闪 3 下 */
    {
        OLED_TestFill(0);
        HAL_Delay(500);
        OLED_TestFill(1);
        HAL_Delay(500);
    }

    OLED_Clear();
    OLED_ShowString(0,  0, (uint8_t *)"A1 OLED OK", 16, 1);
    OLED_ShowString(0, 16, (uint8_t *)"A2 no TIM7", 16, 1);
    OLED_ShowString(0, 32, (uint8_t *)"A3 3 blinks", 16, 1);
    OLED_ShowString(0, 48, (uint8_t *)"A4 DWT on", 16, 1);
    OLED_Refresh();
    HAL_Delay(5000);

    /* ---------------- 乙段：起 TIM7，走调试服务那条路 ---------------- */
#if DBG_ENABLE
    Dbg_Init();      /* 和正式流程完全一样地启动 TIM7 */
#endif
    OLED_Clear();
    OLED_ShowString(0,  0, (uint8_t *)"B1 TIM7 ON", 16, 1);
    OLED_ShowString(0, 16, (uint8_t *)"B2 wait 5s", 16, 1);
    OLED_ShowString(0, 32, (uint8_t *)"B3 see debug", 16, 1);
    OLED_ShowString(0, 48, (uint8_t *)"B4 C8 pause", 16, 1);
    OLED_Refresh();
    HAL_Delay(5000); /* 这 5 秒里 TIM7 正常的话，上面四行会被调试画面整个盖掉 */

    /* ---------------- 之后：来回切，方便一直盯着看 ---------------- */
    for(;;)
    {
        Dbg_ShowScreen(0);       /* 调试服务那份整屏画面 */
        HAL_Delay(2000);

        OLED_Clear();            /* 自己写的对照画面（写完过一会儿又会被 TIM7 盖掉） */
        OLED_ShowString(0,  0, (uint8_t *)"B5 my screen", 16, 1);
        OLED_ShowString(0, 16, (uint8_t *)"B6 TIM7 will", 16, 1);
        OLED_ShowString(0, 32, (uint8_t *)"B7 cover it", 16, 1);
        OLED_Refresh();
        HAL_Delay(2000);
    }
}

/* ========================== 9 臂测试（步进电机 + 两个舵机） ==========================
   目的：把臂上几个执行器的**方向、比例、停位**实测出来。四个子测试各自是一个
   "死循环、不返回"的函数，函数体都在 pick.c，一次只能跑一个（下面换注释）：

     高度步进电机 = 地址5 = Pick_Test_MotorZ()      0 → 30mm → 0，每处停 3s
     半径步进电机 = 地址6 = Pick_Test_MotorR()      0 → 30mm → 0，每处停 3s
     车上转盘舵机 = PA5    = Pick_Test_TurnTable()  A(30°)/B(150°)/C(270°) 循环，每处停 3s
     Z轴旋转舵机  = PE9    = Pick_Test_ZServo()     0°/100°/50° 循环，每处停 3s
     夹爪舵机    = PA6    = Pick_Test_Gripper()     观察/抓紧/放开 走 3 轮，
                                                    停在抓紧（唯一会**返回**的子测试）
                                                    角度见 pick.h 的 PICK_OBSERVE/GRAB/RELEASE

   为什么非测不可：
     * 高度：pick.c 里原来写"一圈=8mm"，机械上量出来是 60mm/圈，差 7.5 倍。
       按旧数字走 30mm 实际会走 225mm —— 已先把比例改成 60mm/圈，这次验它。
       （顺带验证：POS_OBSERVE_CIRCLE 的高度 140mm 按 8mm/圈 是 17.6 圈，
       不可能；按 60mm/圈 是 2.3 圈，合理。）
     * 半径：PICK_MOTOR_R_MM_PER_REV = 100mm/圈 是**纸面值，一次都没实测过**。
     * 转盘：2026-10-14 接线和 Z轴对调过（转盘从 PE9/TIM1 换到 PA5/TIM2），
       换完必须验一遍；顺带看三个槽的角度和停位准不准。
     * Z轴：同上，它从 PA5/TIM2 换到了 **PE9/TIM1**。两路对调是一起做的，
       验的时候注意别只看一个就下结论 —— 两个都动 = 换对了。
     * 夹爪：三个角度(PICK_OBSERVE/GRAB/RELEASE)一直没人专门验过，只跟着取放
       流程"看着差不多"。这次把三个状态单独走一遍，量准开合口。夹爪 PA6/TIM3
       没跟着那次对调动过，所以它动起来只能证明这条链通、证明不了对调。

   判读（屏现在是坏的，靠看 + 尺子；每步之间有 3 秒可以量）：
     方向：0 → 30mm 时臂往哪边走。反了就取反 pick.c 顶部的 POS_DIR
           （高度现在 0 / 半径现在 1），重烧再看。
     比例：量实际走了多少 mm。高度量到 30mm = 60mm/圈 对了（量到 4mm 左右
           = 还是旧的 8mm/圈）；半径量到 30mm = 100mm/圈 对了。别的数按比例回推。
     响应：每步卡 5 秒才动 = 电机没在线/没使能/地址不对。
     转盘：是不是正好停在三个料位上；不动就回去查 PA5 的线和 TIM2 的预分频。
   每条的具体说明写在 pick.c 里各自函数头上。

   跑之前：
     * **步进电机两个轴：把臂放在行程中段**（高度上下各留 5cm、半径内外各留 3cm
       以上）再上电 —— 方向没确认前，30mm 万一往限位那侧走，贴边起跑会顶死。
     * **转盘：臂不能在车体上方**（转盘带料转的时候臂压着会把料扫飞）。
       本函数开头的 Pick_Init() 已经把臂收到 POS_START(运输位)，安全。
     * **Z轴：臂要抬起来。** 运输位 POS_START 的高度只有 10mm，0°/100°/50°
       这么大范围摆 Z 轴很容易蹭到桌面或车体 —— 先把臂手动抬到行程中上部，
       或者把 pick.h 里 POS_START 的高度(现在是 10)改大一点再跑。
   C8 暂停在这个测试里没用：它只给四个轮发停车，不停臂的任何电机/舵机。

   步进电机那两个**必须** TEST_CAN_ENABLE=1（要 CAN 驱动），而且被测的那个电机
   得真的接在总线上 —— 否则 can_SendCmd() 的 while 会死循环。
   （CAN 是广播总线，有一个能回 ACK 的节点就能发，不要求六个电机全接上。）
   **转盘、Z轴、夹爪三个都不需要 CAN** —— 纯舵机测试，CAN 没接也应该能跑。

   夹爪那个是**唯一会返回**的子测试（跑完停在"抓紧"，不返回的话 Test_Switch
   末尾的 for(;;) 会兜住 —— 但它自己在末尾留了个喂 Dbg_Service 的空转循环，
   好让 C8 暂停和刷屏还活着）。其余四个都是死循环。 */
static void Test_Motor(void)
{
    Pick_Init();      /* 机械臂上电：起三个舵机(臂保持姿势) + 收臂到运输位。
                         不自己调 Servo_Init 是为了跟其他测试一个路子；它内部那句
                         Pick_JumpTo POS_START 只动高度(到 10mm)，半径目标 0mm
                         等于不动 —— 所以三个子测试都不受它干扰。
                         转盘子测试就靠这里起的舵机。 */

    /* 五个子测试一次只能跑一个 —— 换就换注释（同一时刻只留一行没注释）。
       判读方法见 pick.c 里各自函数头上那段。 */
    //Pick_Test_MotorZ();               /* 高度轴：死循环，不返回 */
    //Pick_Test_MotorR();             /* 半径轴：把上面那行注释掉、这行打开 */
    //Pick_Test_TurnTable();          /* 转盘舵机：A(30°)/B(150°)/C(270°)，不需要 CAN */
    //Pick_Test_ZServo();             /* Z轴舵机：0°/100°/50°，不需要 CAN（臂先抬高）*/
    //Pick_Test_Gripper();            /* 夹爪：观察/抓紧/放开 走 3 轮停在抓紧，不需要 CAN */

}

/* ========================== 分发入口 ==========================
   main() 在 AGV_Init() 之前调它。测试模式下不返回。 */
void Test_Switch(void)
{
    Test_HwInit();

    Test_WaitStartKey();    /* 阻塞等 A8 短按才往下走；TEST_MODE=0 那条分支不经过这里 */

    switch(TEST_MODE)
    {
        case 1:  Test_AGV();         break;  //跑一遍位置
        case 2:  Test_Correct();     break;  //夹爪到观察位置 发送0x02 位置修正
        case 3:  Test_Pick_Scale();  break;  //POS_RUN上夹爪开合死循环(不返回)
        case 4:  Test_Pick_Raw();    break;  //不用看任务码，依次抓取ABC到车上123位置
        case 5:  Test_Pick_Place();  break;  //不用看任务码，依次抓取车上123位置到圆环123位置
        case 6:  Test_Func();        break;  //之后用来测试Pick_Raw Pick_Place等函数的功能
        case 7:  Test_Camera();      break;  //定时器自带oled显示接收数据 ，然后每隔5秒轮流发送所有模式指令 比如任务0x02的可以就123，0x05就用颜色数字2
        case 8:  Test_OLED();        break;  //屏幕自检：甲段(不启TIM7)黑白闪+标记字，乙段(启TIM7)看调试画面(不返回)
        case 9:  Test_Motor();       break;  //臂测试：高度/半径步进电机(0<->30mm往返)或转盘舵机(A/B/C槽)，靠看+尺子验方向和比例(不返回)
        default: break;                      /* 号还没写实现的：初始化完就停住 */
    }

    for(;;) { }     /* 测试跑完停在这儿 —— 程序不会走到 AGV_Init() */
}

#endif  /* TEST_MODE == 0 */
