/**
 * @file openmv.c
 * @brief OpenMV 视觉模块：串口接收并解析 3 个字段
 *        数据格式："target_dx,target_dy,openmv_cmd\n"
 *        偏差为当前目标(物块/圆环由OpenMV自动判定)的X/Y偏差(像素)，无目标 -999
 *        openmv_cmd：空闲为 -1，颜色命令后回显当前颜色号
 *        数据源：USART1 + DMA（空闲中断），OpenMV 通过串口发送
 *        调用方只需读 OpenMV_RxFlag 及各字段变量。

 *        绿线接相机tx P4引脚，蓝线接相机rx P5引脚
 */
#include "openmv.h"
#include "usart.h"
#include "oled.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdio.h>

volatile uint8_t openmv_start=0;
volatile uint8_t OpenMV_RxFlag = 0;   /* 1=已收到并解析完一帧新数据 */
volatile int16_t target_dx = -999;    /* 目标X偏差(像素)：物块/圆环由OpenMV自动判定 */
volatile int16_t target_dy = -999;    /* 目标Y偏差(像素) */
volatile int16_t openmv_cmd = -1;     /* OpenMV 命令字段：-1空闲，颜色命令后回显颜色号 */

static uint8_t RxBuffer[50];          /* DMA接收缓冲 */
static uint8_t RxParseBuffer[50];     /* 接收缓冲的拷贝，供解析用，防止解析期间被新数据覆盖 */
static uint16_t RxLen = 0;            /* 本帧有效接收长度 */


/**
 * @brief 解析逗号分隔的 3 个整数字段，写回全局变量
 *        格式："target_dx,target_dy,openmv_cmd"，行尾为\r\n
 * @param buf 待解析字符串缓冲区
 * @param len 有效字符数
 * @retval 1 解析成功；0 解析失败(格式错误)
 */
static uint8_t OpenMV_ParseData(const uint8_t *buf, uint16_t len)
{
    uint16_t i = 0;
    volatile int16_t *out[3] = { &target_dx, &target_dy, &openmv_cmd };

    for(uint8_t field = 0; field < 3; field++)
    {
        int32_t v = 0;
        int8_t sign = 1;
        uint8_t digitCnt = 0;

        /* 符号位 */
        if(i < len && (buf[i] == '-' || buf[i] == '+'))
        {
            if(buf[i] == '-') sign = -1;
            i++;
        }

        /* 数字位，遇逗号/行尾结束 */
        while(i < len && buf[i] != ',' && buf[i] != '\r' && buf[i] != '\n')
        {
            if(buf[i] < '0' || buf[i] > '9')
            {
                return 0;
            }
            v = v * 10 + (buf[i] - '0');
            if(v > 32767 + (sign < 0))
            {
                return 0;   /* 超出int16_t范围 */
            }
            digitCnt = 1;
            i++;
        }
        if(digitCnt == 0)
        {
            return 0;       /* 缺数字 */
        }

        *out[field] = (int16_t)(sign * v);

        /* 前2个字段后必须有逗号；最后一个字段可到行尾 */
        if(field < 2)
        {
            if(i >= len || buf[i] != ',')
            {
                return 0;   /* 缺逗号分隔 */
            }
            i++;            /* 跳过逗号 */
        }
    }
    return 1;
}

/**
 * @brief 挂一次串口 DMA 接收（幂等：正在接收时返回 BUSY 无副作用）
 * @note  先关 USART1 全局中断并清残留错误/空闲标志，再启动。
 *        目的：规避 HAL 启动序列里"遗留串口错误中断在启动瞬间把接收中止"的竞态——
 *        HAL 在 DMA 接收模式下遇 FE/NE/ORE 会当作阻塞错误中止接收(见HAL_UART_IRQHandler)，
 *        若中止后无人重挂，openmv_cmd 就永远停在 -1。
 */
static void OpenMV_StartRx(void)
{
    HAL_NVIC_DisableIRQ(USART1_IRQn);          /* 关全局中断，防启动竞态 */

    __HAL_UART_CLEAR_OREFLAG(&huart1);         /* 读SR再读DR，清溢出等残留错误标志 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);        /* 清空闲标志，避免一启动就触发空闲中断 */
    huart1.ErrorCode = HAL_UART_ERROR_NONE;    /* 清错误码，否则DMA满后不进解析回调 */

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuffer, 50);

    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief 启动串口 DMA 接收（在 AGV_Init 中调用一次）
 * @note  就绪判据 = openmv_cmd != -1。这里 -1 仅指"尚未收到任何一帧"(变量定义初值)。
 *        前提：相机空闲/无目标时须回显非 -1 的代码(如旧代码空闲回显 "6")，
 *        否则相机上电停在空闲态会误等满 5s。切勿让相机在空闲态回显 -1。
 */
void OpenMV_Init(void)
{
    OpenMV_StartRx();
    uint32_t start_time = HAL_GetTick();    // 记录进入时间
    while(1)
    {
        // 5秒超时保护
        Openmv_OLED_Show();             /* 调试:显示首帧数据 */
        if((HAL_GetTick() - start_time) >= 5000)
        {
            OLED_ShowString(8, 32, (uint8_t *)("openmv error"), 16, 1);
            break;  // 超时退出，不再死等
        }
        if(openmv_cmd != -1)                /* 收到首帧(相机回显非-1代码)=链路已通 */
        {
            openmv_start = 1;
            Openmv_OLED_Show();             /* 调试:显示首帧数据 */
            break;
        }
        OpenMV_Service();                   /* 接收自愈：串口错误中止接收后重新挂DMA */
        HAL_Delay(50);
    }
}

/**
 * @brief 接收链自愈：周期性调用(如主循环/调试循环每100ms一次)
 * @note  HAL 在 DMA 接收模式下遇串口错误(FE/NE/ORE)会把接收中止并把状态打回
 *        READY，且中止后不会自动重启 → openmv_cmd 永远停在 -1。
 *        本函数检查到状态不是 BUSY_RX(即接收链已死)就立即重挂 DMA 恢复。
 */
void OpenMV_Service(void)
{
    if(HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
    {
        OpenMV_StartRx();
    }
}

/**
 * @brief OLED 显示视觉参数（调试用，多处可调用）
 *        2 行：T:目标X,Y / ID:openmv_cmd
 */
void Openmv_OLED_Show(void)
{
    char buf1[32], buf2[32];
    sprintf(buf1, "T:%d,%d", target_dx, target_dy);
    sprintf(buf2, "ID:%d",   openmv_cmd);
    OLED_Clear();
    OLED_ShowString(0, 0,  (uint8_t *)buf1, 16, 0);
    OLED_ShowString(0, 16, (uint8_t *)buf2, 16, 0);
    OLED_Refresh();
}

/* ========================== 目标运动→静止检测 ==========================
   语义:抓"物块从运动到静止的那一刻"(边沿), 不是稳态"当前是否静止"。
   为什么:只判静止会在一次停止窗口的末尾(即将开始下一次转动前)也被判成静止,
         这时去修正/抓取正好撞上下一次运动 → 必须先看到在动、再停稳才返回。
   判据:用"相对锚点的窗口累计漂移(穿盒)", 不用逐帧位移:
   - 静止抖动 ~±1px/帧 是零均值 → 位置绕锚点漂不出 ±BOX 的盒子;
   - 慢速运动 15px/s @40fps 只有 ~0.4px/帧, 但每 ~BOX/speed≈200ms 就累计穿盒;
   - 穿盒(在动)之后连续 QUIET 不穿盒 = 刚停下 → 返回。
   注意: 低于 BOX/QUIET ≈ 7.5px/s 的漂移会被当成静止(量级上够用);
         进入时若目标本就静止(本次观察内一直没动过), 会等到下一次运动并停稳为止
         (由 TIMEOUT 保护)。 */
#define OPENMV_STILL_BOX_PX     2       /* 锚点盒子半宽(px): 静止累计漂移不得超过它 */
#define OPENMV_STILL_QUIET_MS   300U    /* 穿盒后需连续静置多久才算"刚停下"(ms) */
#define OPENMV_STILL_TIMEOUT_MS 30000U  /* 判稳总超时保护 */
#define OPENMV_NEAR_WIN_PX      45      /* WaitForNearStill 的"接近可抓区"窗口(px) */

/**
 * @brief 通用"目标刚停下"等待(运动→静止边沿)
 * @param winPx 需停在 |dx|,|dy| <= winPx 窗口内才算稳; 传 0 = 不限位置, 只求静止
 */
static void OpenMV_WaitSettle(int16_t winPx)
{
    const uint32_t t0 = HAL_GetTick();
    uint8_t  have = 0;          /* 是否已有有效目标位置 */
    uint8_t  sawMotion = 0;     /* 是否已经看到过在动(穿盒) — 边沿的前提 */
    int16_t  ancX = 0, ancY = 0;/* 静止盒子的锚点(当前判稳参考位置) */
    uint32_t quietSince = 0;    /* 当前这段"未穿盒"的起点 */

    while((HAL_GetTick() - t0) < OPENMV_STILL_TIMEOUT_MS)
    {
        Openmv_OLED_Show();               /* 调试:显示当前帧数据 */
        OpenMV_Service();               /* 接收链自愈 */
        if(!OpenMV_RxFlag) { HAL_Delay(1); continue; }
        OpenMV_RxFlag = 0;

        /* 目标丢失: 状态作废, 重新等 */
        if(target_dx == -999 || target_dy == -999)
        {
            have = 0; sawMotion = 0;
            continue;
        }
        if(!have)
        {
            have = 1;
            ancX = target_dx; ancY = target_dy;
            quietSince = HAL_GetTick();
            continue;
        }

        /* 相对锚点的累计漂移: 静止抖动零均值漂不出盒子; 慢速运动会累计穿盒 */
        int16_t ex = target_dx - ancX; if(ex < 0) ex = -ex;
        int16_t ey = target_dy - ancY; if(ey < 0) ey = -ey;

        if(ex > OPENMV_STILL_BOX_PX || ey > OPENMV_STILL_BOX_PX)
        {
            sawMotion = 1;                          /* 穿盒 = 看到在动 */
            ancX = target_dx; ancY = target_dy;     /* 盒子跟到当前位置 */
            quietSince = HAL_GetTick();
            continue;
        }
        /* 有限定窗口: 盒子内但停在窗口外 → 不算稳, 等它进窗 */
        if(winPx > 0 && (target_dx > winPx || target_dx < -winPx ||
                         target_dy > winPx || target_dy < -winPx))
        {
            quietSince = HAL_GetTick();
            continue;
        }
        /* 已经看到在动 + 当前盒子内连续静置够久 = "刚停下" */
        if(sawMotion && (HAL_GetTick() - quietSince) >= OPENMV_STILL_QUIET_MS)
        {
            break;
        }
    }
}

/**
 * @brief 等待目标"刚停下": 先看到在动、再停稳即返回(不限位置)。见 WaitSettle 顶注。
 */
void OpenMV_WaitForStill(void)
{
    OpenMV_WaitSettle(0);
}

/**
 * @brief 等待目标"刚停下"且位于视野中央 ±40px 窗口内(接近可抓区)
 */
void OpenMV_WaitForNearStill(void)
{
    OpenMV_WaitSettle(OPENMV_NEAR_WIN_PX);
}

/**
 * @brief 串口空闲中断回调：收到一帧数据后解析，任务读取
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART1)
    {
        if(Size <= 50)
        {
            for(uint16_t i = 0; i < Size; i++)
            {
                RxParseBuffer[i] = RxBuffer[i];   /* 拷贝后立即重启DMA接收，避免解析期间被覆盖 */
            }
            RxLen = Size;

            if(OpenMV_ParseData(RxParseBuffer, RxLen))
            {
                OpenMV_RxFlag = 1;                /* 解析成功，置标志供任务读取 */
            }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuffer, 50);
    }
}


/**
 * @brief 依次探测三种颜色、记录各自物块离视野中心的距离，最后切到最近的颜色
 * @note  发送/回显确认统一走 OpenMV_SwitchTo()（ASCII数字+定时补发+3s超时），
 *        不再手动 HAL_UART_Transmit。
 *        切换失败(3s超时)的颜色置为 30000 远距，不参与"最近"竞争；
 *        三种颜色都失败(相机无响应/没有可见物块)则直接返回，不白等最后一次。
 */
void OpenMV_SwitchCloseColor(void)
{
    extern uint8_t Pick_Color1[3];
    extern uint8_t Pick_Color2[3];
    extern uint8_t RerunFlag;
    uint8_t *Pick_Color = (RerunFlag == 0) ? Pick_Color1 : Pick_Color2;

    typedef struct {
        int16_t dx;
        int16_t dy;
        uint8_t color_id;
        int16_t distance;
    } ColorData_t;
    ColorData_t colors[3] = {0};
    uint8_t ok_count = 0;

    /* 循环三种颜色:切过去→等一帧该色坐标刷到 target_*→记录距离 */
    for(uint8_t i = 0; i < 3; i++)
    {
        colors[i].color_id = Pick_Color[i];
        colors[i].distance = 30000;                 /* 默认远距 */
        if(OpenMV_SwitchTo(colors[i].color_id))     /* 回显==色号即确认切好 */
        {
            ok_count++;
            HAL_Delay(50);                          /* 确认帧已带该色坐标,再等一帧保险 */
            colors[i].dx = target_dx;
            colors[i].dy = target_dy;
            /* 该色物块不可见时 dx/dy=-999 → 距离1998,仍远于正常可见值 */
            colors[i].distance = (int16_t)((target_dx >= 0 ? target_dx : -target_dx) +
                                           (target_dy >= 0 ? target_dy : -target_dy));
        }
    }
    if(ok_count == 0) return;                       /* 全部切换失败,直接返回 */

    /* 找离视野中心最近的 */
    uint8_t min_idx = 0;
    for(uint8_t i = 1; i < 3; i++)
    {
        if(colors[i].distance < colors[min_idx].distance) min_idx = i;
    }
    OpenMV_SwitchTo(colors[min_idx].color_id);      /* 最终切到最近的颜色 */
}

/**
 * @brief 发送颜色切换命令给OpenMV，并等待其回显确认切换成功
 * @note  颜色号是QR解析出的单个ASCII数字(0~9)。发送后相机在下一帧数据帧的
 *        openmv_cmd 字段回显该数字，相等即确认已切到该颜色。
 *        等待期间每 OPENMV_SWITCH_RETRY_MS 补发一次(命令幂等,补发无害)。
 *        补发不能比相机帧周期(~50ms)更频繁——发太快只会灌满其接收缓冲、
 *        并让相机反复重启颜色检测而永不就绪；点对点串口首帧极难丢，慢补即可。
 *        同时每拍调 OpenMV_Service() 自愈接收链(本工程DMA接收遇串口错误会中止，
 *        是"回显永远不来"的真凶之一)。
 * @param color 目标颜色号 0~9
 * @retval 1 相机已回显确认；0 3s超时仍未确认
 */
uint8_t OpenMV_SwitchTo(uint8_t color)
{
    const uint32_t OPENMV_SWITCH_TIMEOUT_MS = 3000U;  /* 总等待 */
    const uint32_t OPENMV_SWITCH_RETRY_MS   = 500U;   /* 无回显时的补发周期 */
    char digit = (char)('0' + (color % 10U));         /* 颜色号=单个ASCII数字 */
    uint32_t t0 = HAL_GetTick();
    uint32_t lastSend = 0;                            /* 首轮立即发送 */

    while((HAL_GetTick() - t0) < OPENMV_SWITCH_TIMEOUT_MS)
    {
        OpenMV_Service();                             /* 接收链自愈 */
        if(openmv_cmd == (int16_t)color)
        {
            return 1;                                 /* 回显相等,切换成功 */
        }
        if((HAL_GetTick() - lastSend) >= OPENMV_SWITCH_RETRY_MS)
        {
            HAL_UART_Transmit(&huart1, (uint8_t *)&digit, 1, HAL_MAX_DELAY);
            lastSend = HAL_GetTick();
        }
        HAL_Delay(10);
    }
    return 0;                                         /* 3s超时 */
}