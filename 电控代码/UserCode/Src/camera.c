/**
 * @file camera.c
 * @brief 相机视觉模块：二进制定长帧收发（协议见 camera.h 顶部注释）
 *        帧格式 AA <cmd> <payload...> BB；dx/dy 为有符号 int16 高字节在前
 *        数据源：USART1 + DMA（空闲中断）
 *        绿线接相机tx P4引脚，蓝线接相机rx P5引脚
 *        （模块原名 openmv.c / OpenMV_*，2026-10-04 统一改成 camera.c / Cam_*）
 */
#include "camera.h"
#include "usart.h"
#include "oled.h"
#include "QRcode.h"      /* Pick_Color1 / Place_Color1 —— Cam_SwitchTo 的 0x01 / 0x05 要自己取参数 */
#include "test.h"        /* Dbg_ShowScreen —— 整屏显示(原 Cam_OLED_Show 已并入 test.c) */
#include "stm32f4xx_hal.h"

/* ========================== 帧长表 ========================== */
/* 相机返回帧的总长（含 AA 与 BB），按命令码区分 —— 新协议是定长帧，靠它切帧 */
#define CAM_LEN_IDLE      4U     /* AA 00 flag BB */
#define CAM_LEN_TASKCODE  4U     /* AA 01 flag BB */
#define CAM_LEN_DISC      8U     /* AA 02 flag dxH dxL dyH dyL BB */
#define CAM_LEN_BOXCOLOR  7U     /* AA 03 flag c1 c2 c3 BB */
#define CAM_LEN_RING      8U     /* AA 04 flag dxH dxL dyH dyL BB */
#define CAM_LEN_COLOR     8U     /* AA 05 flag dxH dxL dyH dyL BB */
#define CAM_LEN_ANGLE     5U     /* AA 06 flag AngleH AngleL BB */

#define CAM_RAW_LEN   64U    /* DMA 单次接收缓冲 */
#define CAM_ACC_LEN   96U    /* 字节流累积缓冲：一次中断可能收到半帧或多个帧 */

#define CAM_INIT_TMO_MS   5000U  /* 上电探链路总超时 */
#define CAM_RESEND_MS      100U  /* 第一段等待期间没动静就补发一次命令 */
#define CAM_ACK_TMO_MS   10000U  /* 第一段超时：等相机回"本命令"的帧(第二段由调用方传，两段都是 10s) */

/* ========================== 解析结果 ========================== */
volatile int16_t dx = 0;                /* 最近一帧偏差X(px)，仅 0x02/0x04 更新 */
volatile int16_t dy = 0;                /* 最近一帧偏差Y(px) */
volatile uint8_t cam_flag = 0;          /* 最近一帧的 flag 字段；0=没识别到目标 */
volatile uint8_t cam_mode = 0xFF;       /* 最近一帧的命令码 */
volatile uint8_t cam_receive_flag = 0;  /* 1=有新帧待读，读取方清零 */
volatile uint8_t box_color[3] = {0, 0, 0};   /* 0x03 的三个框内颜色 */
volatile int16_t cam_angle = 0;         /* 0x06 的偏离角度(°)，仅 0x06 更新 */

volatile uint8_t cam_start = 0;

/* 完成事件的锁存：flag!=0 的那一帧必须单独记下来，
   否则会被紧随其后的 flag=0 帧覆盖，等待循环就永远等不到"干完了"。
   只给本文件的等待函数用，不对外暴露。
   ⚠ 只锁"哪条命令完成了"，不锁它的 flag 值 —— 完成帧的 flag 调用方返回后
   直接读全局 cam_flag 就够（Cam_SwitchTo 是在收到完成帧那一拍当场 return 的）。*/
static volatile uint8_t s_doneCmd  = 0xFF;   /* 最近一个 flag!=0 的命令码 */

/* 握手锁存：最近一帧的命令码，**不管 flag 是几都记**。
   相机在任何模式下都会持续往回发数据，所以"收到一帧"完全不能说明它收到了我们的
   命令；只有收到「命令码和刚发出去的那条相同」的帧才算命令被采用。第一段等待用它，
   读的是一个"有没有"的锁存而不是 cam_mode，免得被后面来的别种帧冲掉。 */
static volatile uint8_t s_ackCmd   = 0xFF;

/* ========================== 接收状态 ========================== */
static uint8_t  RxBuffer[CAM_RAW_LEN];   /* DMA 落点 */
static uint8_t  RxAcc[CAM_ACC_LEN];      /* 字节流累积，跨中断拼帧 */
static uint16_t RxAccLen = 0;

/* ========================== 发送 ========================== */
static void Cam_SendRaw(const uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, HAL_MAX_DELAY);
}

/**
 * @brief 发送一帧命令：AA <cmd> <payload...> BB
 * @param cmd     命令码
 * @param payload 参数字节首地址，无参数命令传 NULL
 * @param plen    参数字节数(0~3)：0x01 传 3、0x05 传 1、其余传 0
 * @note  参数长度上限 3 由本函数的 f[6] 决定；协议里最长的命令就是 0x01 的三色。
 */
static void Cam_SendCmdPayload(uint8_t cmd, const uint8_t *payload, uint8_t plen)
{
    uint8_t f[6];       /* AA + cmd + 最多 3 字节参数 + BB */
    uint8_t n = 0;

    f[n++] = CAM_HEAD;
    f[n++] = cmd;
    for(uint8_t i = 0; i < plen; i++) f[n++] = payload[i];
    f[n++] = CAM_TAIL;
    Cam_SendRaw(f, n);
}

/**
 * @brief 挂一次串口 DMA 接收（幂等：正在接收时返回 BUSY 无副作用）
 * @note  先关 USART1 全局中断并清残留错误/空闲标志，再启动。
 *        目的：规避 HAL 启动序列里"遗留串口错误中断在启动瞬间把接收中止"的竞态——
 *        HAL 在 DMA 接收模式下遇 FE/NE/ORE 会当作阻塞错误中止接收(见HAL_UART_IRQHandler)，
 *        若中止后无人重挂，接收链就死了。
 */
static void Cam_StartRx(void)
{
    HAL_NVIC_DisableIRQ(USART1_IRQn);          /* 关全局中断，防启动竞态 */

    __HAL_UART_CLEAR_OREFLAG(&huart1);         /* 读SR再读DR，清溢出等残留错误标志 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);        /* 清空闲标志，避免一启动就触发空闲中断 */
    huart1.ErrorCode = HAL_UART_ERROR_NONE;    /* 清错误码，否则DMA满后不进解析回调 */

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuffer, CAM_RAW_LEN);

    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* ========================== 解析 ========================== */

/**
 * @brief 查命令码对应的返回帧长
 * @retval 帧长(字节)；-1 = 不是已知命令码
 */
static int16_t Cam_FrameLen(uint8_t cmd)
{
    switch(cmd)
    {
        case CAM_CMD_IDLE:     return (int16_t)CAM_LEN_IDLE;
        case CAM_CMD_TASKCODE: return (int16_t)CAM_LEN_TASKCODE;
        case CAM_CMD_DISC:     return (int16_t)CAM_LEN_DISC;
        case CAM_CMD_BOXCOLOR: return (int16_t)CAM_LEN_BOXCOLOR;
        case CAM_CMD_RING:     return (int16_t)CAM_LEN_RING;
        case CAM_CMD_COLOR:    return (int16_t)CAM_LEN_COLOR;
        case CAM_CMD_ANGLE:    return (int16_t)CAM_LEN_ANGLE;
        default:              return -1;
    }
}

/**
 * @brief 从累积缓冲头部丢掉 n 个字节（失步/校验失败时逐字节重同步用）
 */
static void Cam_AccDrop(uint16_t n)
{
    if(n >= RxAccLen)
    {
        RxAccLen = 0;
        return;
    }
    for(uint16_t i = 0; (uint16_t)(i + n) < RxAccLen; i++)
    {
        RxAcc[i] = RxAcc[i + n];
    }
    RxAccLen -= n;
}

/**
 * @brief 解析一帧完整帧（已通过帧头/帧长/帧尾校验），写回全局结果
 */
static void Cam_DecodeFrame(const uint8_t *f)
{
    const uint8_t cmd  = f[1];
    const uint8_t flag = f[2];

    cam_mode = cmd;
    cam_flag = flag;
    cam_receive_flag  = 1;
    s_ackCmd = cmd;     /* 无条件锁存：收到本命令的回帧 = 相机已切到该模式（第一段握手） */

    switch(cmd)
    {
        case CAM_CMD_DISC:   /* AA 02 flag dxH dxL dyH dyL BB */
            dx = (int16_t)(((uint16_t)f[3] << 8) | (uint16_t)f[4]);
            dy = (int16_t)(((uint16_t)f[5] << 8) | (uint16_t)f[6]);
            break;

        case CAM_CMD_RING:   /* AA 04 flag dxH dxL dyH dyL BB */
            dx = (int16_t)(((uint16_t)f[3] << 8) | (uint16_t)f[4]);
            dy = (int16_t)(((uint16_t)f[5] << 8) | (uint16_t)f[6]);
            break;

        case CAM_CMD_COLOR:  /* AA 05 flag dxH dxL dyH dyL BB —— 与 0x02 同构 */
            dx = (int16_t)(((uint16_t)f[3] << 8) | (uint16_t)f[4]);
            dy = (int16_t)(((uint16_t)f[5] << 8) | (uint16_t)f[6]);
            break;

        case CAM_CMD_ANGLE:  /* AA 06 flag AngleH AngleL BB */
            cam_angle = (int16_t)(((uint16_t)f[3] << 8) | (uint16_t)f[4]);
            break;

        case CAM_CMD_BOXCOLOR:   /* AA 03 flag c1 c2 c3 BB */
            box_color[0] = f[3];
            box_color[1] = f[4];
            box_color[2] = f[5];
            break;

        case CAM_CMD_IDLE:
        case CAM_CMD_TASKCODE:
        default:
            break;          /* 只更新 cam_mode / cam_flag */
    }

    if(flag != 0U)      /* 这一轮干完了，锁存命令码给等待循环 */
    {
        s_doneCmd = cmd;
    }
}

/**
 * @brief 在累积缓冲里逐帧切分：找 AA → 按命令码取帧长 → 校验 BB → 解析
 * @note  失步（非帧头字节/未知命令/帧尾不符）时只丢 1 字节再重试，
 *        这样噪声里的 0xAA 也能被跳过；帧没收全就退出，等下一批字节。
 */
static void Cam_ParseAcc(void)
{
    while(RxAccLen >= 2U)
    {
        if(RxAcc[0] != CAM_HEAD)
        {
            Cam_AccDrop(1);              /* 不是帧头，丢一字节重同步 */
            continue;
        }

        int16_t flen = Cam_FrameLen(RxAcc[1]);
        if(flen < 0)
        {
            Cam_AccDrop(1);              /* 未知命令码：可能是噪声里的 AA */
            continue;
        }

        if(RxAccLen < (uint16_t)flen)
        {
            break;                          /* 本帧还没收全，等下一批 */
        }

        if(RxAcc[flen - 1] != CAM_TAIL)
        {
            Cam_AccDrop(1);              /* 帧尾不符，丢头字节重找 */
            continue;
        }

        Cam_DecodeFrame(RxAcc);
        Cam_AccDrop((uint16_t)flen);     /* 消费掉整帧 */
    }
}

/**
 * @brief 串口空闲中断回调：把本批字节追加进累积缓冲后切帧解析
 * @note  定长帧协议下一次中断可能只收到半帧、也可能一次收到好几帧，
 *        所以必须累积+切分，不能像旧协议那样把整批当一个完整帧。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART1)
    {
        if(((uint16_t)(RxAccLen + Size)) <= CAM_ACC_LEN)
        {
            for(uint16_t i = 0; i < Size; i++)
            {
                RxAcc[RxAccLen + i] = RxBuffer[i];
            }
            RxAccLen += Size;
        }
        else
        {
            RxAccLen = 0;       /* 累积缓冲溢出：丢帧或噪声，整个清掉重新同步 */
        }

        Cam_ParseAcc();

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuffer, CAM_RAW_LEN);
    }
}

/* ========================== 等待 ========================== */

/**
 * @brief 发一条请求命令，两段式等待：先等相机"回报本命令"，再等它"回 flag!=0"
 * @param cmd        命令码，0x01 ~ 0x06 这些请求式命令都走这里
 * @param payload    参数字节首地址：0x01 传三色 c1c2c3、0x05 传 color、其余传 NULL
 * @param plen       参数字节数 0~3
 * @retval 1 两段都完成；0 任一段超时
 *
 * @note  **为什么分两段**：相机在任何模式下都会持续往回发数据，所以"收到一帧"本身
 *        完全不能说明它收到了我们的命令 —— 必须收到「第二字节(命令码)和刚发出去的
 *        那条相同」的帧，才算相机切到了该模式并按这条命令响应。这就是第一段存在的
 *        理由，它同时也是唯一的链路判据。
 *
 *        第一段：等 s_ackCmd == cmd；每 CAM_RESEND_MS 补发一次(命令幂等，补发无害)，
 *                CAM_ACK_TMO_MS(10s) 到点还没等到 → 返回 0(命令没送达)。
 *        第二段：等 s_doneCmd == cmd(即回了一帧 flag!=0)；CAM_REQ_TMO_MS(20s) 到点 → 返回 0。
 *                **不补发** —— 相机已在本模式下持续回帧，重复下发可能让它重开一轮识别。
 *                两句超时都不从外面传：调用方(Cam_SwitchTo)只管挑模式，耐心是本模块的事。
 *
 *        两段锁存都在发命令之前一起清；所以第一段期间若相机就已经回了 flag!=0，
 *        第二段进来立刻命中返回，不会把那次完成事件漏掉。
 *        每拍调 Cam_Service() 自愈接收链。
 */
static uint8_t Cam_WaitSettle(uint8_t cmd, const uint8_t *payload, uint8_t plen)
{
    uint32_t t0, lastSend;

    s_ackCmd  = 0xFF;           /* 清两段锁存 */
    s_doneCmd = 0xFF;

    Cam_SendCmdPayload(cmd, payload, plen);      /* 首帧立即发，不等补发周期 */
    lastSend = HAL_GetTick();

    /* ---- 第一段：等相机回一帧「命令码 == cmd」的，确认命令被采用 ---- */
    t0 = HAL_GetTick();
    while((HAL_GetTick() - t0) < CAM_ACK_TMO_MS)
    {
        Cam_Service();       /* 接收链自愈 */
        if(s_ackCmd == cmd) break;

        if((HAL_GetTick() - lastSend) >= CAM_RESEND_MS)
        {
            Cam_SendCmdPayload(cmd, payload, plen);  /* 相机没动静，补发 */
            lastSend = HAL_GetTick();
        }
        HAL_Delay(1);
    }
    if(s_ackCmd != cmd) return 0;       /* 第一段超时：命令没送达 */
    /* ---- 第二段：等这同一条命令回 flag!=0，即相机把活干完了 ---- */
    t0 = HAL_GetTick();
    while((HAL_GetTick() - t0) < CAM_REQ_TMO_MS)
    {
        Cam_Service();
        if(s_doneCmd == cmd) return 1;
        HAL_Delay(1);
    }
    return 0;
}

/* ========================== 新协议 API ========================== */

void Cam_SendIdle(void)
{
    Cam_SendCommand(CAM_CMD_IDLE, NULL, 0);
}

/**
 * @brief 发送一条命令帧，**发完就走、不等回复**（AA <cmd> <payload...> BB）
 * @param cmd     命令码
 * @param payload 参数字节首地址；无参数命令传 NULL
 * @param plen    参数字节数(0~3)：0x01 传 3、0x05 传 1、其余传 0
 * @note  相机被 Cam_Init() 的 0x00 唤醒之后就会**持续**往回发它当前模式的数据
 *        （见 camera.h 顶部"相机在任何模式下都会持续回数据"），所以这里不需要等 ——
 *        发完结果自己去看全局 cam_mode / cam_flag / dx / dy / box_color[]。
 *
 *        和 Cam_SwitchTo() 的区别就在这：那个是"发一条、等它干完"的阻塞版
 *        （第一段等回帧固定 10s + 第二段 CAM_REQ_TMO_MS 20s），相机没接时会一条卡 30 秒，
 *        用来做"每 N 秒换一条命令"的轮询是不行的。这个是几十微秒。
 *
 *        注意这不代表"发出去就一定被采用"：相机不认这条命令的话，它会继续回
 *        上一个模式的数据，cam_mode 也就不变 —— 判据是"cam_mode 跟上了没有"。
 */
void Cam_SendCommand(uint8_t cmd, const uint8_t *payload, uint8_t plen)
{
    Cam_SendCmdPayload(cmd, payload, plen);
}

/**
 * @brief 统一切换相机模式：发一条命令 + 阻塞等它干完。
 *        原来那五个 Cam_RequestXxx()（TaskCode/Disc/BoxColor/Ring/Color）已经删掉，
 *        出参一律没有 —— 结果全在全局量 dx/dy/box_color[]/cam_flag 里，返回后直接读。
 * @param mode 命令码 0x00~0x06，调用处直接写八进制字面量 01/02/03/04/05/06（写 2 也行，
 *             是同一个值；但 08/09 不是合法字面量）。带参数的两条自己取参数，调用方不用管：
 *             0x01 任务码   → 直接用本轮扫出来的 Pick_Color1[0..2]
 *             0x05 指定颜色 → 按任务码算出"第一轮放进 2 号环的那一块的颜色"
 * @return 1 = 命令被采用且相机回了 flag!=0（全局量已刷新，可读）；
 *         0 = 超时（第一段 10s / 第二段 20s），此时全局量还是旧值；
 *             0x05 另有一种 0：任务码里没有哪一槽进过 2 号环，没颜色可发，一帧都不发。
 * @note  为什么调用方返回后能直接读 cam_flag 拿 flag（0x02 的颜色号 / 0x04 的圆环编号）：
 *        Cam_WaitSettle 是在"收到完成帧"的那一拍当场 return 的，中间没有再收帧，
 *        所以返回点上 cam_flag 就是那条完成帧的 flag。读它之前别插会跑 Cam_Service()
 *        的动作（HAL_Delay / 再接一次 Cam_SwitchTo），否则可能被新来的 flag=0 帧冲掉。
 */
uint8_t Cam_SwitchTo(uint8_t mode)
{
    uint8_t payload[3];
    uint8_t plen = 0;
    switch(mode)
    {
        case CAM_CMD_TASKCODE:          /* 0x01：任务码三色，直接用扫出来的那一组 */
            payload[0] = Pick_Color1[0];
            payload[1] = Pick_Color1[1];
            payload[2] = Pick_Color1[2];
            plen = 3;
            break;
        case CAM_CMD_COLOR:             /* 0x05：第一轮放在 2 号环上的那一块的颜色 */
        {
            uint8_t c = 0;
            if      (Place_Color1[0] == 2U) c = Pick_Color1[0];
            else if (Place_Color1[1] == 2U) c = Pick_Color1[1];
            else c = Pick_Color1[2];
            payload[0] = c;
            plen = 1;
            break;
        }
        default:                        /* 0x00 / 0x02 / 0x03 / 0x04 / 0x06 都不带参数 */
            plen = 0;
            break;
    }
    return Cam_WaitSettle(mode, payload, plen);
}

/* ========================== 初始化 / 维护 ========================== */

/**
 * @brief 启动串口 DMA 接收，并主动发一次空闲命令探链路
 * @note  新协议是"请求-应答"式的：相机上电后不会主动说话，
 *        必须由电控发出第一条命令(0x00 空闲)才会收到回帧，
 *        因此就绪判据是"收到任意一帧合法帧"，而不是旧协议的"相机空闲时自己回显"。
 */
void Cam_Init(void)
{
    uint32_t start_time;
    uint32_t lastSend = 0;

    RxAccLen         = 0;
    cam_receive_flag = 0;
    cam_mode         = 0xFF;
    cam_flag         = 0;
    dx = 0;
    dy = 0;
    cam_angle = 0;
    cam_start     = 0;

    Cam_StartRx();

    start_time = HAL_GetTick();
    while(1)
    {
        Dbg_ShowScreen(0);               /* 显示当前帧数据(测试.c，也是全工程唯一的整屏) */

        if((HAL_GetTick() - start_time) >= CAM_INIT_TMO_MS)
        {
            /* 只写进显存是看不见的：Dbg_ShowScreen 上面那次 Refresh 已经把屏推完了，
               这里再写一行必须自己 Refresh 一次。 */
            OLED_ShowString(8, 32, (uint8_t *)("camera error"), 16, 1);
            OLED_Refresh();
            break;                          /* 超时退出，不再死等 */
        }

        if(cam_receive_flag)                         /* 收到任意合法回帧 = 链路已通 */
        {
            cam_receive_flag = 0;
            cam_start = 1;
            Dbg_ShowScreen(0);
            break;
        }

        if((HAL_GetTick() - lastSend) >= CAM_RESEND_MS)
        {
            Cam_SendIdle();              /* 探链路，超时前反复补发 */
            lastSend = HAL_GetTick();
        }

        Cam_Service();                   /* 接收自愈：串口错误中止接收后重新挂DMA */
        HAL_Delay(50);
    }
}

/**
 * @brief 接收链自愈：周期性调用(如主循环/调试循环每100ms一次)
 * @note  HAL 在 DMA 接收模式下遇串口错误(FE/NE/ORE)会把接收中止并把状态打回
 *        READY，且中止后不会自动重启 → 接收链永久失效。
 *        本函数检查到状态不是 BUSY_RX(即接收链已死)就立即重挂 DMA 恢复。
 */
void Cam_Service(void)
{
    if(HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
    {
        Cam_StartRx();
    }
}
