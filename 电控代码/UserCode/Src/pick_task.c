/**
 * @file pick_task.c
 * @brief 物料取放任务阶段实现（2026-10-03 视觉协议换代后重写；2026-10-16 原料区取料重构）
 *
 * 视觉接口一律走 camera.h 的新二进制定长帧协议：调用方发一条请求命令，
 * 等相机回 flag!=0 的帧即算"干完了"；偏差/颜色从全局量 dx/dy/box_color[] 直接读。
 * 新协议里没有"电控下发颜色号"这回事 —— 颜色是相机回报的东西，不是命令。
 *
 * ── 2026-10-16 原料区取料重构 ──────────────────────────────────────
 * 相机侧把 0x02 的语义改了：原来报"圆盘中心"、flag 只有 0/1，现在报
 * "**最近物块**的位置偏差 + 边沿判断"，**flag 就是那个物块的颜色号(1~6)**，
 * 边沿静止时才发。原料区因此从"读三框颜色表 + 推算转盘走格"改成
 * "在取料位一直等，等到这一色转到取料位上才抓"：
 *   Pick_Correct()   摆 POS_RAW_B → 开爪 → 0x02 → AGV_Correct(5000) → 收爪
 *   Pick_RAW()       摆 POS_RAW_B → 开爪 → 死等 0x02 的 flag == Pick_Color[i] → 抓
 *   Pick_RAW_PLUS()  备用路线：摆 POS_OBSERVE_RAW_CENTER 读一次 0x03 三框颜色，
 *                    一次算出整条路线，分派给六个固定函数之一
 *                    （Pick_RAW_ABC/ACB/BAC/BCA/CAB/CBA，**名字 = 开局观测到的
 *                    初始排列，不是要抓的框**）：抓取过程中底盘完全不动，
 *                    只动臂和转盘
 * 详见各函数自己的注释。
 *
 * ── 2026-10-16 放置侧同样拆开 ──────────────────────────────────────
 * 照原料区那套分工，把视觉对正从两个 Place_xxx 里摘出来，只留下放置动作：
 *   Place_Correct_High()   高位观察位 → 开爪 → 0x04 圆环2 粗修正(1s)，不收爪
 *   Place_Correct_Angle()  挪开 7cm → 0x06 角度闭环 → 记基准 a1 → 退回 → 航向复位 a1
 *   Place_Correct_Low()    低位观察位 → 0x04 细修正(30s) → 固定 0.5° 补角 → 收爪
 *   Place_ROUGH()          只管放：车上三槽依次放到 Place_Color[] 指定的圆环
 *   Place_TEMP_Correct()   摆暂存区观察位 → 开爪 → 0x05 → AGV_Correct(5000) → 收爪
 *   Place_TEMP()           只管放：第二轮同色同环码到高位
 * 上层 task.c：
 *   case Place1  Place_Correct_High() → Place_Correct_Angle() → Place_Correct_Low()
 *                → Place_ROUGH()
 *   case Place2  按 RerunFlag 选对正函数（第一圈观察位就在圆环区，直接用
 *                Place_Correct_Low），再调 Place_TEMP()
 * 0x04 / 0x05 语义都没变，所以这两次拆分都只是搬家，没有逻辑改动。
 *
 * 与旧版的对应关系：
 *   旧 Cam_SwitchTo(color)            → 新 Cam_SwitchTo(02) / (04)
 *   旧 Cam_WaitFor* （电控自己判静止）  → 删除；静止判据由相机侧负责，靠 flag 回报
 *   旧 Cam_SwitchCloseColor()         → 删除；改由 Cam_SwitchTo(03) 问颜色
 *   旧 ScanNearestColor / TrackOneColor 等本地追踪 → 整体删除
 */
#include "pick_task.h"
#include "positionPIDcontroller.h"  // AGV_Correct()

/* 视觉请求的两段超时都在 camera.c 内部、都不是参数：第一段等相机回报本命令是
   CAM_ACK_TMO_MS(10s)，第二段等相机把活干完是 CAM_REQ_TMO_MS(20s)。
   调用 Cam_SwitchTo() 时不用传，只是在死等时要留个心：一条命令最坏卡 30s。 */

/* RerunFlag 定义在 task.c，但 task.h 只声明了 Route_id —— 这里补一条文件作用域声明。
   Pick_Color1/2、Place_Color1/2 不用写：QRcode.h 已声明，经 pick.h 已可见。 */
extern uint8_t RerunFlag;

/* ---- 原料区 A/C 位的底盘补偿 ----
   A、C 两框在车头两侧、比 B 位远，底盘要分两步挪：
     去程：先沿车头方向走 PICK_RAW_STRAIGHT_MM 靠近料架，再**横移**
           PICK_RAW_SIDE_MM 对到框口，然后才让臂下手；
     回程：抓完放回车上后按**相反顺序**退回 —— 先横向、后纵向。
   开环相对位移，来回等距，净值 0，不会让后续导航的坐标基准漂掉。
   方向宏按各自函数的约定分开写，别把两套 dir 混用：
     AGV_Position_GoStraight  0 = 向后、1 = 向前
     AGV_Position_Translate   0 = 向右、1 = 向左
   A、C 分在两侧，横向方向必然相反；纵向方向目前沿用旧值，现场标定不对就改这里。 */
#define PICK_RAW_STRAIGHT_MM     70U    /* 纵向 7cm（旧值，未动） */
#define PICK_RAW_STRAIGHT_A_DIR  1U     /* A 位纵向：1 = 向前 */
#define PICK_RAW_STRAIGHT_C_DIR  0U     /* C 位纵向：0 = 向后（与 A 相反） */

#define PICK_RAW_SIDE_MM         40U    /* 横向 3cm */
#define PICK_RAW_SIDE_A_DIR      0U     /* A 位横向：1 = 向左 */
#define PICK_RAW_SIDE_C_DIR      0U     /* C 位横向：与A相同*/

/* ========================== 静态辅助函数 ========================== */

/**
 * @brief 本轮任务码的取料颜色表(下标 0=A 槽, 1=B 槽, 2=C 槽)
 */
static uint8_t *Pick_Color_List(void)
{
    return (RerunFlag == 0) ? Pick_Color1 : Pick_Color2;
}

/**
 * @brief 从原料区第 box 个框抓到车上第 slot 个槽
 * @param box    框下标 0=POS_RAW_A, 1=POS_RAW_B, 2=POS_RAW_C —— **此刻**料所在的格。
 *               Pick_RAW 固定传 1（0x02 看的"最近物块"就是 B 位那一块）；
 *               Pick_RAW_PLUS 传「初始格 + 已抓块数」推算出的格号。
 * @param slot   槽下标 0=A, 1=B, 2=C（转盘 30°/150°/270°）
 * @param direct 下到料位这一步怎么走：
 *               0 = `Pick_JumpTo`，先升到安全高度再转+伸半径，慢但能跨障碍（两条路线现在都用这个）
 *               1 = `Pick_MoveTo`，贴着观察位高度直接转+伸，最后才降下去，快（暂时没人用）
 *               **只有这一步分两版**：从料位回 POS_CAR 一律 `Pick_JumpTo` ——
 *               那一趟是往车体上方去，路上就是转盘和已放的料，不能贴着扫。
 *               两轴都是绝对位置模式（见 pick.c）；
 * @note  先转转盘再摆臂：转盘转动时臂还在原料区，不会碰到臂。
 */
static void GrabRawBoxToCar(uint8_t box, uint8_t slot, uint8_t direct)
{
    Pick_Gripper_RawOpen();         /* 原料区下探专用张角（PICK_RAW_OPEN，与放料解耦） */
    switch(box)
    {
        case 0:  if(direct) Pick_MoveTo POS_RAW_A; else Pick_JumpTo POS_RAW_A; break;
        case 1:  if(direct) Pick_MoveTo POS_RAW_B; else Pick_JumpTo POS_RAW_B; break;
        default: if(direct) Pick_MoveTo POS_RAW_C; else Pick_JumpTo POS_RAW_C; break;
    }
    Pick_TurnTable_Set(slot);       /* 启动选槽，非阻塞：转盘后台转 */
    Pick_Gripper_Grab();            /* 夹爪收拢与转盘转动并行 */
    Pick_JumpTo POS_CAR;            /* 再摆臂（Π 形抬升，耗时远长于换槽） */
    Pick_Gripper_Release();
}

/**
 * @brief 从车上第 slot 个槽取出并放到圆环
 * @param slot      槽下标 0=A, 1=B, 2=C
 * @param ring      目标圆环编号 (1/2/3)
 * @param next_slot 下一块要用的槽下标（has_next == 0 时不看这个值）
 * @param has_next  1 = 后面还有一块要放：松爪前先把转盘转到 next_slot；
 *                  0 = 这是最后一块，放完就松爪
 * @note  转盘方案下三个槽共用 POS_CAR 一个臂位姿，原先"A 槽夹出来要经过 B 槽"
 *        的绕行过渡位(POS_CAR_A_OUT)已不需要，删掉。
 *
 *        **换槽提前到松爪之前**（只对第 1、2 块，即后面还有料要放的时候）：
 *        此时臂已经伸到圆环上方、离开了车体，转盘可以自由转下一槽。
 *        Pick_TurnTable_Set 非阻塞启动转动，与松爪、回 POS_CAR 这段完全重叠，
 *        下一块回到 POS_CAR 时槽位早就换好了 —— 全程没有任何等待。
 *        最后一块没有"下一次"，仍然一到圆环就松爪。
 */
static void PlaceToRing(uint8_t slot, uint8_t ring, uint8_t next_slot, uint8_t has_next)
{
    Pick_TurnTable_Set(slot);       /* 启动选槽，非阻塞 */
    Pick_JumpTo POS_CAR;            /* 再摆臂 */
    Pick_Gripper_Grab();
    MOVE_TO_RING(ring);
    if(has_next)
    {
        Pick_TurnTable_Set(next_slot);  /* 臂在圆环处非阻塞启动下一槽，和松爪并行 */
    }
    Pick_Gripper_Release();
}

/* ========================== 公共函数实现 ========================== */

/**
 * @brief 原料区取料前的视觉对正：摆取料位 → 开爪 → 0x02 拿偏差 → AGV_Correct 闭环 → 收爪
 * @note  由 task.c 的 `case Pick1` 在 Pick_RAW / Pick_RAW_PLUS 之前调用一次，
 *        所以 Pick_RAW 自己**不再**重复做这一步。
 *
 *        位姿用 POS_RAW_B（原料区"下"位，正对前方且最近）：相机 0x02 看的就是
 *        这个位置上的"最近物块"，在这儿对正、接下来也在这儿取料，全程只用一个位姿。
 *
 *        拿不到偏差就不修正 —— 超时的话 dx/dy 还是上一次留下的旧值，拿它驱轮子
 *        会把车带偏；车本来就已经停在原料区，原地不动、直接进抓取反而更安全。
 *        请求函数传 NULL 是因为偏差由 AGV_Correct 直接读全局 dx/dy，不必在这里接值。
 *
 *        相机安装偏置用**取料侧那套**（PICK_CAM_MOUNT_CORR_X/Y），和放置侧
 *        （Place_Correct_*）那套完全独立：原料区观察位与圆环区观察位的光轴落点
 *        不同处，共用一个数会让某一侧系统性偏半个偏置量。
 */
void Pick_Correct(void)
{
    Pick_JumpTo POS_OBSERVE_RAW_A;
    Pick_Gripper_Observe();         /* 修正期间张开，别挡住相机 */
    if(Cam_SwitchTo(02)&&cam_flag!=0)  /* 偏差落到全局 dx/dy */
    {
        AGV_CorrectWithMountCorr(3000, MOUNT_CORR_PICK);   /* 取料侧独立偏置 */
    }
    Pick_Gripper_Release();         /* 修正完了，下去抓之前收成 release 位 */
}

/**
 * @brief 原料区取料（当前路线）：在取料位等，等到取料位上那一块就是要抓的颜色才下手
 * @note  流程（严格按任务码顺序，i = 0 → 1 → 2）：
 *        1) 摆回 POS_OBSERVE_RAW_A、开爪（别挡住相机）；
 *        2) 反复发 0x02，相机回的 flag 就是"取料位上最近物块的颜色编号(1~6)"：
 *             拿到的颜色 == Pick_Color[i]  → 停，下去抓；
 *             不等（含 0x02 超时、物块还没静止）→ 接着发，**一直等，没有上限**。
 *        3) 收成 release 位 → GrabRawBoxToCar(B, i, 0)：抓 B 位那一块，放进车上第 i 槽。
 *
 *        ⚠ 为什么是死循环：原料转盘是**取走一块才自动转一格**（机械联动，不拿就不转），
 *        所以想让下一色转到取料位上，必须先把它上面那一块抓走。要是连第一块都不是
 *        Pick_Color[0]，这里就会一直等下去 —— 这是明确接受的取舍。
 *        调试期 DBG_ENABLE=1 时还能按 C8 暂停（Cam_WaitSettle 里的 HAL_Delay 会跑
 *        Dbg_Service）；正式比赛 DBG_ENABLE=0，没有逃生口，只能断电。
 *
 *        每轮开头那句 Pick_JumpTo POS_RAW_B 不能省：上一轮结束时臂停在 POS_CAR，
 *        在那儿发 0x02 相机根本看不到原料区。第一轮时臂本来就在 B 位（Pick_Correct
 *        留下的），Pick_JumpTo 内部会短路，不会白走一趟。
 *
 *        0x02 的 flag!=0 **只代表物块边缘已静止，不代表颜色是要的那一色** ——
 *        颜色必须自己拿 flag 去和 Pick_Color[i] 比，这一步是本次改动的关键。
 */
void Pick_RAW(void)
{
    uint8_t *Pick_Color = Pick_Color_List();

    for(uint8_t i = 0; i < 3; i++)
    {
        Pick_JumpTo POS_OBSERVE_RAW_A;
        for(;;)
        {
            if(Cam_SwitchTo(02) && (cam_flag == Pick_Color[i]))
            {
                break;                  /* 取料位上就是要抓的那一色 */
            }
        }
        GrabRawBoxToCar(1, i, 0);       /* box=1 即 POS_RAW_B；第 i 色 → 车上第 i 槽（门字形） */
    }
    Pick_JumpTo POS_RUN;
}

/* ===================== 原料区六种固定抓取路线 =====================
 * 前置：相机 0x03 已经报出 A/B/C 三框的**初始**颜色。每抓走一块，转盘内容按
 * 索引 −1 挪一格（A→C、C→B、B→A，2026-10-23 实车确认），所以「任务码第 i 个
 * 颜色」实际要去 (初始格 + 3 - i) % 3 抓。三个颜色都齐时，结果只可能是六种：
 *
 *   初始排列(颜色在 A/B/C 的哪一格)   实际抓取顺序   函数
 *   A B C                             A A A         Pick_RAW_ABC
 *   A C B                             A B C         Pick_RAW_ACB
 *   B A C                             B C A         Pick_RAW_BAC
 *   B C A                             B B B         Pick_RAW_BCA
 *   C A B                             C C C         Pick_RAW_CAB
 *   C B A                             C A B         Pick_RAW_CBA
 *
 * ⚠ **函数名 = 开局在 POS_OBSERVE_RAW_CENTER 观测到的初始排列**（任务码第
 * 0/1/2 色分别落在 A/B/C 哪一格），不是三次抓取实际去的框。抓哪些框写在函数体
 * 里，由上面那条 −1 规律从名字推出来。
 *
 * 抓取过程中**底盘完全不动**：三块全靠臂自己伸到 A/B/C 位去取。以后要配底盘
 * 移动再单独加，现在先留纯臂版本。
 */

typedef enum
{
    RAW_BOX_A = 0,
    RAW_BOX_B = 1,
    RAW_BOX_C = 2
} RawBox;

/* 臂动作：去 box 框抓一块，放进车上第 slot 槽。不碰底盘。
   下探前先摆到原料区专用张角 PICK_RAW_OPEN（Pick_Gripper_RawOpen）——以前这里
   直接沿用离爪后的 PICK_RELEASE，调"下探别蹭料"会把"往圆环放料"的角度一起改掉。
   已经把两者拆开：原料区只认 PICK_RAW_OPEN，放料只认 PICK_RELEASE。
   注意初始情况不同：Pick_RAW 进来时夹爪停在 release 位（=RawOpen 默认值，不动作），
   走六条路线时停在 observe 位，会多一次收拢行程（~250ms，只在第一块）。 */
static void Raw_PickAt(RawBox box, uint8_t slot)
{
    Pick_Gripper_RawOpen();         /* 原料区下探专用张角 */
    switch(box)
    {
        case RAW_BOX_A: Pick_JumpTo POS_RAW_A; break;
        case RAW_BOX_C: Pick_JumpTo POS_RAW_C; break;
        default:        Pick_JumpTo POS_RAW_B; break;
    }
    Pick_TurnTable_Set(slot);       /* 臂还在原料区时启动转盘选槽 */
    Pick_Gripper_Grab();            /* 与转盘转动并行 */
    Pick_JumpTo POS_CAR;
    Pick_Gripper_Release();
}

/* ---- 六种固定路线 ----
   每条都只调三次 Raw_PickAt（臂 + 转盘），**底盘全程不动**。
   槽位固定按 0→1→2 先后放，与任务码第 i 色对应。
   进函数时夹爪停在 Pick_RAW_PLUS 摆的 observe 位；每抓完一块停在 release
   位，可以直接下探下一块（见 Raw_PickAt 注释）。

   使用宏 PICK_RAW_PLUS_GRAB_INTERVAL_MS 控制相邻两次 Raw_PickAt 的起始时刻：
   每次调用前按上一次的开始时间补足剩余等待，不是在 Raw_PickAt 结束后固定再等
   一个完整间隔。**这段等待不再空耗**：先摆到下一块上方 30mm 的等待位
   （POS_RAW_x_APPROACH），摆位本身也算在这段间隔里，摆完再补足剩余时间。
   等待位和抓取位只差高度，所以后面的 Raw_PickAt 里 Pick_JumpTo 会短路成
   "只降高度"，不会再抬到安全高度绕一圈。
   如果一次 Raw_PickAt 本身耗时超过宏值，就不摆等待位、直接开始下一块。 */
typedef struct
{
    uint32_t last_start_tick;
    uint8_t  has_last_start;
} RawPickTiming;

/* 摆到指定框的"上方等待位"：角度/半径与抓取位相同，高度高 30mm。
   用 Pick_JumpTo（Π 形）而不是 Pick_MoveTo：这一趟是从 POS_CAR 往原料区走，
   横向扫过车体，而车上已经放了前面几块料，必须抬起来过。 */
static void Raw_MoveToApproach(RawBox box)
{
    switch(box)
    {
        case RAW_BOX_A: Pick_JumpTo POS_RAW_A_APPROACH; break;
        case RAW_BOX_C: Pick_JumpTo POS_RAW_C_APPROACH; break;
        default:        Pick_JumpTo POS_RAW_B_APPROACH; break;
    }
}

static void Raw_PickAtWithInterval(RawBox box, uint8_t slot, RawPickTiming *timing,
                                   uint32_t interval_ms)
{
    if(timing->has_last_start != 0U)
    {
        uint32_t elapsed_ms = HAL_GetTick() - timing->last_start_tick;

        if(elapsed_ms < interval_ms)
        {
            /* 间隔里先把臂摆到下一块上方等好，摆完再补足剩余时间 */
            Raw_MoveToApproach(box);

            elapsed_ms = HAL_GetTick() - timing->last_start_tick;
            if(elapsed_ms < interval_ms)
            {
                HAL_Delay(interval_ms - elapsed_ms);
            }
        }
    }

    timing->last_start_tick = HAL_GetTick();
    timing->has_last_start = 1U;
    Raw_PickAt(box, slot);
}

/* 普通六种路线用的那一版：间隔固定取 PICK_RAW_PLUS_GRAB_INTERVAL_MS */
static void Raw_PickAtWithStartInterval(RawBox box, uint8_t slot, RawPickTiming *timing)
{
    Raw_PickAtWithInterval(box, slot, timing, PICK_RAW_PLUS_GRAB_INTERVAL_MS);
}

/* ---- Fast 路线的转盘相位判断 ----
   转盘一个周期 = t1(PICK_RAW_PLUS_STATIC_MS，静止) + t2(PICK_RAW_PLUS_MOVING_MS，
   转动)，合计仍是 PICK_RAW_PLUS_GRAB_INTERVAL_MS(7200ms)。从 origin_tick
   （相机 0x03 报完三框颜色的时刻，也就是第 1 块动手之前）起算，每经过一个周期，
   所有料块按索引 −1 挪一格（A→C、C→B、B→A）。所以开局在 box 的那一块：
     正在转的周期里  → 它被带着走，抓的时候已经在下一位；
     转过 n 个整周期 → 它在 (box − n) 那一格。
   这里统一按"已经转过几格"换算，t1/t2 只决定"现在能不能抓"。 */
static RawBox Raw_ShiftBox(RawBox box, uint8_t turns)
{
    return (RawBox)(((uint8_t)box + 3U - (turns % 3U)) % 3U);
}

/* Fast 路线第 2 / 3 块：紧接上一块出发，目标块开局在 box0。
   origin_tick 是**这一块自己的计时基准**：第 2 块传相机读数时刻，第 3 块传
   它 + 一个整周期（调用处加的）。摆到 box0 上方 30mm（悬挂高度）后：
     **还没到 origin_tick → 先原地等到 origin_tick**（第 3 块因此会等满一个
     周期再动手）；到了之后再按相位判断：
     elapsed 落在静止窗口(phase < t1)
         → 转盘没在动，它就在"已转过 turns 格"的位置，直接抓
           （turns 为 0 就是还在 box0；已经超过整周期时位置自然跟着变）；
     elapsed 落在运动窗口(phase >= t1)
         → 它马上会被带到下一格：**先摆到下一格上方悬挂**，等这个周期转完
           (elapsed 到 t1+t2 的整数倍) 再抓。
   最后保证在悬挂位停够 PICK_RAW_APPROACH_SETTLE_MS，等大幅旋转的晃动衰减。 */
static void Raw_PickAtFollow(RawBox box0, uint8_t slot, uint32_t origin_tick)
{
    const uint32_t cycle = (uint32_t)PICK_RAW_PLUS_GRAB_INTERVAL_MS;
    uint32_t elapsed_ms;
    uint32_t phase;
    uint32_t arrived_ms;
    uint8_t  turns;
    RawBox   box;

    Raw_MoveToApproach(box0);            /* 先到开局位置上方悬挂，到这里才判相位 */
    arrived_ms = HAL_GetTick();

    /* 还没到这一块的基准时刻（第 3 块传的是 origin_tick + 一个周期）：先等到基准。
       ⚠ 不能像旧写法那样把负的 elapsed 直接钳成 0 —— 那会被当成"静止窗口、
       转过 0 格"，第 3 块刚到就抓，等待就消失了。 */
    {
        int32_t early_ms = (int32_t)(origin_tick - HAL_GetTick());

        if(early_ms > 0)
        {
            HAL_Delay((uint32_t)early_ms);
        }
    }

    elapsed_ms = HAL_GetTick() - origin_tick;
    turns      = (uint8_t)(elapsed_ms / cycle);      /* 已经转过的整格数 */
    phase      = elapsed_ms % cycle;

    if(phase >= (uint32_t)PICK_RAW_PLUS_STATIC_MS)
    {
        /* 正在转：这一块会被带到下一格 → 先跟到下一格上方悬挂 */
        box = Raw_ShiftBox(box0, (uint8_t)(turns + 1U));
        Raw_MoveToApproach(box);
        arrived_ms = HAL_GetTick();

        /* 再等它转完（移动本身也花了时间，重新取相位） */
        phase = (HAL_GetTick() - origin_tick) % cycle;
        if(phase >= (uint32_t)PICK_RAW_PLUS_STATIC_MS)
        {
            HAL_Delay(cycle - phase);
        }
    }
    else
    {
        /* 静止窗口：它现在在"转过 turns 格"的位置 */
        box = Raw_ShiftBox(box0, turns);
        if(box != box0)
        {
            /* 已经转过整格，走之前那一趟同样是大角度旋转/变半径 */
            Raw_MoveToApproach(box);
            arrived_ms = HAL_GetTick();
        }
    }

    /* 在悬挂位停够，等晃动衰减 */
    elapsed_ms = HAL_GetTick() - arrived_ms;
    if(elapsed_ms < PICK_RAW_APPROACH_SETTLE_MS)
    {
        HAL_Delay(PICK_RAW_APPROACH_SETTLE_MS - elapsed_ms);
    }

    Raw_PickAt(box, slot);
}

void Pick_RAW_ABC(void)     /* 初始观测 A B C → 三次都抓 A 框 */
{
    RawPickTiming timing = {0U, 0U};
    Raw_PickAtWithStartInterval(RAW_BOX_A, 0, &timing);
    Raw_PickAtWithStartInterval(RAW_BOX_A, 1, &timing);  /* 同框：转盘已把下一块送到 A */
    Raw_PickAtWithStartInterval(RAW_BOX_A, 2, &timing);
}

void Pick_RAW_ACB(void)     /* 初始观测 A C B → A → B → C */
{
    RawPickTiming timing = {0U, 0U};
    Raw_PickAtWithStartInterval(RAW_BOX_A, 0, &timing);
    Raw_PickAtWithStartInterval(RAW_BOX_B, 1, &timing);
    Raw_PickAtWithStartInterval(RAW_BOX_C, 2, &timing);
}

void Pick_RAW_BAC(void)     /* 初始观测 B A C → B → C → A */
{
    RawPickTiming timing = {0U, 0U};
    Raw_PickAtWithStartInterval(RAW_BOX_B, 0, &timing);
    Raw_PickAtWithStartInterval(RAW_BOX_C, 1, &timing);
    Raw_PickAtWithStartInterval(RAW_BOX_A, 2, &timing);
}

void Pick_RAW_BCA(void)     /* 初始观测 B C A → 三次都抓 B 框 */
{
    RawPickTiming timing = {0U, 0U};
    Raw_PickAtWithStartInterval(RAW_BOX_B, 0, &timing);
    Raw_PickAtWithStartInterval(RAW_BOX_B, 1, &timing);
    Raw_PickAtWithStartInterval(RAW_BOX_B, 2, &timing);
}

void Pick_RAW_CAB(void)     /* 初始观测 C A B → 三次都抓 C 框 */
{
    RawPickTiming timing = {0U, 0U};
    Raw_PickAtWithStartInterval(RAW_BOX_C, 0, &timing);
    Raw_PickAtWithStartInterval(RAW_BOX_C, 1, &timing);
    Raw_PickAtWithStartInterval(RAW_BOX_C, 2, &timing);
}

void Pick_RAW_CBA(void)     /* 初始观测 C B A → C → A → B */
{
    RawPickTiming timing = {0U, 0U};
    Raw_PickAtWithStartInterval(RAW_BOX_C, 0, &timing);
    Raw_PickAtWithStartInterval(RAW_BOX_A, 1, &timing);
    Raw_PickAtWithStartInterval(RAW_BOX_B, 2, &timing);
}

/* ---- 六种快速路线（Fast） ----
   与上面六种普通路线一一对应（名字同样是**开局观测到的初始排列**）。
   转盘规律同普通路线：每过一个周期按索引 −1 挪一格（A→C、C→B、B→A）。
   三块的规则（第 1 块之外的 2、3 块都按 elapsed 与 origin_tick 比较）：
     第 1 块：抓 c1 初始所在框，**不看时间**（origin_tick 在这之前取）；
     第 2 块：紧接第 1 块出发，目标 c2 开局所在框；**到达悬挂高度(上方 30mm)
             时才判断**（Raw_PickAtFollow）：
                 elapsed < t1                → 转盘静止，位置没变，直接抓；
                 t1 < elapsed < t1+t2        → 正被带着走，**先摆到下一格上方
                                               悬挂，等到 t1+t2 再抓**；
                 elapsed > t1+t2             → 已经转过格，**直接摆到下一格上方
                                               悬挂抓取**（不额外等）；
     第 3 块：同样按相位比较，但**基准是 origin_tick + 一个整周期**（这一块的
             计时从第 2 块那一轮的起点往后推一轮），在"静止时间对应的位置"抓。
   目标框 = 开局框按索引 −1 走过"已过周期数"格（Raw_ShiftBox）；
   第 3 块的"已过周期数"从 origin_tick + 一个周期算起，所以比第 2 块晚一轮。
   下面注释里的抓取顺序是按各块都落在第一个静止窗口（最快情况）写的：
       观测 ABC → A B B      观测 ACB → A C A      观测 BAC → B A B
       观测 BCA → B C C      观测 CAB → C A A      观测 CBA → C B C
   实际位置随 elapsed 自动修正，不会抓空。 */

void Pick_RAW_ABC_Fast(uint32_t origin_tick)    /* 初始观测 A B C */
{
    Raw_PickAt(RAW_BOX_A, 0);                        /* 第 1 块：c1 开局所在框，不看时间 */
    Raw_PickAtFollow(RAW_BOX_B, 1, origin_tick);     /* 第 2 块：c2 开局所在框 + 相位判断 */
    Raw_PickAtFollow(RAW_BOX_B, 2, origin_tick + PICK_RAW_PLUS_GRAB_INTERVAL_MS);     /* 第 3 块：基准后移一个周期 */
}

void Pick_RAW_ACB_Fast(uint32_t origin_tick)    /* 初始观测 A C B */
{
    Raw_PickAt(RAW_BOX_A, 0);
    Raw_PickAtFollow(RAW_BOX_C, 1, origin_tick);
    Raw_PickAtFollow(RAW_BOX_A, 2, origin_tick + PICK_RAW_PLUS_GRAB_INTERVAL_MS);
}

void Pick_RAW_BAC_Fast(uint32_t origin_tick)    /* 初始观测 B A C */
{
    Raw_PickAt(RAW_BOX_B, 0);
    Raw_PickAtFollow(RAW_BOX_A, 1, origin_tick);
    Raw_PickAtFollow(RAW_BOX_B, 2, origin_tick + PICK_RAW_PLUS_GRAB_INTERVAL_MS);
}

void Pick_RAW_BCA_Fast(uint32_t origin_tick)    /* 初始观测 B C A */
{
    Raw_PickAt(RAW_BOX_B, 0);
    Raw_PickAtFollow(RAW_BOX_C, 1, origin_tick);
    Raw_PickAtFollow(RAW_BOX_C, 2, origin_tick + PICK_RAW_PLUS_GRAB_INTERVAL_MS);
}

void Pick_RAW_CAB_Fast(uint32_t origin_tick)    /* 初始观测 C A B */
{
    Raw_PickAt(RAW_BOX_C, 0);
    Raw_PickAtFollow(RAW_BOX_A, 1, origin_tick);
    Raw_PickAtFollow(RAW_BOX_A, 2, origin_tick + PICK_RAW_PLUS_GRAB_INTERVAL_MS);
}

void Pick_RAW_CBA_Fast(uint32_t origin_tick)    /* 初始观测 C B A */
{
    Raw_PickAt(RAW_BOX_C, 0);
    Raw_PickAtFollow(RAW_BOX_B, 1, origin_tick);
    Raw_PickAtFollow(RAW_BOX_C, 2, origin_tick + PICK_RAW_PLUS_GRAB_INTERVAL_MS);
}

/**
 * @brief 原料区三框的一次性观测：摆观察位 → 0x03 → 算出任务码三色开局各落在哪个框
 * @param pos 输出：pos[i] = 任务码第 i 色**开局**所在框（1=A 2=B 3=C）
 * @retval 颜色读到的时刻(HAL_GetTick)。Fast 路线的转盘相位(静止/运动窗口)
 *         从这里起算 —— 第 2、3 块都按它换算"这块现在走到哪一格"。
 * @note  普通路线 Pick_RAW_PLUS 和快速路线 Pick_RAW_PLUS_FAST 共用这一份观测：
 *        1) 摆 POS_OBSERVE_RAW_CENTER（高位、收臂，不挡相机），张开夹爪；
 *        2) **只问一次** 0x03，拿到三个框的初始颜色，下标 0/1/2 = A/B/C 框；
 *        3) 对任务码第 i 个颜色，记下它开局落在哪个框。
 *
 *        为什么能一次问完就算出来：原料转盘每抓走一块固定挪一格（方向已实测为
 *        索引 −1），三块的颜色排列因此只有 6 种，开局一次读数就足以定下整条路线。
 *        三块各占一格，颜色一定齐，所以不做缺色处理。
 *        只有 0x03 本身超时（相机没响应）时 box_color[] 还是上一轮的旧值，
 *        照着抓会连错三块 —— 这是已知的取舍（相机不应答时整轮都不可信）。
 *        0x02 的视觉对正已搬到 Pick_Correct()，这里不做。
 */
static uint32_t Raw_ObserveThreeBoxes(uint8_t pos[3])
{
    uint8_t *Pick_Color = Pick_Color_List();
    uint8_t  box[3];                    /* 开局读一次的三框颜色: 0=A 1=B 2=C */

    Pick_JumpTo POS_OBSERVE_RAW_CENTER;   /* 要看清三个框，得摆到观察位 */
    Pick_Gripper_Observe();             /* 张开，别挡住相机 */
    HAL_Delay(600);                     /* 等相机输出稳定(上电/切模式后头几帧不可信) */

    Cam_SwitchTo(03);                   /* 相机没响应就不猜，直接收工 */

    for(uint8_t k = 0; k < 3U; k++)
    {
        box[k] = box_color[k];          /* 立刻拷走：box_color[] 会被后续帧刷新 */
    }

    for(uint8_t i = 0; i < 3U; i++)
    {
        uint8_t k = 0U;
        while(k < 3U && box[k] != Pick_Color[i]) k++;   /* 这一色初始在第 k 格 */

        pos[i] = (uint8_t)(k + 1U);     /* 转成 1=A 2=B 3=C */
    }

    return HAL_GetTick();               /* 颜色已拿到：Fast 路线的转盘相位从这儿起算 */
}

/**
 * @brief 原料区取料（普通路线）：一次观测三框颜色 → 按初始排列分派给六个普通函数
 * @note  六条路线内部**底盘完全不动**，只动臂和转盘；相邻两块按
 *        PICK_RAW_PLUS_GRAB_INTERVAL_MS 的起始间隔走，等待期间会先摆到下一块
 *        上方 30mm 的等待位（见 Raw_PickAtWithInterval）。
 *        调用关系：task.c 的 `case Pick1` 直接调用本函数。
 */
void Pick_RAW_PLUS(void)
{
    uint8_t pos[3];                     /* 任务码第 i 色开局落在哪个框：1=A 2=B 3=C */

    Raw_ObserveThreeBoxes(pos);

    /* 六种初始排列查表，函数名就是这串数（见上面的路线表） */
    if     (pos[0] == 1U && pos[1] == 2U && pos[2] == 3U) Pick_RAW_ABC();
    else if(pos[0] == 1U && pos[1] == 3U && pos[2] == 2U) Pick_RAW_ACB();
    else if(pos[0] == 2U && pos[1] == 1U && pos[2] == 3U) Pick_RAW_BAC();
    else if(pos[0] == 2U && pos[1] == 3U && pos[2] == 1U) Pick_RAW_BCA();
    else if(pos[0] == 3U && pos[1] == 1U && pos[2] == 2U) Pick_RAW_CAB();
    else if(pos[0] == 3U && pos[1] == 2U && pos[2] == 1U) Pick_RAW_CBA();

    Pick_JumpTo POS_RUN;
}

/**
 * @brief 原料区取料（快速路线）：一次观测三框颜色 → 按初始排列分派给六个 Fast 函数
 * @note  观测流程与 Pick_RAW_PLUS 完全一样（共用 Raw_ObserveThreeBoxes）；
 *        差别在六条 Fast 路线：第 2、3 块都在**到达悬挂高度时**按转盘相位
 *        （t1 静止 / t2 转动）判断在哪个框抓、要不要等，见 Raw_PickAtFollow。
 *        观测函数返回的"读到颜色的时刻"会原样传给 Fast 函数，作为相位起算点。
 *        原来的 PICK_RAW_PLUS_USE_FAST 编译开关已拆掉，本函数单独存在 ——
 *        要切路线，在上层把 Pick_RAW_PLUS 换成 Pick_RAW_PLUS_FAST 即可。
 */
void Pick_RAW_PLUS_FAST(void)
{
    uint8_t  pos[3];                    /* 任务码第 i 色开局落在哪个框：1=A 2=B 3=C */
    uint32_t origin_tick;               /* 相机报完三框颜色的时刻 */

    origin_tick = Raw_ObserveThreeBoxes(pos);

    if     (pos[0] == 1U && pos[1] == 2U && pos[2] == 3U) Pick_RAW_ABC_Fast(origin_tick);
    else if(pos[0] == 1U && pos[1] == 3U && pos[2] == 2U) Pick_RAW_ACB_Fast(origin_tick);
    else if(pos[0] == 2U && pos[1] == 1U && pos[2] == 3U) Pick_RAW_BAC_Fast(origin_tick);
    else if(pos[0] == 2U && pos[1] == 3U && pos[2] == 1U) Pick_RAW_BCA_Fast(origin_tick);
    else if(pos[0] == 3U && pos[1] == 1U && pos[2] == 2U) Pick_RAW_CAB_Fast(origin_tick);
    else if(pos[0] == 3U && pos[1] == 2U && pos[2] == 1U) Pick_RAW_CBA_Fast(origin_tick);

    Pick_JumpTo POS_RUN;
}

/* ---- 粗加工区低位对正后的固定角度前馈 ----
   Place_Correct_Low() 的视觉闭环只修 dx/dy（位置），修不了"车体相对圆环还差一个
   固定角度"这种机械偏角；实测每轮都差同一个方向同一个量，所以在这里补一次开环补角。
   正负号与 AGV_SetYaw 的目标角同向：实车越补越歪就把 +0.5f 改成 -0.5f。
   改成 0.0f 即关闭（AGV_TrimYaw 内部会直接返回，连 CAN 都不发）。 */
#define PLACE_CORRECT_LOW_YAW_FF_DEG   (0.0f)

/**
 * @brief 粗加工区放置前的视觉对正【低位】：摆圆环观察位 → 开爪 → 0x04 拿偏差 → AGV_Correct 闭环 → 固定补角 → 收爪
 * @note  这是三步对正里的**最后一步**，也负责收爪：
 *          Place_Correct_High()  → 高位粗修正（不收爪）
 *          Place_Correct_Angle() → 移位量角度、航向复位（不碰夹爪）
 *          Place_Correct_Low()   → 本位细修正 + 固定补角 + 收爪，然后才 Place_ROUGH()
 *        也只单独调它（第二轮暂存区 case Place2 就走这一条）。
 *        所以 Place_ROUGH 自己**不再**重复做视觉这一步（和 Pick_Correct / Pick_RAW 一样的分工）。
 *
 *        请求必须发在"摆到观察位"之后 —— 相机得先看见圆环才有东西可识别。
 *        环号在这段用不上（环号由二维码的 Place_Color[] 给出），只要 dx/dy 落全局给
 *        AGV_Correct 用。
 *
 *        拿不到偏差就不修正 —— 超时的话 dx/dy 还是上一次留下的旧值，拿它驱轮子
 *        会把车带偏；车本来就停在粗加工区，原地不动、直接进放置反而更安全。
 *
 *        补角(PLACE_CORRECT_LOW_YAW_FF_DEG)是**开环**的机械常数补偿：位置闭环只看
 *        得见 dx/dy，看不见"车体相对圆环固定歪了半度"。走 AGV_TrimYaw() 而不是
 *        AGV_SetYaw() —— 后者在小误差下会被 (uint16_t) 速度截断卡住、且没有超时。
 */
void Place_Correct_Low(void)
{
    Pick_JumpTo POS_OBSERVE_RING_LOW;     /* 圆环区观察位：相机 0x04 看的就是这儿 */
    Pick_Gripper_Observe();             /* 修正期间张开，别挡住相机 */
    if(Cam_SwitchTo(04))                /* 偏差落到全局 dx/dy */
    {
        AGV_Correct(3000);
    }
    /* 位置闭环只修 dx/dy，修不了"车体相对圆环固定差一个角"，所以再补一次固定前馈。
       相机这次没识别成功(上面没进 if)也照补：偏角是机械常数，跟识别成功与否无关。 */
    AGV_TrimYaw(PLACE_CORRECT_LOW_YAW_FF_DEG);
    /* 这里不要清行进偏差：这 0.5° 是**有意**加的偏置，不是误差；
       而到达这里之前直行跑偏的那部分该留到下次转弯补掉。 */
    Pick_Gripper_Release();             /* 修正完了，下去放之前收成 release 位 */
}

/* ---- Place_Correct_High / Place_Correct_Angle 的参数 ----
   对正拆成三段，顺序固定：High → Angle → Low → Place_ROUGH()。
     High  高位观察位(POS_OBSERVE_RING_HIGH)上做圆环2 粗修正，1s 就够，
           给太久反而会被单个误识别的圆环带着跑偏；
     Angle 此时臂还停在同一个高位观察位，把底盘沿车头方向挪开 7cm 再量角度：
           挪开后三个圆环在相机视野里排得更开，0x06 报的偏角才准。量完把
           对正后的航向记成基准 a1，退回原位，最后复位到 a1；
     Low   回低位观察位(POS_OBSERVE_RING_LOW)做长时间细修正，收爪。

   角度这一段是**闭环**：不是读一次转一次就完事，而是
     读 0x06 → 按 GAIN 转 SGN*err → 再读 0x06 → ……
   直到 |err| 进死区(ANGLE_DB)或迭代用完。单次修正会残留一部分误差（相机的
   偏角本身有噪声、AGV_SetYaw 也有到位死区），叠到放置时就是斜着下爪。
   `PLACE_CORRECT_ANGLE_SGN` 跟相机侧的约定绑定：实车越修越歪就取反。 */
#define PLACE_CORRECT_ANGLE_SHIFT_MM        70U     /* 移位距离：沿车头方向 7cm */
#define PLACE_CORRECT_ANGLE_SHIFT_DIR       1U      /* GoStraight: 1=向前(当作"左")，0=向后(当作"右") */
#define PLACE_CORRECT_ANGLE_SGN             (-1.0f) /* 实车越修越歪 → 取反（已按实车改成 -1）*/
#define PLACE_CORRECT_ANGLE_GAIN            (1.0f)  /* 外环比例增益，>1 收敛快但会过冲 */
#define PLACE_CORRECT_ANGLE_DB              (0.3f)  /* 死区(°)：残差小于它就认为对正了 */
#define PLACE_CORRECT_ANGLE_ITERS           5U      /* 最多迭代几轮，防止来回摆 */
#define PLACE_CORRECT_ANGLE_SETTLE_MS       1000U   /* 每轮转完等车停稳、相机出新图 */

/**
 * @brief 粗加工区对正【第一步·高位粗修正】：摆高位观察位 → 开爪 → 0x04 圆环2 粗修正
 * @note  只做位置粗修正，**不收爪**：下一步 Place_Correct_Angle 还在同一个观察位
 *        用相机量角度，夹爪收起来会挡视线。收爪由流程最后一步 Place_Correct_Low 负责。
 *
 *        拿不到 0x04 的结果就不修正 —— 超时时 dx/dy 还是旧值，拿它驱轮子会把车带偏。
 */
void Place_Correct_High(void)
{
    Pick_JumpTo POS_OBSERVE_RING_HIGH;  /* 高位：三个圆环同时进视野 */
    Pick_Gripper_Observe();             /* 修正期间张开，别挡住相机 */

    if(Cam_SwitchTo(04))                /* 偏差落到全局 dx/dy */
    {
        AGV_Correct(400);
    }
}

/**
 * @brief 粗加工区对正【第二步·角度闭环】：挪开 7cm → 0x06 闭环对正 → 记基准 a1
 *        → 退回原位 → 航向复位 a1
 * @note  进函数时臂停在 Place_Correct_High 摆好的 POS_OBSERVE_RING_HIGH、夹爪张开，
 *        本函数不碰臂也不碰夹爪，只动底盘和航向。
 *
 *        1) 沿车头方向挪 PLACE_CORRECT_ANGLE_SHIFT_MM(7cm)：三个圆环在视野里
 *           排得更开，0x06 报的偏角更准；那里不是放料位，所以量完就退回来。
 *        2) 角度闭环：反复 0x06 → AGV_SetYaw(当前 + SGN*GAIN*err)，直到 |err|
 *           进 PLACE_CORRECT_ANGLE_DB 或迭代到 ANGLE_ITERS；0x06 超时直接跳出
 *           （不拿旧值闭环）。
 *        3) 把对正后的**实际**航向记成基准 a1 —— 不是把上一轮算出来的目标值当
 *           a1，那样相机噪声会一点点累积。
 *        4) 退回原位（方向取反），再把航向复位到 a1。
 *
 *        ⚠ 角度是**增量**：目标航向 = 当前航向 + SGN*GAIN*cam_angle。
 */
void Place_Correct_Angle(void)
{
    float a1;

    /* ---- 1) 挪开 7cm，站到角度看得准的位置 ---- */
    AGV_Position_GoStraight(PLACE_CORRECT_ANGLE_SHIFT_MM, Vel, PLACE_CORRECT_ANGLE_SHIFT_DIR);

    /* ---- 2) 角度闭环：读到误差进死区为止 ---- */
    for(uint8_t n = 0; n < PLACE_CORRECT_ANGLE_ITERS; n++)
    {
        float err, abs_err;

        HAL_Delay(PLACE_CORRECT_ANGLE_SETTLE_MS);   /* 车停稳、相机出新图 */
        if(!Cam_SwitchTo(06)) break;                /* 超时：不拿旧值闭环 */

        err     = PLACE_CORRECT_ANGLE_SGN * (float)cam_angle;
        abs_err = (err < 0.0f) ? -err : err;
        if(abs_err < PLACE_CORRECT_ANGLE_DB) break; /* 已经对正，不用再动 */

        AGV_SetYaw(HWT101_GetYaw() + PLACE_CORRECT_ANGLE_GAIN * err);
    }

    /* ---- 3) 把对正后的实际航向记成基准 a1 ---- */
    a1 = HWT101_GetYaw();

    /* ---- 4) 退回原放料位，并把航向复位到 a1 ---- */
    AGV_Position_GoStraight(PLACE_CORRECT_ANGLE_SHIFT_MM, Vel,
                            (uint8_t)(PLACE_CORRECT_ANGLE_SHIFT_DIR ^ 1U));
    AGV_SetYaw(a1);
    /* 相机刚把航向绝对对正过：对正之前记下的行进偏差已经作废，清掉，
       否则会到下次转弯里再补一遍。停位残差(yawLastResid)不动，那是机械属性。 */
    Yaw_ClearMovingDelta();
}

/**
 * @brief 粗加工区放置：车上三个槽依次放到二维码指定的圆环上
 * @note  视觉对正已搬到 Place_Correct_High/Angle/Low，这里只管放。
 *
 *        三次放置：A→环, B→环, C→环（取料顺序 A 先出来）
 *        槽号 i 始终是「槽位下标」(0=A,1=B,2=C)，和 Pick_ROUGH / Place_TEMP /
 *        Pick_Color[] 同一套约定，这样放置完再拾取物块才能回到原槽位。
 *
 *        第 1、2 块放完松爪前就把转盘转到下一槽（见 PlaceToRing 的 next_slot），
 *        第 3 块后面没有料了，直接放。换槽顺序仍是 0→1→2。
 */
void Place_ROUGH(void)
{
    uint8_t *Place_Color = (RerunFlag == 0) ? Place_Color1 : Place_Color2;

    for (int i = 0; i < 3; i++) {
        uint8_t has_next = (uint8_t)(i < 2);
        PlaceToRing((uint8_t)i, Place_Color[i], (uint8_t)((i + 1) % 3), has_next);
    }
}

void Pick_ROUGH(void)
{
    /* 必须和本轮 Place_ROUGH 用同一个数组：第二轮 Place_ROUGH 按 Place_Color2 放，
       这里也按 Place_Color2 取，槽 i 才会回到本轮该有的颜色 Pick_Color2[i]；
       否则两轮环号不同时槽里的颜色会被置换，害得后面 Place_TEMP 的同色同环对不上。 */
    uint8_t *Place_Color = (RerunFlag == 0) ? Place_Color1 : Place_Color2;

    GRAB_FROM_RING_AND_PLACE_SLOT(Place_Color[0], 0);   /* → A 槽 */
    GRAB_FROM_RING_AND_PLACE_SLOT(Place_Color[1], 1);   /* → B 槽 */
    GRAB_FROM_RING_AND_PLACE_SLOT(Place_Color[2], 2);   /* → C 槽 */

    Pick_JumpTo POS_RUN;
}

/**
 * @brief 第二轮暂存区码垛用的环号：按第一轮「物块颜色 → 环位」的对应关系，同色放到同一个环(高位)
 * @param slot 第二轮槽位下标 0=A 槽, 1=B 槽, 2=C 槽（与 Pick_Color2[]/Place_Color2[] 一致）
 * @note  例：第一轮 4 号色放在 2 号环 → 第二轮 4 号色也放 2 号环的高位。
 *        两轮任务码的颜色种类一致，必能匹配上，不会走到最后的 return。
 */
static uint8_t Place_RingSameAsLastRound(uint8_t slot)
{
    for (uint8_t j = 0; j < 3; j++) {
        if (Pick_Color1[j] == Pick_Color2[slot]) return Place_Color1[j];
    }
    return Place_Color2[slot];      /* 兜底：正常不会到 */
}

/**
 * @brief 暂存区放置前的视觉对正【第二轮】：摆暂存区观察位 → 开爪 → 0x05 指定颜色 → AGV_Correct → 收爪
 * @note  第一轮不走这里 —— 暂存区第一轮的观察位和粗加工区是**同一处**、同用 0x04 请求圆环，
 *        直接复用 Place_Correct_Low()。两边由上层在 task.c 的 `case Place2` 里按 RerunFlag 二选一：
 *          RerunFlag == 0 → Place_Correct_Low()
 *          RerunFlag == 1 → Place_TEMP_Correct()
 *
 *        发哪一色不用在这儿算：Cam_SwitchTo(05) 内部按任务码取"第一轮放进 2 号环的
 *        那一块的颜色"（与 Place_RingSameAsLastRound 同一套对应关系，只是反过来查）。
 *        任务码里没有哪一槽进过 2 号环 → 它返回 0、一帧都不发，不做修正。
 *
 *        拿不到偏差就不修正 —— 超时的话 dx/dy 还是上一次留下的旧值，拿它驱轮子会把车带偏。
 */
void Place_TEMP_Correct(void)
{
    Pick_JumpTo POS_OBSERVE_RING_MIDDLE;       /* 暂存区观察位 */
    Pick_Gripper_Observe();             /* 修正期间张开，别挡住相机 */
    Cam_Service();                      /* 原 Place_TEMP 里就有这句接收链自愈，照搬过来 */
    if(Cam_SwitchTo(05))
    {
        AGV_Correct(2500);
    }
    Pick_Gripper_Release();             /* 修正完了，下去放之前收成 release 位 */
}

/**
 * @brief 暂存区放置（第二轮码垛）：车上三槽按第一轮的「颜色→环位」关系放到对应环的高位
 * @note  视觉对正已搬走（第一轮 Place_Correct_Low / 第二轮 Place_TEMP_Correct），这里只管放。
 *
 *        每轮用哪个颜色数组由 RerunFlag 定：第一轮 Place_Color1，第二轮换成本轮算出的
 *        ring_reuse[]（见下面注释）。
 */
void Place_TEMP(void)
{
    /* 第二轮暂存区：环号不按第二轮任务码，而是按第一轮「颜色→环位」的对应关系同色同环码垛 */
    uint8_t ring_reuse[3];
    for (int i = 0; i < 3; i++) {
        ring_reuse[i] = Place_RingSameAsLastRound(i);
    }

    uint8_t *Place_Color = (RerunFlag == 0) ? Place_Color1 : ring_reuse;
    uint8_t is_high = (RerunFlag == 1);

    // 三次放置：A→环, B→环, C→环（取料顺序 A 先出来）
    GRAB_AND_PLACE_TO_RING_SLOT_EX(0, Place_Color[0], is_high);   /* A 槽 */
    GRAB_AND_PLACE_TO_RING_SLOT_EX(1, Place_Color[1], is_high);   /* B 槽 */
    GRAB_AND_PLACE_TO_RING_SLOT_EX(2, Place_Color[2], is_high);   /* C 槽 */

    Pick_JumpTo POS_RUN;
}
