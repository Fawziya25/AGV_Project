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
 *                    （Pick_RAW_AAA/BBB/CCC/ABC/BCA/CAB）：抓 A/C 位配底盘移动
 *                    （同 Pick_RAW_A/C 的去程/归位顺序），同框连抓的三次之间底盘不动，
 *                    换框只走两框偏移之差、不回中位 —— 比逐块重问快得多
 * 详见各函数自己的注释。
 *
 * ── 2026-10-16 放置侧同样拆开 ──────────────────────────────────────
 * 照原料区那套分工，把视觉对正从两个 Place_xxx 里摘出来，只留下放置动作：
 *   Place_Correct_High()   高位观察位 → 开爪 → 0x04 圆环2 粗修正(1s)，不收爪
 *   Place_Correct_Angle()  挪开 7cm → 0x06 角度闭环 → 记基准 a1 → 退回 → 航向复位 a1
 *   Place_Correct_Low()    低位观察位 → 0x04 细修正(30s) → 收爪
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
    switch(box)
    {
        case 0:  if(direct) Pick_MoveTo POS_RAW_A; else Pick_JumpTo POS_RAW_A; break;
        case 1:  if(direct) Pick_MoveTo POS_RAW_B; else Pick_JumpTo POS_RAW_B; break;
        default: if(direct) Pick_MoveTo POS_RAW_C; else Pick_JumpTo POS_RAW_C; break;
    }
    Pick_TurnTable_Set(slot);       /* 先选槽 */
    Pick_Gripper_Grab();
    Pick_JumpTo POS_CAR;            /* 再摆臂（往车上走一律抬起来） */
    Pick_Gripper_Release();
}

/**
 * @brief 从车上第 slot 个槽取出并放到圆环
 * @param slot 槽下标 0=A, 1=B, 2=C
 * @param ring 目标圆环编号 (1/2/3)
 * @note  转盘方案下三个槽共用 POS_CAR 一个臂位姿，原先"A 槽夹出来要经过 B 槽"
 *        的绕行过渡位(POS_CAR_A_OUT)已不需要，删掉。
 */
static void PlaceToRing(uint8_t slot, uint8_t ring)
{
    Pick_TurnTable_Set(slot);       /* 先选槽 */
    Pick_JumpTo POS_CAR;            /* 再摆臂 */
    Pick_Gripper_Grab();
    MOVE_TO_RING(ring);
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
 */
void Pick_Correct(void)
{
    Pick_JumpTo POS_OBSERVE_RAW_A;
    Pick_Gripper_Observe();         /* 修正期间张开，别挡住相机 */
    if(Cam_SwitchTo(02)&&cam_flag!=0)  /* 偏差落到全局 dx/dy */
    {
        AGV_Correct(3000);
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

/* ===================== 原料区三个抓取位 =====================
 * A/B/C 三框的取料动作拆成三个函数：现在差别只在"要不要挪底盘、臂摆哪个位姿"，
 * 以后三处要加不同动作时各改各的，不会互相牵连。
 * 三个函数都完成完整的"抓一块 → 放到车上第 slot 槽"，调用方不用再补动作。
 */

/**
 * @brief 从原料区 A 位抓一块，放到车上第 slot 槽
 * @param slot 车上槽下标 0=A, 1=B, 2=C
 * @note  先挪底盘再动臂：A 位在侧前方，底盘要**先纵向、再横向**挪到位才够得着；
 *        抓完放上车的动作和 B 位一样；最后按**相反顺序**（先横向、后纵向）退回来，
 *        下一块和后续导航的基准才不变（来回都是开环位移，等距反向，净值 0）。
 */
void Pick_RAW_A(uint8_t slot)
{
    AGV_Position_GoStraight(PICK_RAW_STRAIGHT_MM, Vel, PICK_RAW_STRAIGHT_A_DIR);  /* 先纵向 7cm */
    AGV_Position_Translate(PICK_RAW_SIDE_MM, Vel, PICK_RAW_SIDE_A_DIR);           /* 再横向 3cm */

    Pick_JumpTo POS_RAW_A;
    Pick_TurnTable_Set(slot);       /* 臂还在原料区时转盘选槽，不会碰到臂 */
    Pick_Gripper_Grab();
    Pick_JumpTo POS_CAR;
    Pick_Gripper_Release();

    /* 归位：先横向、后纵向，和去程反过来，避免斜着退 */
    AGV_Position_Translate(PICK_RAW_SIDE_MM, Vel, (uint8_t)(PICK_RAW_SIDE_A_DIR ^ 1U));
    AGV_Position_GoStraight(PICK_RAW_STRAIGHT_MM, Vel, (uint8_t)(PICK_RAW_STRAIGHT_A_DIR ^ 1U));
}

/**
 * @brief 从原料区 B 位抓一块，放到车上第 slot 槽
 * @param slot 车上槽下标 0=A, 1=B, 2=C
 * @note  B 位就在车头正前方，底盘不用动 —— 三个位置里最短的一条。
 */
void Pick_RAW_B(uint8_t slot)
{
    Pick_JumpTo POS_RAW_B;
    Pick_TurnTable_Set(slot);
    Pick_Gripper_Grab();
    Pick_JumpTo POS_CAR;
    Pick_Gripper_Release();
}

/**
 * @brief 从原料区 C 位抓一块，放到车上第 slot 槽
 * @param slot 车上槽下标 0=A, 1=B, 2=C
 * @note  和 A 对称：底盘纵向、横向都走另一侧，抓 POS_RAW_C，放完按相反顺序退回来。
 *        现在和 Pick_RAW_A 只差方向/位姿；以后要是两侧不对称（场地偏了、
 *        料架位置不一样），改这个函数就行，别去动 A。
 */
void Pick_RAW_C(uint8_t slot)
{
    AGV_Position_GoStraight(PICK_RAW_STRAIGHT_MM, Vel, PICK_RAW_STRAIGHT_C_DIR);  /* 先纵向 7cm */
    AGV_Position_Translate(PICK_RAW_SIDE_MM, Vel, PICK_RAW_SIDE_C_DIR);           /* 再横向 3cm */

    Pick_JumpTo POS_RAW_C;
    Pick_TurnTable_Set(slot);
    Pick_Gripper_Grab();
    Pick_JumpTo POS_CAR;
    Pick_Gripper_Release();

    /* 归位：先横向、后纵向 */
    AGV_Position_Translate(PICK_RAW_SIDE_MM, Vel, (uint8_t)(PICK_RAW_SIDE_C_DIR ^ 1U));
    AGV_Position_GoStraight(PICK_RAW_STRAIGHT_MM, Vel, (uint8_t)(PICK_RAW_STRAIGHT_C_DIR ^ 1U));
}

/* ===================== 原料区六种固定抓取路线 =====================
 * 前置：相机 0x03 已经报出 A/B/C 三框的**初始**颜色。每抓走一块，转盘内容按
 * 索引 −1 挪一格（A→C、C→B、B→A，2026-10-23 实车确认），所以「任务码第 i 个
 * 颜色」实际要去 (初始格 + 3 - i) % 3 抓。三个颜色都齐时，结果只可能是六种：
 *
 *   初始排列(颜色在 A/B/C 的哪一格)   实际抓取顺序   函数
 *   A B C                             A A A         Pick_RAW_AAA
 *   B C A                             B B B         Pick_RAW_BBB
 *   C A B                             C C C         Pick_RAW_CCC
 *   A C B                             A B C         Pick_RAW_ABC
 *   B A C                             B C A         Pick_RAW_BCA
 *   C B A                             C A B         Pick_RAW_CAB
 *
 * 抓 A/C 位的每一块都要先把底盘挪到该框偏移位（纵向 + 横向，和 Pick_RAW_A/C
 * 的去程同一套），抓完再挪走；抓 B 位那块底盘不动。
 * 同一格连抓三次时，转盘会把下一块送到同一格，那三次之间底盘不动。
 * 换格时底盘只走「两框偏移之差」，不回中位 —— 相对旧版每块都回原位省掉大段行程。
 * A/C 两框的横向偏移相同、纵向相反(见文件顶部的 PICK_RAW_* 宏)，所以 A↔C
 * 之间**只有纵向移动**，不用横移，这正是 BCA / CAB 两条路线省下来的地方。
 */

typedef enum
{
    RAW_BOX_A = 0,
    RAW_BOX_B = 1,
    RAW_BOX_C = 2
} RawBox;

/* 各框相对基准位(中位)的底盘偏移(mm，带符号)：纵向 + = 向前，横向 + = 向左。
   符号直接从顶部宏的 dir 推出来，改宏不用改这里。 */
static int16_t Raw_StraightOffset(RawBox box)
{
    if(box == RAW_BOX_A)
        return PICK_RAW_STRAIGHT_A_DIR ? (int16_t)PICK_RAW_STRAIGHT_MM : -(int16_t)PICK_RAW_STRAIGHT_MM;
    if(box == RAW_BOX_C)
        return PICK_RAW_STRAIGHT_C_DIR ? (int16_t)PICK_RAW_STRAIGHT_MM : -(int16_t)PICK_RAW_STRAIGHT_MM;
    return 0;   /* B 位在正前方，底盘不用动 */
}

static int16_t Raw_SideOffset(RawBox box)
{
    if(box == RAW_BOX_A)
        return PICK_RAW_SIDE_A_DIR ? (int16_t)PICK_RAW_SIDE_MM : -(int16_t)PICK_RAW_SIDE_MM;
    if(box == RAW_BOX_C)
        return PICK_RAW_SIDE_C_DIR ? (int16_t)PICK_RAW_SIDE_MM : -(int16_t)PICK_RAW_SIDE_MM;
    return 0;
}

/* 底盘从 from 框的偏移位挪到 to 框的偏移位（相对量 = 两框偏移之差）。
   GoStraight: 1=向前 0=向后；Translate: 1=向左 0=向右。
   两条腿的**顺序**照抄 Pick_RAW_A/Pick_RAW_C：
     去 A/C 位（离开中位）：先纵向、后横向；
     回中位（to == B）      ：先横向、后纵向 —— 和原函数的归位完全一致，
                              斜着退回到料架边会蹭。
   某方向差值为 0 就一条命令都不发：A↔C 只有纵向，同框连抓整句空转。 */
static void Raw_Chassis_Goto(RawBox from, RawBox to)
{
    int16_t ds = (int16_t)(Raw_StraightOffset(to) - Raw_StraightOffset(from));
    int16_t dt = (int16_t)(Raw_SideOffset(to)     - Raw_SideOffset(from));

    if(to == RAW_BOX_B)
    {
        /* 归位：先横向、后纵向 */
        if(dt > 0)      AGV_Position_Translate((uint32_t)dt,    Vel, 1U);
        else if(dt < 0) AGV_Position_Translate((uint32_t)(-dt), Vel, 0U);
        if(ds > 0)      AGV_Position_GoStraight((uint32_t)ds,   Vel, 1U);
        else if(ds < 0) AGV_Position_GoStraight((uint32_t)(-ds), Vel, 0U);
    }
    else
    {
        /* 去 A/C 位：先纵向、后横向 */
        if(ds > 0)      AGV_Position_GoStraight((uint32_t)ds,   Vel, 1U);
        else if(ds < 0) AGV_Position_GoStraight((uint32_t)(-ds), Vel, 0U);
        if(dt > 0)      AGV_Position_Translate((uint32_t)dt,    Vel, 1U);
        else if(dt < 0) AGV_Position_Translate((uint32_t)(-dt), Vel, 0U);
    }
}

/* 臂动作：去 box 框抓一块，放进车上第 slot 槽。不碰底盘。
   夹爪在上一块 Release 之后停在 release 位(65°)——这个角度本来就是放开料块的角度，
   比料块宽，可以直接下探；实车要是蹭到料，在 Pick_JumpTo 之前补一句
   Pick_Gripper_Observe() 即可，代价是每块多一次舵机行程。 */
static void Raw_PickAt(RawBox box, uint8_t slot)
{
    switch(box)
    {
        case RAW_BOX_A: Pick_JumpTo POS_RAW_A; break;
        case RAW_BOX_C: Pick_JumpTo POS_RAW_C; break;
        default:        Pick_JumpTo POS_RAW_B; break;
    }
    Pick_TurnTable_Set(slot);       /* 臂还在原料区时转盘选槽，不会碰到臂 */
    Pick_Gripper_Grab();
    Pick_JumpTo POS_CAR;
    Pick_Gripper_Release();
}

/* ---- 六种固定路线 ----
   名称就是三次抓取实际去的框。**每抓 A/C 位的一块，前面都有一条
   Raw_Chassis_Goto 把底盘挪到该框偏移位**，抓 B 位那块底盘不动。
   底盘只在换框时动、不回中位；最后一次抓完才归位（两条腿的顺序见
   Raw_Chassis_Goto 的注释，与 Pick_RAW_A/C 的去程、归位一致）。
   槽位固定按 0→1→2 先后放，与任务码第 i 色对应。
   进函数时夹爪停在 Pick_RAW_PLUS 摆的 observe 位；每抓完一块停在 release
   位，可以直接下探下一块（见 Raw_PickAt 注释）。

   同框连抓(AAA/BBB/CCC)每抓完一块停 PICK_RAW_SAMEBOX_HOLD_MS(700ms)：
   让料块在车上槽里落稳、夹爪松到位，再去转盘换下一个槽 —— 连抓时转盘马上
   就要转，不等这一下容易把刚放下去的料拨歪。第三块抓完后面没有换槽动作，
   就不用停了（省 700ms）。 */
#define PICK_RAW_SAMEBOX_HOLD_MS   700U

void Pick_RAW_AAA(void)     /* 三次都抓 A 框 */
{
    Raw_Chassis_Goto(RAW_BOX_B, RAW_BOX_A);  /* 底盘去 A 位：先纵向、后横向 */
    Raw_PickAt(RAW_BOX_A, 0);
    HAL_Delay(PICK_RAW_SAMEBOX_HOLD_MS);     /* 料块落稳再抓下一块 */
    Raw_PickAt(RAW_BOX_A, 1);                /* 同框：转盘已把下一块送到 A，底盘不动 */
    HAL_Delay(PICK_RAW_SAMEBOX_HOLD_MS);
    Raw_PickAt(RAW_BOX_A, 2);
    Raw_Chassis_Goto(RAW_BOX_A, RAW_BOX_B);  /* 归位：先横向、后纵向 */
}

void Pick_RAW_BBB(void)     /* 三次都抓 B 框：B 就在车头正前方，底盘全程不动 */
{
    Raw_PickAt(RAW_BOX_B, 0);
    HAL_Delay(PICK_RAW_SAMEBOX_HOLD_MS);
    Raw_PickAt(RAW_BOX_B, 1);
    HAL_Delay(PICK_RAW_SAMEBOX_HOLD_MS);
    Raw_PickAt(RAW_BOX_B, 2);
}

void Pick_RAW_CCC(void)     /* 三次都抓 C 框 */
{
    Raw_Chassis_Goto(RAW_BOX_B, RAW_BOX_C);  /* 底盘去 C 位：先纵向、后横向 */
    Raw_PickAt(RAW_BOX_C, 0);
    HAL_Delay(PICK_RAW_SAMEBOX_HOLD_MS);
    Raw_PickAt(RAW_BOX_C, 1);                /* 同框：底盘不动 */
    HAL_Delay(PICK_RAW_SAMEBOX_HOLD_MS);
    Raw_PickAt(RAW_BOX_C, 2);
    Raw_Chassis_Goto(RAW_BOX_C, RAW_BOX_B);  /* 归位：先横向、后纵向 */
}

void Pick_RAW_ABC(void)     /* A → B → C */
{
    Raw_Chassis_Goto(RAW_BOX_B, RAW_BOX_A);  /* 去 A 位 */
    Raw_PickAt(RAW_BOX_A, 0);
    Raw_Chassis_Goto(RAW_BOX_A, RAW_BOX_B);  /* A 回中位（先横向、后纵向）*/
    Raw_PickAt(RAW_BOX_B, 1);
    Raw_Chassis_Goto(RAW_BOX_B, RAW_BOX_C);  /* 中位去 C 位（先纵向、后横向）*/
    Raw_PickAt(RAW_BOX_C, 2);
    Raw_Chassis_Goto(RAW_BOX_C, RAW_BOX_B);  /* 归位 */
}

void Pick_RAW_BCA(void)     /* B → C → A */
{
    Raw_PickAt(RAW_BOX_B, 0);                /* B 位，底盘不动 */
    Raw_Chassis_Goto(RAW_BOX_B, RAW_BOX_C);  /* 去 C 位 */
    Raw_PickAt(RAW_BOX_C, 1);
    Raw_Chassis_Goto(RAW_BOX_C, RAW_BOX_A);  /* C→A：两框横向偏移相同，只有纵向 */
    Raw_PickAt(RAW_BOX_A, 2);
    Raw_Chassis_Goto(RAW_BOX_A, RAW_BOX_B);  /* 归位 */
}

void Pick_RAW_CAB(void)     /* C → A → B */
{
    Raw_Chassis_Goto(RAW_BOX_B, RAW_BOX_C);  /* 去 C 位 */
    Raw_PickAt(RAW_BOX_C, 0);
    Raw_Chassis_Goto(RAW_BOX_C, RAW_BOX_A);  /* C→A：只有纵向 */
    Raw_PickAt(RAW_BOX_A, 1);
    Raw_Chassis_Goto(RAW_BOX_A, RAW_BOX_B);  /* A 回中位（先横向、后纵向）*/
    Raw_PickAt(RAW_BOX_B, 2);
}

/**
 * @brief 原料区取料（备用路线）：读一次三框颜色 → 算出六种固定路线之一 → 交给对应函数
 * @note  流程：
 *        1) 摆 POS_OBSERVE_RAW_CENTER（高位、收臂，不挡相机），张开夹爪；
 *        2) **只问一次** 0x03，拿到三个框的初始颜色，下标 0/1/2 = A/B/C 框；
 *        3) 对任务码第 i 个颜色，算出它**现在**该去哪个框抓：
 *             在该颜色初始所在格上，减去已经抓走的 i 块（转盘索引 −1），
 *             即 seq[i] = (j0 + 3 - i) % 3，再转成 1/2/3(A/B/C)；
 *        4) 三个数只可能是六种组合之一，直接调对应函数：
 *              1-1-1 → Pick_RAW_AAA   2-2-2 → Pick_RAW_BBB   3-3-3 → Pick_RAW_CCC
 *              1-2-3 → Pick_RAW_ABC   2-3-1 → Pick_RAW_BCA   3-1-2 → Pick_RAW_CAB
 *           六条路线内部只在换框时动底盘，同框连抓时底盘全程不动，也不回中位。
 *
 *        为什么能一次问完就算出来：原料转盘每抓走一块固定挪一格（方向已实测为
 *        索引 −1），三块的颜色排列因此只有 6 种，开局一次读数就足以定下整条路线；
 *        旧版每抓一块都回观察位重问一次，准但慢，这次换成"一次读、按排列查表"。
 *
 *        三块各占一格，颜色一定齐，所以不做缺色处理 —— 直接按排列查表。
 *        只有 0x03 本身超时（相机没响应）才原样返回：那时 box_color[] 是上一轮的
 *        旧值，照着抓会连错三块。
 *        0x02 的视觉对正已搬到 Pick_Correct()，这里不做。
 *
 *        调用关系：目前没有地方调它（task.c 的 `case Pick1` 走的仍是 Pick_RAW）。
 *        想切换时把那里的 Pick_RAW() 换成 Pick_RAW_PLUS() 即可。
 */
void Pick_RAW_PLUS(void)
{
    float    temp_yaw = ZeroBias_GetYaw();  /* 记录修正前的航向，修正后再回到此航向 */
    uint8_t *Pick_Color = Pick_Color_List();
    uint8_t  box[3];                    /* 开局读一次的三框颜色: 0=A 1=B 2=C */
    uint8_t  seq[3];                    /* 第 i 块实际去哪个框：1=A 2=B 3=C */

    Pick_JumpTo POS_OBSERVE_RAW_CENTER;   /* 要看清三个框，得摆到观察位 */
    Pick_Gripper_Observe();             /* 张开，别挡住相机 */
    HAL_Delay(2000);                    /* 等相机输出稳定(上电/切模式后头几帧不可信) */

    Cam_SwitchTo(03);           /* 相机没响应就不猜，直接收工 */

    for(uint8_t k = 0; k < 3U; k++)
    {
        box[k] = box_color[k];          /* 立刻拷走：box_color[] 会被后续帧刷新 */
    }

    for(uint8_t i = 0; i < 3U; i++)
    {
        uint8_t k = 0U;
        while(k < 3U && box[k] != Pick_Color[i]) k++;   /* 这一色初始在第 k 格 */

        /* 转盘每抓走一块按索引 −1 挪一格 → 第 i 块时它在 (k + 3 - i) % 3 格；
           +1 转成 1=A 2=B 3=C */
        seq[i] = (uint8_t)((k + 3U - i) % 3U) + 1U;
    }

    /* 六种排列查表，一一对应上面的路线表 */
    if     (seq[0] == 1U && seq[1] == 1U && seq[2] == 1U) Pick_RAW_AAA();
    else if(seq[0] == 2U && seq[1] == 2U && seq[2] == 2U) Pick_RAW_BBB();
    else if(seq[0] == 3U && seq[1] == 3U && seq[2] == 3U) Pick_RAW_CCC();
    else if(seq[0] == 1U && seq[1] == 2U && seq[2] == 3U) Pick_RAW_ABC();
    else if(seq[0] == 2U && seq[1] == 3U && seq[2] == 1U) Pick_RAW_BCA();
    else if(seq[0] == 3U && seq[1] == 1U && seq[2] == 2U) Pick_RAW_CAB();

    Pick_JumpTo POS_RUN;
    AGV_SetYaw(temp_yaw);  /* 修正后回到修正前的航向 */
}

/**
 * @brief 粗加工区放置前的视觉对正【低位】：摆圆环观察位 → 开爪 → 0x04 拿偏差 → AGV_Correct 闭环 → 收爪
 * @note  这是三步对正里的**最后一步**，也负责收爪：
 *          Place_Correct_High()  → 高位粗修正（不收爪）
 *          Place_Correct_Angle() → 移位量角度、航向复位（不碰夹爪）
 *          Place_Correct_Low()   → 本位细修正 + 收爪，然后才 Place_ROUGH()
 *        也只单独调它（第二轮暂存区 case Place2 就走这一条）。
 *        所以 Place_ROUGH 自己**不再**重复做视觉这一步（和 Pick_Correct / Pick_RAW 一样的分工）。
 *
 *        请求必须发在"摆到观察位"之后 —— 相机得先看见圆环才有东西可识别。
 *        环号在这段用不上（环号由二维码的 Place_Color[] 给出），只要 dx/dy 落全局给
 *        AGV_Correct 用。
 *
 *        拿不到偏差就不修正 —— 超时的话 dx/dy 还是上一次留下的旧值，拿它驱轮子
 *        会把车带偏；车本来就停在粗加工区，原地不动、直接进放置反而更安全。
 */
void Place_Correct_Low(void)
{
    Pick_JumpTo POS_OBSERVE_RING_LOW;     /* 圆环区观察位：相机 0x04 看的就是这儿 */
    Pick_Gripper_Observe();             /* 修正期间张开，别挡住相机 */
    if(Cam_SwitchTo(04))                /* 偏差落到全局 dx/dy */
    {
        AGV_Correct(30000);
    }
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
#define PLACE_CORRECT_HIGH_MS               1000U   /* 高位圆环2 粗修正时间 */
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
        AGV_Correct(PLACE_CORRECT_HIGH_MS);
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

        AGV_SetYaw(ZeroBias_GetYaw() + PLACE_CORRECT_ANGLE_GAIN * err);
    }

    /* ---- 3) 把对正后的实际航向记成基准 a1 ---- */
    a1 = ZeroBias_GetYaw();

    /* ---- 4) 退回原放料位，并把航向复位到 a1 ---- */
    AGV_Position_GoStraight(PLACE_CORRECT_ANGLE_SHIFT_MM, Vel,
                            (uint8_t)(PLACE_CORRECT_ANGLE_SHIFT_DIR ^ 1U));
    AGV_SetYaw(a1);
}

/**
 * @brief 粗加工区放置：车上三个槽依次放到二维码指定的圆环上
 * @note  视觉对正已搬到 Place_Correct_High/Angle/Low，这里只管放。
 *
 *        三次放置：A→环, B→环, C→环（取料顺序 A 先出来）
 *        槽号 i 始终是「槽位下标」(0=A,1=B,2=C)，和 Pick_ROUGH / Place_TEMP /
 *        Pick_Color[] 同一套约定，这样放置完再拾取物块才能回到原槽位。
 */
void Place_ROUGH(void)
{
    uint8_t *Place_Color = (RerunFlag == 0) ? Place_Color1 : Place_Color2;

    for (int i = 0; i < 3; i++) {
        PlaceToRing((uint8_t)i, Place_Color[i]);
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
        AGV_Correct(5000);
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
