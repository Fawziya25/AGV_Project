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
 *   Pick_RAW_PLUS()  改动前的老路线：读一次 0x03 三框颜色 + (j0+grabbed)%3 推算（未调用）
 * 详见各函数自己的注释。
 *
 * ── 2026-10-16 放置侧同样拆开 ──────────────────────────────────────
 * 照原料区那套分工，把视觉对正从两个 Place_xxx 里摘出来，只留下放置动作：
 *   Place_Correct()        摆圆环观察位 → 开爪 → 0x04 → AGV_Correct(30000) → 收爪
 *   Place_ROUGH()          只管放：车上三槽依次放到 Place_Color[] 指定的圆环
 *   Place_TEMP_Correct()   摆暂存区观察位 → 开爪 → 0x05 → AGV_Correct(5000) → 收爪
 *   Place_TEMP()           只管放：第二轮同色同环码到高位
 * 上层 task.c：
 *   case Place1  连着调 Place_Correct() + Place_ROUGH()
 *   case Place2  按 RerunFlag 选对正函数（第一圈观察位就在圆环区，复用 Place_Correct），
 *                再调 Place_TEMP()
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
    Pick_JumpTo POS_RAW_B;
    Pick_Gripper_Observe();         /* 修正期间张开，别挡住相机 */
    if(Cam_SwitchTo(02))  /* 偏差落到全局 dx/dy */
    {
        AGV_Correct(3000);
    }
    Pick_Gripper_Release();         /* 修正完了，下去抓之前收成 release 位 */
}

/**
 * @brief 原料区取料（当前路线）：在取料位等，等到取料位上那一块就是要抓的颜色才下手
 * @note  流程（严格按任务码顺序，i = 0 → 1 → 2）：
 *        1) 摆回 POS_RAW_B、开爪（别挡住相机）；
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
        Pick_JumpTo POS_RAW;
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

/**
 * @brief 原料区取料（备用路线，当前未调用）：读一次三框颜色，靠推算转盘走格取料
 * @note  这是 2026-10-16 改动**之前**的 Pick_RAW 后半段，原样留作备份：
 *        -- 开局向相机要一次三个框内的颜色(0x03)，拿到 box_color[0..2]，
 *           下标 0/1/2 = A/B/C 框 = 相机回报的 左上/下/右上；
 *        -- 按任务码顺序 Pick_Color[0]→[1]→[2]，每个颜色去"装着它的那个框"抓，
 *           第 i 个颜色抓到的料放到车上第 i 个槽；框里没有这个颜色就跳过这一块，
 *           对应槽位留空。
 *
 *        **原料转盘会跟着走格**：抓走一块后，剩下的料整体 A→B→C→A 挪一格
 *        （机械自动触发，不拿走就不转）。所以 box[] 只在开头读这一次，读完就作废 ——
 *        第 i 个颜色要去「它初始所在格 + 已经抓走的块数」上找，直接拿 box[] 原来的
 *        下标用是错的，第二次起每次都抓错格。跳过的块不计入 grabbed（没拿走就不会转）。
 *
 *        与 Pick_RAW 的分工：0x02 的对正已经搬到 Pick_Correct()，这里不再做；
 *        夹爪摆到观察位后停在 Pick_Correct 留下的 release 位就直接读 0x03，
 *        和改动前的 Pick_RAW 一致。
 *
 *        调用关系：目前没有地方调它（`task.c` 的 `case Pick1` 走的仍是 Pick_RAW）。
 *        想切换时把 `case Pick1` 里的 Pick_RAW() 换成 Pick_RAW_PLUS() 即可。
 */
void Pick_RAW_PLUS(void)
{
    uint8_t *Pick_Color = Pick_Color_List();
    uint8_t  grabbed = 0;               /* 已抓走的块数 = 转盘已经转过的格数 */

    Pick_JumpTo POS_OBSERVE_RAW;        /* 要看清三个框，得摆到观察位 */

    /* ---- 问三个框内颜色（结果在全局 box_color[0..2]），再按任务码顺序抓 ----
       超时就直接不抓 —— box_color[] 此时可能是上一轮的旧值，拿来抓是错的。
       这一读只在开局做一次，读完就作废：下面全靠推算走格，不再问相机。 */
    if(Cam_SwitchTo(03))
    {
        for(uint8_t i = 0; i < 3; i++)
        {
            int8_t j0 = -1;                 /* 任务码第 i 个颜色在**初始**读数里的格号 */
            for(uint8_t k = 0; k < 3; k++)
            {
                if(box_color[k] == Pick_Color[i]) { j0 = (int8_t)k; break; }
            }
            if(j0 < 0) continue;            /* 相机没看到这一色 → 这块跳过，槽位留空 */

            /* 每抓走一块转盘走一格(内容索引 +1 mod 3)，所以初始在第 j0 格的料
               现在在 (j0 + grabbed) % 3 格。跳过的块不计 —— 没拿走就不会转。 */
            uint8_t j = (uint8_t)((j0 + grabbed) % 3);

            GrabRawBoxToCar(j, i, 0);       /* 第 i 个颜色 → 第 i 个槽（走门字形） */
            grabbed++;
        }
    }

    Pick_JumpTo POS_RUN;
}

/**
 * @brief 粗加工区放置前的视觉对正：摆圆环观察位 → 开爪 → 0x04 拿偏差 → AGV_Correct 闭环 → 收爪
 * @note  由 task.c 的 `case Place1` 在 Place_ROUGH 之前调用一次，
 *        所以 Place_ROUGH 自己**不再**重复做这一步（和 Pick_Correct / Pick_RAW 一样的分工）。
 *
 *        请求必须发在"摆到观察位"之后 —— 相机得先看见圆环才有东西可识别。
 *        环号在这段用不上（环号由二维码的 Place_Color[] 给出），只要 dx/dy 落全局给
 *        AGV_Correct 用。
 *
 *        拿不到偏差就不修正 —— 超时的话 dx/dy 还是上一次留下的旧值，拿它驱轮子
 *        会把车带偏；车本来就停在粗加工区，原地不动、直接进放置反而更安全。
 */
void Place_Correct(void)
{
    Pick_JumpTo POS_OBSERVE_RING;     /* 圆环区观察位：相机 0x04 看的就是这儿 */
    Pick_Gripper_Observe();             /* 修正期间张开，别挡住相机 */
    if(Cam_SwitchTo(04))                /* 偏差落到全局 dx/dy */
    {
        AGV_Correct(30000);
    }
    Pick_Gripper_Release();             /* 修正完了，下去放之前收成 release 位 */
}

/**
 * @brief 粗加工区放置：车上三个槽依次放到二维码指定的圆环上
 * @note  视觉对正已搬到 Place_Correct()，这里只管放。
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
 *        直接复用 Place_Correct()。两边由上层在 task.c 的 `case Place2` 里按 RerunFlag 二选一：
 *          RerunFlag == 0 → Place_Correct()
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
    Pick_JumpTo POS_OBSERVE_TEMP;       /* 暂存区观察位 */
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
 * @note  视觉对正已搬走（第一轮 Place_Correct / 第二轮 Place_TEMP_Correct），这里只管放。
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
