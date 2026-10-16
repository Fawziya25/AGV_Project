/**
 * @file pick_task.c
 * @brief 物料取放任务阶段实现（2026-10-03 视觉协议换代后重写）
 *
 * 视觉接口一律走 camera.h 的新二进制定长帧协议：调用方发一条请求命令，
 * 等相机回 flag!=0 的帧即算"干完了"；偏差/颜色从全局量 dx/dy/box_color[] 直接读。
 * 新协议里没有"电控下发颜色号"这回事 —— 颜色是相机回报的东西，不是命令。
 *
 * 与旧版的对应关系：
 *   旧 Cam_SwitchTo(color)            → 新 Cam_RequestDisc() / Cam_RequestRing()
 *   旧 Cam_WaitFor* （电控自己判静止）  → 删除；静止判据由相机侧负责，靠 flag 回报
 *   旧 Cam_SwitchCloseColor()         → 删除；改由 Cam_RequestBoxColor() 直接问颜色
 *   旧 ScanNearestColor / TrackOneColor 等本地追踪 → 整体删除
 */
#include "pick_task.h"
#include "positionPIDcontroller.h"  // AGV_Correct()

/* 视觉请求的两段超时都是 10s：第二段用 camera.h 的 CAM_REQ_TMO_MS，
   第一段那个 10s 在 camera.c 的 CAM_ACK_TMO_MS。 */

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
 *               Pick_RAW 按「初始格 + 已抓块数」推算后传入；Pick_RAW_PLUS 传相机当轮读数。
 * @param slot   槽下标 0=A, 1=B, 2=C（转盘 30°/150°/270°）
 * @param direct 下到料位这一步怎么走：
 *               0 = `Pick_JumpTo`，先升到安全高度再转+伸半径，慢但能跨障碍（Pick_RAW 用）
 *               1 = `Pick_MoveTo`，贴着观察位高度直接转+伸，最后才降下去，快（Pick_RAW_PLUS 用）
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
 * @brief 原料区取料：对正圆盘中心 → 问三框颜色 → 按任务码顺序抓到车上
 * @note  流程：
 *        1) 摆到观察位、开爪（别挡住相机）；
 *        2) 请求圆盘中心偏差(0x02)，再 AGV_Correct 闭环把车对正；
 *        3) 请求三个框内颜色(0x03)，拿到 box_color[0..2]，下标 0/1/2 = A/B/C 框；
 *        4) 按任务码顺序 Pick_Color[0]→[1]→[2]，每个颜色去"装着它的那个框"抓，
 *           第 i 个颜色抓到的料放到车上第 i 个槽(A/B/C)。框里没有这个颜色
 *           (相机没识别到/颜色不符)就跳过这一块，对应槽位留空。
 *
 *        **原料转盘会跟着走格**：抓走一块后，剩下的料整体 A→B→C→A 挪一格
 *        (机械自动触发，不拿走就不转)。所以 box[] 只在开头读这一次，读完就作废 ——
 *        第 i 个颜色要去「它初始所在格 + 已经抓走的块数」上找，直接拿 box[] 原来的
 *        下标用是错的，第二次起每次都抓错格。
 *
 *        请求函数传 NULL 是因为 AGV_Correct 永远读全局 dx/dy，不必在这里接值。
 *        RequestBoxColor 超时就直接不抓 —— box[] 此时可能是上一轮的旧值，拿来抓是错的。
 */
void Pick_RAW(void)
{
    uint8_t *Pick_Color = Pick_Color_List();
    uint8_t  box[3] = {0, 0, 0};        /* 相机回报的三个框内颜色(只在开局读一次) */
    uint8_t  grabbed = 0;               /* 已抓走的块数 = 转盘已经转过的格数 */
    Pick_JumpTo POS_OBSERVE_RAW;
    Pick_Gripper_Observe();         /* 修正期间张开，别挡住相机 */
    /* ---- 1) 对正圆盘中心：拿不到偏差就不修正 ----
       超时的话 dx/dy 还是上一段留下的旧值，拿它驱轮子会把车带偏；
       车本来就已经停在原料区了，原地不动、直接进抓取反而更安全。 */
    if(Cam_RequestDisc(NULL, NULL, CAM_REQ_TMO_MS))     /* 偏差落到全局 dx/dy */
    {
        AGV_Correct(5000);
    }
    Pick_Gripper_Release();         /* 修正完了，下去抓之前收成 release 位 */
    /* ---- 2) 问三个框内颜色，再按任务码顺序抓 ---- */
    if(Cam_RequestBoxColor(&box[0], &box[1], &box[2], CAM_REQ_TMO_MS))
    {
        for(uint8_t i = 0; i < 3; i++)
        {
            int8_t j0 = -1;                 /* 任务码第 i 个颜色在**初始**读数里的格号 */
            for(uint8_t k = 0; k < 3; k++)
            {
                if(box[k] == Pick_Color[i]) { j0 = (int8_t)k; break; }
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
 * @brief 同 Pick_RAW，但**每抓一块之前都回观察位重新问一次位置**（当前未调用）
 *
 * @note  与 Pick_RAW 的两点区别：
 *
 *        1) "料转到哪一格"怎么拿到
 *             Pick_RAW      开局读一次 box[]，之后靠 (j0 + grabbed) % 3 自己推算走格。
 *             Pick_RAW_PLUS 不推算 —— 每抓一块前回 POS_OBSERVE_RAW 重发一次 0x03，
 *                           按相机**当轮**报的颜色直接下手。
 *           所以 PLUS 对转盘方向/格数不敏感（转反了、转两格、甚至人工挪过料都无所谓），
 *           代价是每块多一次"回观察位 + 等相机识别"的时间。
 *
 *        2) 从观察位下到料位那一步用 `Pick_MoveTo`（不抬到安全高度，贴着观察位高度
 *           直接转+伸半径，最后才降），比 `Pick_RAW` 的 `Pick_JumpTo` 少一次升降。
 *           —— 这一步是 `GrabRawBoxToCar()` 的 `direct` 参数；只有"下到料位"分两版，
 *           "从料位回 POS_CAR"那一趟仍然一律 `Pick_JumpTo`，因为那是往车体上方去。
 *
 *        "等 flag 再次等于 1"这一步不用在这里额外写：Cam_RequestBoxColor() 内部
 *        第二段等的就是 flag!=0，而且它在发命令**之前**会把完成锁存清掉，因此它等的
 *        必然是这条新命令之后**新来**的一个 flag=1，不是上一条命令残留的那个。
 *
 *        某一块颜色相机没报到就跳过该块、槽位留空，但**下一轮照样重新问一遍**——
 *        这正是 PLUS 版比 Pick_RAW 强的地方：跳过一块不会让后面所有格号全错位。
 *
 *        调用关系：目前没有地方调它（`task.c` 的 `case Pick1` 走的仍是 Pick_RAW）。
 *        想切换时把 `case Pick1` 里的 Pick_RAW() 换成 Pick_RAW_PLUS() 即可。
 */
void Pick_RAW_PLUS(void)
{
    uint8_t *Pick_Color = Pick_Color_List();
    uint8_t  box[3] = {0, 0, 0};        /* 每次重问都整体刷新，不用保留上一轮的值 */

    /* ---- 开局：摆到观察位、开爪、对正圆盘中心（与 Pick_RAW 相同）---- */
    Pick_JumpTo POS_OBSERVE_RAW;
    Pick_Gripper_Observe();         /* 修正期间张开，别挡住相机 */
    if(Cam_RequestDisc(NULL, NULL, CAM_REQ_TMO_MS))     /* 偏差落到全局 dx/dy */
    {
        AGV_Correct(5000);          /* 拿不到偏差就不修正，免得用旧 dx/dy 把车带偏 */
    }
    Pick_Gripper_Release();         /* 修正完了，下去抓之前收成 release 位 */

    for(uint8_t i = 0; i < 3; i++)
    {
        int8_t j = -1;                  /* 任务码第 i 个颜色此刻在第几格 */

        /* 回观察位重问一次。第一轮时臂已经在这个位姿上，Pick_JumpTo 会短路掉，
           不会白走一趟；之后每轮臂才真的从车上摆回来。 */
        Pick_JumpTo POS_OBSERVE_RAW;
        Pick_Gripper_Observe();         /* 问颜色期间张开，别挡住相机 */
        if(!Cam_RequestBoxColor(&box[0], &box[1], &box[2], CAM_REQ_TMO_MS))
        {
            break;                      /* 这一色都问不到，后面的更问不到，直接收工 */
        }

        for(uint8_t k = 0; k < 3; k++)
        {
            if(box[k] == Pick_Color[i]) { j = (int8_t)k; break; }
        }
        if(j < 0) continue;             /* 相机没看到这一色 → 这块跳过，槽位留空 */

        Pick_Gripper_Release();         /* 问完了要下去抓，先收成 release 位 */
        GrabRawBoxToCar((uint8_t)j, i, 1);  /* 第 i 个颜色 → 第 i 个槽（直接下料位） */
    }

    Pick_JumpTo POS_RUN;
}

void Place_ROUGH(void)
{
    uint8_t *Place_Color = (RerunFlag == 0) ? Place_Color1 : Place_Color2;

    /* ---- 视觉修正：请求圆环(0x04) ----
       请求必须发在"摆到观察位"之后 —— 相机得先看见圆环才有东西可识别。
       环号在这段用不上（环号由二维码的 Place_Color[] 给出），只要 dx/dy 落全局给
       AGV_Correct 用，所以三个出参都传 NULL。 */
    Pick_JumpTo POS_OBSERVE_CIRCLE;
    Pick_Gripper_Observe();
    if(Cam_RequestRing(NULL, NULL, NULL, CAM_REQ_TMO_MS))
    {
        AGV_Correct(30000);      /* 拿不到偏差就不修正，免得用旧 dx/dy 把车带偏 */
    }
    Pick_Gripper_Release();

    // 三次放置：A→环, B→环, C→环（取料顺序 A 先出来）
    // 槽号 i 始终是「槽位下标」(0=A,1=B,2=C)，和 Pick_ROUGH / Place_TEMP /
    // Pick_Color[] 同一套约定，这样放置完再拾取物块才能回到原槽位。
    for (int i = 0; i < 3; i++) {
        PlaceToRing((uint8_t)i, Place_Color[i]);
    }
    /* 由上层状态机控制返回 POS_RUN */
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

void Place_TEMP(void)
{
    /* 第二轮暂存区：环号不按第二轮任务码，而是按第一轮「颜色→环位」的对应关系同色同环码垛 */
    uint8_t ring_reuse[3];
    uint8_t vision_ok = 0;      /* 视觉请求是否成功；失败就不做 AGV 修正 */
    for (int i = 0; i < 3; i++) {
        ring_reuse[i] = Place_RingSameAsLastRound(i);
    }

    uint8_t *Place_Color = (RerunFlag == 0) ? Place_Color1 : ring_reuse;
    uint8_t is_high = (RerunFlag == 1);

    /* ---- 视觉修正 ----
       第二轮：对正"第一轮放在 2 号环上的那一块"的颜色 —— 用 0x05 把这个颜色发下去，
       相机识别到该颜色后回报 flag=1 和 dx/dy。
       第一轮：观察位是圆环区，与 Place_ROUGH 同一处，同样用 0x04 请求圆环。 */
    if (RerunFlag == 0)
    {
        Pick_JumpTo POS_OBSERVE_CIRCLE;
        Pick_Gripper_Observe();
        Cam_Service();
        vision_ok = Cam_RequestRing(NULL, NULL, NULL, CAM_REQ_TMO_MS);
    }
    else
    {
        /* 第一轮哪一槽放到了 2 号环，就取那一槽的颜色 —— 与 Place_RingSameAsLastRound
           同一套对应关系，只是反过来查。 */
        uint8_t color_to_send = 0;
        if      (Place_Color1[0] == 2) color_to_send = Pick_Color1[0];
        else if (Place_Color1[1] == 2) color_to_send = Pick_Color1[1];
        else if (Place_Color1[2] == 2) color_to_send = Pick_Color1[2];

        /* 颜色号是 1~6，所以 0 只可能是"第一轮没有哪一槽放进 2 号环"。
           这时没有颜色可问，发 AA 05 00 BB 相机也不认，直接当失败处理。 */
        if(color_to_send != 0U)
        {
            Pick_JumpTo POS_OBSERVE_TEMP;
            Pick_Gripper_Observe();
            Cam_Service();
            vision_ok = Cam_RequestColor(color_to_send, NULL, NULL, CAM_REQ_TMO_MS);
        }
    }

    if(vision_ok)
    {
        AGV_Correct(5000);      /* 拿不到偏差就不修正，免得用旧 dx/dy 把车带偏 */
    }
    Pick_Gripper_Release();

    // 三次放置：A→环, B→环, C→环（取料顺序 A 先出来）
    GRAB_AND_PLACE_TO_RING_SLOT_EX(0, Place_Color[0], is_high);   /* A 槽 */
    GRAB_AND_PLACE_TO_RING_SLOT_EX(1, Place_Color[1], is_high);   /* B 槽 */
    GRAB_AND_PLACE_TO_RING_SLOT_EX(2, Place_Color[2], is_high);   /* C 槽 */

    Pick_JumpTo POS_RUN;
}
