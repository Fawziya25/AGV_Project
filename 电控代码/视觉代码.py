# -*- coding: utf-8 -*-
"""
================================================================================
 MaixCAM / MaixCAM-Pro 视觉框架 v6   (配套协议：《视觉-电控通信》)
 ================================================================================
 已确认的模型类别表（model_322427.mud）：
     0: blue        1: light blue   2: yellow      3: three
     4: two         5: disk         6: black       7: green
     8: red         9: one

 发给电控的编码（不再直接发 yolo 的 class_id）：
     red=1  yellow=2  blue=3  green=4  black=5  light blue=6   没识别到 = 0x00
     圆环编号(0x04 的 flag)：one=1  two=2  three=3
     0x05 电控发编码 1~6 过来，代码反查成 class_id 再跟踪。

 0x02 = 最近物块（不再用圆盘 disk）：
     找离画面中心最近的【颜色物块】，回 [AA 02 flag dxH dxL dyH dyL BB]
     flag = 该物块颜色编码(1~6)，但要的是「动了 → 又停稳」这个事件：
            · 收到 0x02 时物块是静止的 → flag=0（电控不动手，不微调）
            · 物块被拿起/推走（位移>20px 或速度>6px/帧）→ 之后重新停稳 → flag=颜色码
            · 之后锁存，直到下一条指令 / 换了颜色
            · 状态机：WAIT_MOVE → WAIT_STILL → DONE；控制台每秒打印 [STATIC] 诊断
     dx/dy = 物块中心相对画面中心的偏差（一直在发）

 0x04 = 圆环：★只返回 two 的坐标（RING_TARGET_NUMS=(2,)）。
     flag=2 表示认到 two，0 表示画面里没有 two（dx=dy=0）。
     若 two 在 HOUGH_FOR_RING_NUMS 里（默认就是），额外跑【霍夫圆】用圆心修正 dx/dy
     （ENABLE_HOUGH 是总开关）

 0x06 = 请求修正角度（用 one/two 两个圆环替代会漂的陀螺仪）：
     两个中心 → 镜头畸变校正 → 平滑 → 换算成"相对光轴的水平方位角"
     → 取平均当作"这对圆环在画面里的方位" → 减去参考角 = 车头偏差角
     （LINE_METHOD="bearing"；旧的"连线角度"法仍保留为 "segment" 可切换）
     回 [AA 06 flag AngleH AngleL BB]   flag=1 已识别到；Angle 单位 0.1°，int16 大端
 0x07 = （可选、协议外）现场标定：把此刻的方位角记为"车头正对"的参考

 调试画面：三个固定框只在 0x03 模式显示；霍夫圆用橙色圆标出；0x06 拟合线用青线画出

 其它功能：
   - 任意模式固定 50Hz 循环回帧，与有没有检出目标无关；目标丢失 → flag=0 + dx=dy=0
   - 检出框前后帧关联 + 低通滤波（防抖动/防闪烁），滤波结果同时用于画图和算 dx/dy
   - 画框：什么颜色画什么颜色的框 + 中心十字准心；圆盘不画（铺满画面），数字标记白框
   - 0x03：三个框里 ≥ BOX_READY_MIN(默认2) 个就绪 → flag=1
     ★ 每 BOX_EVAL_PERIOD_MS(300ms) 重新数一次 "认出几个框"：
        3 个 → 原样发；2 个 → 用 0x01 任务码排除法补出第 3 个颜色；
        0/1 个 → 清掉三个框的颜色锁存，继续下一轮识别
       所以 flag=1 时 c1 c2 c3 一定是完整的三个颜色，顺序固定：左上 / 下 / 右上
   - 收帧按固定长度解析（payload 里的 0xBB 不会被当成帧尾）
   - ★ 补光灯：宏 LIGHT_ON_IN_MODE 控制，工作模式(0x01~0x07)点亮、空闲(0x00)熄灭；
        点亮/熄灭电平可用 FILL_LIGHT_ON_LEVEL / FILL_LIGHT_OFF_LEVEL 改
   - ★ dx/dy 归一：宏 DXDY_REF_ENABLE，把所有发给电控的 dx/dy 换算到 320x320 参考坐标系
        （只影响发的数据，画框/识别/阈值仍用真实像素），换分辨率不用改电控的 PID
   - ★ 夹爪偏置：宏 DXDY_OFFSET_ENABLE + DXDY_OFFSET_X/Y（正数=落点偏右/偏下），
        把 dx/dy 的零点挪到夹爪真实落点，修"视觉对正了却一直往右下放"
        （只影响发的数据，画框/识别/阈值仍用真实像素），换分辨率不用改电控的 PID
================================================================================
"""
from maix import camera, display, image, nn, app, comm, time, uart, pinmap, gpio, err
import os
import math
import struct

# ★ 版本号：跑起来第一行会打印，用来确认板子上跑的是不是最新这份代码
CODE_VERSION = "v7.2-aim-offset"
print("[VER] 代码版本:", CODE_VERSION)

# ==============================================================================
# 一、配置区（标定只改这一段）
# ==============================================================================
# ---------- 模型 ----------
MODEL_PATH     = "model_327205.mud"
MODEL_FALLBACK = "/root/models/training2/model_327205.mud"
# 换模型时不用删旧的：下面按顺序尝试，哪个能加载就用哪个（控制台会打印最终用的是哪个）
MODEL_CANDIDATES = [
    MODEL_PATH,
    MODEL_FALLBACK,
    "/root/model_327205.mud",
    "/root/models/model_327205.mud",
    "/root/models/training2/model_327205.mud",
    # ↓ 之前能跑的几份，留作保底；文件不存在会自动跳过
    "/root/models/model_326232.mud",
    "/root/models/training1/model_324947.mud",
    "/root/models/disk/model_322427.mud",
]
CONF_TH        = 0.5
IOU_TH         = 0.45

# ---------- 串口接线方案（先选这个！） ----------
# MaixCAM / MaixCAM-Pro 实际存在的串口（Sipeed 官方文档 + MaixCDK pinmap 源码）：
#   "A16A17"     UART0  A16=TX  A17=RX   /dev/ttyS0
#                与 Type-C 转接小板引出的是同一组线；开机有系统日志
#                ★★ 不要用来接电控：A16 是芯片的启动模式检测脚，开机时被拉低直接不启动
#   "A19A18"     UART1  A19=TX  A18=RX   /dev/ttyS1   ★推荐：不在启动脚上、无日志
#   "A28A29"     UART1  A28=TX  A29=RX   /dev/ttyS1   1.25mm 6pin 排线上的串口（接线最方便）
#   "A28A29_U2"  UART2  A28=TX  A29=RX   /dev/ttyS2   同一组引脚映射到 UART2
# ★ A0 / A1 在 MaixCAM(Pro) 上不存在，写进去只会报 "pin not valid" 而且静默失败
#
# ⚠ 关于"上电前引脚有电导致开不了机"（官方 FAQ 明确写了）：
#   1) UART0_TX(A16) 被拉低 → 系统起不来；
#   2) 芯片【所有引脚】在开机前都不能有电流灌入 —— 外接 MCU 的 TX/RX 如果是高电平，
#      会把电流灌进 MaixCAM 导致无法启动（把串口换到 A18/A19 也躲不掉这一条！）。
#   解决办法：① MaixCAM 先上电，电控后上电（或整机同时上电）；
#             ② 电控 TX/RX 串一个二极管 + 1k 电阻，并保证电控 TX 上电时是浮空输入；
#             ③ 或用电平转换/隔离芯片。
UART_OPTION = "A19A18"
UART_PLAN = {
    "A16A17":    {"dev": "/dev/ttyS0", "pins": (("A16", "UART0_TX"), ("A17", "UART0_RX"))},
    "A19A18":    {"dev": "/dev/ttyS1", "pins": (("A19", "UART1_TX"), ("A18", "UART1_RX"))},
    "A28A29":    {"dev": "/dev/ttyS1", "pins": (("A28", "UART1_TX"), ("A29", "UART1_RX"))},
    "A28A29_U2": {"dev": "/dev/ttyS2", "pins": (("A28", "UART2_TX"), ("A29", "UART2_RX"))},
}
UART_BAUD = 9600            # ← 你现在用的 9600（官方对 MaixCAM/Pro 只验证过 115200，
                            #    要是发现丢字节/乱码就把它和电控一起改成 115200）

# ---------- dx/dy 字节序 ----------
# True  = 先发高字节 = 设计书写的 dxH dxL
INT16_BIG_ENDIAN = True

# ---------- dx/dy 坐标系归一：一律按 320x320 输出 ----------
# 换模型后输入分辨率可能不是 320（比如 352 / 640）。如果直接把原始像素偏差发出去，
# 电控那边同一套 PID 参数就会"一会儿灵一会儿不灵"。打开这个开关后，
# 只把【发给电控的 dx/dy】换算到 320x320 参考坐标系；
# 画框、识别、所有像素阈值仍然用真实像素，互不影响。
#   dx_out = dx_px × (320 / 输入宽)      dy_out = dy_px × (320 / 输入高)
DXDY_REF_ENABLE = True      # ★宏开关：True = dx/dy 换算到 320x320 坐标系
DXDY_REF_W      = 320       # 参考坐标系宽（只影响 dx/dy）
DXDY_REF_H      = 320       # 参考坐标系高

# ---------- 夹爪/落点偏置补偿（修"视觉对正了但一直往右下放"这类固定偏差） ----------
# ★ 这是【加法】补偿，和上面的缩放是两回事：
#     缩放只能改大小（乘法），改不了零点；
#     偏置才能把零点挪走（加法）。
#   计算公式（AIM = 瞄准点 = dx/dy 的零点）：
#     AIM_X = 画面中心X + DXDY_OFFSET_X
#     dx_out = (物块中心x − AIM_X) × 缩放
#   所以：DXDY_OFFSET_X / Y 填【落点相对画面中心的偏移量】，
#         **正数 = 偏右 / 偏下（就是你看到的"一直放右下"），代码内部做减法，不要填负数**
#   例：落点稳定偏右 10px、偏下 15px → DXDY_OFFSET_X = 10，DXDY_OFFSET_Y = 15
DXDY_OFFSET_ENABLE = True   # ★宏开关：True = 启用瞄准点偏置
DXDY_OFFSET_X      = -1.2      # + = 落点偏右（单位：图像像素；输入正好 320 时等于电控看到的数）
DXDY_OFFSET_Y      = -45      # + = 落点偏下

# ---------- 回帧节流（任何模式都按这个频率循环回发） ----------
REPLY_HZ        = 50
REPLY_PERIOD_MS = max(1, 1000 // REPLY_HZ)

# ---------- 目标丢失时 dx/dy 填什么 ----------
NO_TARGET_DXDY = 0

# ---------- flag 锁存 ----------
# False(默认) = flag 每帧实时反映：目标从视野消失 → 立刻回 flag=0
# True        = 条件满足后一直保持 flag=1 不放（写设计书时"flag=1 电控执行下一步"那种用法，
#               只对 0x03 还有意义；0x02/0x04/0x05 目标丢了照样回 0）
FLAG_LATCH = False

# ---------- 目标消失时 dx/dy 发什么 ----------
# False(默认) = 发 0,0（配 flag=0，明确告诉电控"我现在看不到目标"）
# True        = 仍发消失前最后一次的 dx/dy（但 flag 一定是 0）
KEEP_LAST_DXDY_WHEN_LOST = False

# ---------- 丢帧宽限（防止 flag 抖动） ----------
# 0(默认) = 一帧没检到就立刻回 flag=0（你要的行为）
# 3~5    = 连续丢 N 帧以内还当作"目标还在"（保持 flag=1 或圆环编号 + 上一次的 dx/dy），
#          超过 N 帧才回 flag=0。模型置信度卡在阈值附近（0.5 上下）时会掉帧，
#          电控看起来就是 flag 1/0 乱抖，这时把它设成 3~5。
LOST_GRACE_FRAMES = 0

# ---------- 位置低通（发给电控的 dx/dy 中心点滤波） ----------
LOWPASS_ALPHA = 0.4
MAX_JUMP_PX   = 30

# ==============================================================================
# 一之三之二、0x02「动了 → 又停稳」事件检测
# ==============================================================================
# 要的效果：收到 0x02 时物块是静止的 → flag=0（电控不动手）；
#           物块被拿起/推走（动起来）→ 之后重新停稳 → flag=颜色码；
#           此后锁存，直到下一条指令或换了颜色。
# 状态机：WAIT_MOVE（等它动） → WAIT_STILL（等它停稳） → DONE（报颜色码）
ENABLE_EDGE_STATIC   = True    # 总开关：False = 0x02 只要跟踪到物块就报颜色码（不等动/静）

# ① 「动了」的判据（满足任一就算动了）
MOTION_FROM_ANCHOR_PX = 20     # 相对"收到指令那一刻"的位置，位移超过这么多像素 → 动了
MOTION_SPEED_PX       = 6      # 单帧位移超过这么多像素，且连续 MOTION_SPEED_HITS 帧 → 动了
MOTION_SPEED_HITS     = 2

# ② 「停稳了」的判据：最近 EDGE_STATIC_FRAMES 帧里，原始检测框的
#       中心 x/y 峰峰值 <= EDGE_STATIC_RANGE_PX  且  宽/高峰峰值 <= EDGE_STATIC_SIZE_PX
EDGE_STATIC_FRAMES   = 8      # 统计窗口（帧），约 0.3s
EDGE_STATIC_RANGE_PX = 10     # 窗口内中心峰峰值门限（px）
EDGE_STATIC_SIZE_PX  = 20     # 窗口内宽高峰峰值门限（px），设 999 = 只看中心不看尺寸
# 调参：判不到"停稳" → RANGE/SIZE 调大；动不动就判成停稳 → 调小；
#       拿着慢慢挪也说没动 → MOTION_FROM_ANCHOR_PX 调小

# ---------- 动/静判定的"重新开始"策略 ----------
# 收到指令后要不要清空动/静数据（hist/still）重新判：
#   REARM_ALWAYS = True（默认，你们是"发一次"）：每条指令都清空重判，语义最简单
#   REARM_ALWAYS = False：只有"距上一条同类指令超过 REARM_MIN_GAP_MS"才清空重判
#       —— 电控周期性反复发 AA 02 BB 时用它，否则每帧清零永远判不到静止
REARM_ALWAYS      = True
REARM_MIN_GAP_MS  = 300       # 仅 REARM_ALWAYS=False 时生效：同类指令间隔小于该值视为重复请求

# ---------- 圆环（0x04）滤波 ----------
RING_LOWPASS_ALPHA = 0.25     # 圆环中心低通（比全局 0.4 更稳）
RING_HOLD_FRAMES   = 6        # 圆环丢检后仍保持编号+偏差的帧数（治"时有时无"）
# ★ 0x04 只返回这些圆环编号的坐标（默认只 two=2；一个都没检到就 flag=0）
#   想同时认 three 就写 (2, 3)；想恢复"见谁发谁"就写 (1, 2, 3)
RING_TARGET_NUMS   = (2,)

# ---------- 霍夫圆（0x04：识别到 two 时用霍夫圆心修正 dx/dy） ----------
# 移植自 K230 圆环版本：ROI 搜索 → 候选过滤 → 同心圆聚类 → 关联选目标
#                        → 圆心/半径平滑 + 卡尔曼
ENABLE_HOUGH        = False    # ★宏开关：False = 完全不跑霍夫圆（省 CPU）
HOUGH_FOR_RING_NUMS = {2}     # 哪些圆环编号跑霍夫圆（2 = two）
HOUGH_R_FROM_HINT   = True    # 半径范围按 two 的 YOLO 框自适应（换分辨率不用改）
HOUGH_R_HINT_MIN_F  = 0.15    # r_min = 框短边 * 该系数
HOUGH_R_HINT_MAX_F  = 1.20    # r_max = 框长边 * 该系数
HOUGH_R_MIN         = 10      # 不自适应时的固定半径范围
HOUGH_R_MAX         = 80
HOUGH_R_STEP        = 3       # 半径搜索步长（越大越快越粗）
HOUGH_THRESHOLD     = 1600    # 霍夫累加阈值（K230 那份用的 1600，小了灵敏但易误检）
HOUGH_MARGIN        = 10      # x/y/r 合并边界
HOUGH_ROI_PAD       = 30      # 以 two 框为中心搜索的额外外扩像素
HOUGH_TRACK_ROI_PAD = 45      # 锁定后只在目标附近搜索的外扩像素
HOUGH_MERGE_DIST    = 8.0     # 同心圆归并：两圆心距离小于该值算同一个环
HOUGH_TRACK_GATE    = 45.0    # 与上一帧目标的关联门限（像素）
HOUGH_R_RATIO_MAX   = 1.50    # 半径相对上一帧允许的最大变化倍率
HOUGH_XY_ALPHA      = 0.60    # 圆心指数平滑（越大跟得越快）
HOUGH_R_ALPHA       = 0.35    # 半径指数平滑（越小半径越稳）
HOUGH_LOST_FRAMES   = 6       # 连续丢失超过该帧数 → 放弃目标
HOUGH_CONFIRM_FRAMES = 2      # 新目标连续命中几帧才认为锁定（抗单帧误检）
HOUGH_VOTE_BONUS    = 0.85    # 同心环每多检出一环，得分乘该系数
HOUGH_HOLD_FRAMES   = 4       # 短暂丢失时沿用上次圆心（0 = 丢失立刻不用）
HOUGH_KF_Q          = 0.05    # 卡尔曼过程噪声
HOUGH_KF_R          = 5.0     # 卡尔曼观测噪声
HOUGH_EDGE_MARGIN   = 6       # 圆心离画面边缘少于该像素 → 丢弃（被截断的圆）
HOUGH_BLOCK_OFFSET  = 8       # 圆心落在颜色物块框外扩该像素内 → 丢弃
HOUGH_EVERY_N       = 1       # 每 N 帧跑一次霍夫（FPS 掉了就调大）

# ==============================================================================
# 一之二、0x06 圆环连线校准（用 one/two/three 的连线修车头方向，不漂）
# ==============================================================================
# 思路：陀螺仪会漂，但这三个圆环在场地上的位置是固定的 ——
#       同时看到其中 ≥2 个时，把它们（畸变校正后的）中心连成一条直线，
#       这条线的方向就是场地的一条绝对参考线；与"车头方向"比较得到角度误差，
#       直线到画面中心的垂直距离就是横向偏差，一起发给电控修正。
ENABLE_LINE_CALIB     = True
# ★ 测量方法（这是关键）：
#   "bearing"（推荐）：只用一对圆环，把两个中心换算成"相对光轴的水平方位角"，
#       取平均值当作"这对圆环在画面里的方位"。纯偏航时两个方位角同时转 δ，
#       平均值也正好转 δ —— 所以它直接反映【车头转了多少】，和车停在哪里关系很小。
#   "segment"（旧做法）：把若干个中心拟合成一条直线，取直线角度。
#       直线角度是"两点位置之差"的方向，会被透视/安装俯仰和横向位置混进来，
#       所以精度差、容易飘。
LINE_METHOD           = "bearing"
LINE_TARGET_NUMS      = (1, 2)      # ★ 只用 one / two（segment 法可以再放 3 进来）
LINE_FOCAL_PX         = 0           # 0 = 用画面宽度当焦距；可用"已知转角"现场标定修正
LINE_MIN_BASELINE_PX  = 50          # 两点在画面上的间距小于它 → 太远/看不清，flag=0
LINE_MIN_POINTS       = 2           # segment 法：至少几个点才拟合（2 = 直接连线；3 = 最小二乘）
LINE_USE_HOUGH_FOR_TWO = False      # True = two 用霍夫圆心代替 YOLO 框中心（更准但怕误检）
LINE_POINT_ALPHA      = 0.35        # 每个圆环中心的指数平滑系数
LINE_ANGLE_ALPHA      = 0.35        # 偏差角的角度平滑系数（越小越稳）
LINE_OFFSET_ALPHA     = 0.35        # segment 法的横向偏移平滑系数
LINE_HOLD_FRAMES      = 8           # 点不够时沿用上次结果最多几帧
# ★ 车头参考：bearing 法里 0 = 正前方(光轴)，因为方位角本身就是相对光轴量的；
#   segment 法里 -90 = 画面正上方（y 向下）。两种方法不要混用同一个值。
LINE_CAR_FORWARD_DEG  = 0.0
LINE_ANGLE_SIGN       = 1           # 角度误差符号：装反了/定义不同就改成 -1
LINE_TENTH_DEG        = True        # True = 角度按 0.1° 为单位发（int16），False = 整度
LINE_MAX_ANGLE_DEG    = 45.0        # 超过这个角度认为不可信，按无效处理
LINE_DRAW             = True        # 在画面上画出两点连线
LINE_DRAW_LEN         = 160         # segment 法画线时沿方向向两端延伸的像素（只为显示）

# ---------- 镜头畸变校正（Brown-Conrady，先把点校正再做直线拟合） ----------
# 参数全为 0 时原样返回，等于没开。桶形畸变一般 K1 填负数（-0.05 ~ -0.30），
# 校正后场地里的直线在画面上才会真的是直线（否则拟合出来的角度会有偏差）。
DIST_ENABLE = True
DIST_K1     = 0.0     # 径向一阶
DIST_K2     = 0.0     # 径向二阶
DIST_P1     = 0.0     # 切向
DIST_P2     = 0.0
DIST_FX     = 0       # 0 = 自动用画面宽度当焦距（无标定时够用）
DIST_FY     = 0       # 0 = 自动用画面高度
DIST_CX     = 0       # 0 = 自动用画面中心
DIST_CY     = 0

# ==============================================================================
# 一之三、画框滤波 + 画框样式（让 MaixVision 上画出来的框不抖）
# ==============================================================================
# 原理：把前后帧的检出按"同类 + IoU/中心距离"关联成一条轨迹，
#       对 x/y/w/h 做低通(指数滑动平均)，再输出平滑后的框。
BOX_FILTER_ON         = True
BOX_FILTER_ALPHA      = 0.35   # 0~1，越小框越稳、但跟随越迟钝（0.25~0.5 比较合适）
BOX_FILTER_IOU_MATCH  = 0.10   # 前后帧 IoU 超过它就认为是同一个目标
BOX_FILTER_MAX_JUMP   = 45     # 单帧中心跳变超过这个像素 → 直接重新锁定（跟丢/跟错时快速纠正）
BOX_FILTER_KEEP_MISS  = 5      # 目标消失后轨迹保留几帧（防闪断，重新出现时接着原轨迹平滑）
BOX_FILTER_MIN_HITS   = 2      # 连续出现几帧才画出来（滤掉一闪而过的误检）
# 滤波后的框同时用于画图和给电控算 dx/dy，所以控制量也会一起变稳。
# 觉得响应太慢就把 ALPHA 调大(0.5~0.7)，觉得还抖就调小(0.15~0.25)。

# ---------- 画框样式 ----------
BOX_LINE_THICK    = 2      # 普通检出框线宽（你要"粗一点点"）
TARGET_LINE_THICK = 3      # 当前模式正在跟踪的目标框线宽
DRAW_CROSSHAIR    = True   # 每个框中心画十字准心
CROSSHAIR_SIZE    = 7      # 十字准心的臂长（像素）

# 黑色物块用纯黑框在画面上看不清，默认仍是白框；想真黑就改成 image.COLOR_BLACK
COLOR_BOX_BLACK = getattr(image, "COLOR_BLACK", image.COLOR_WHITE)

# ==============================================================================
# 一之四、类别 → 编码（★按【名字】配，换模型不用改代码★）
# ==============================================================================
# 发给电控的颜色编码（《视觉-电控通信》）：
#     red=1  yellow=2  blue=3  green=4  black=5  light blue=6   没识别到=0x00
# 这里只写"名字"，程序启动时去模型的 labels 里找对应的 class_id，
# 所以模型类别顺序怎么变都不会错（前提：模型里类别名字就是这些）。
COLOR_CODE_BY_NAME = {
    "red":       1,
    "yellow":    2,
    "blue":      3,
    "green":     4,
    "black":     5,
    "lightblue": 6,      # 'light blue' / 'light_blue' / 'light-blue' 都认
}
# 圆环编号（0x04 的 flag）：one=1  two=2  three=3
RING_NUM_BY_NAME = {"one": 1, "two": 2, "three": 3}
# 圆盘类别名：只用来"不画框 / 不参与 0x02"，模型里没这一类也不影响
DISK_NAME = "disk"

# ★ 兜底表：只有在"读不到模型 labels"时才会用到（旧模型 class_id → 名字）
FALLBACK_LABELS = {
    0: "blue", 1: "light blue", 2: "yellow", 3: "three", 4: "two",
    5: "disk", 6: "black", 7: "green", 8: "red", 9: "one",
}

# 画框用的颜色/字符（只影响 MaixVision 上的显示）
# 什么颜色的物块就画什么颜色的框；不在表里的（圆盘/数字标记等）默认白框
CODE_COLOR = {
    1: image.COLOR_RED,       # red
    2: image.COLOR_YELLOW,    # yellow
    3: image.COLOR_BLUE,      # blue
    4: image.COLOR_GREEN,     # green
    5: COLOR_BOX_BLACK,       # black
    6: image.COLOR_PURPLE,    # light blue（没有"浅蓝"这个常量，用紫色区分）
}
CODE_CHAR = {1: "R", 2: "Y", 3: "B", 4: "G", 5: "K", 6: "c", 0: "-"}

# ---------- 0x03 的三个固定框：c1=左上 c2=下 c3=右上 ----------
# 坐标是按 ROI_BASE 这个输入尺寸量的；换模型后输入尺寸变了会自动按比例缩放
# 这个模型训练时就是缩放到 320x320 的，所以这里保持 320
ROI_BASE = 320
ROI_BOX = [
    [0,  10,  100, 100],      # c1 左上
    [110, 230,  100, 100],     # c2 下
    [220, 10,   100, 100],     # c3 右上
]
# 三个框里至少几个就绪 → flag=1（你要的"两个及以上"就是 2）
BOX_READY_MIN = 2
# ---------- 0x03 排除法：已知 2 个框的颜色 → 用 0x01 任务码推出第 3 个 ----------
# 逻辑：任务码（0x01 的 c1 c2 c3）就是这一轮要处理的 3 个颜色，
#       如果两个框已经认到颜色、第三个框空着，那么把"任务码里剩下的那个颜色"填进去。
#       这样即使第三个物块没看到，回帧的三个颜色也是完整的、且顺序固定（左上/下/右上）。
BOX_DEDUCE_BY_TASK     = True   # 总开关
BOX_DEDUCE_NEED_READY  = True   # True = 只有"已就绪(R)"的框才算认出颜色（抗瞬时误检）
# ---------- 0x03 重新统计节奏 ----------
# 每 BOX_EVAL_PERIOD_MS 重新数一次"认出几个框"，据此决定怎么发：
#   3 个 → 原样发；2 个 → 任务码排除法补第 3 个；
#   0/1 个 → 清掉三个框里的颜色锁存，继续下一轮识别（避免卡在只认到一个的状态）
BOX_EVAL_PERIOD_MS     = 300
# 就绪判据：设计书写的是"等待到框内从无到有"，想用"静止N帧"就改成 "static"
BOX_READY_MODE    = "empty_to_full"
ROI_STATIC_FRAMES = 12
ROI_MOVE_THRESH   = 4.0

# ---------- 补光灯（MaixCAM-Pro: B3 是照明 LED，高电平使能） ----------
ENABLE_FILL_LIGHT = True    # 硬件总开关：False = 完全不初始化 B3
# ★ 按模式控制补光灯：True = 只在【工作模式】点亮，空闲(0x00)时熄灭
#   工作模式 = 除 0x00 以外的所有模式（0x01~0x07），收到指令就亮，回空闲就灭
#   写 False 就回到"上电后一直保持 FILL_LIGHT_OFF_LEVEL 不变"的老行为
LIGHT_ON_IN_MODE     = True
FILL_LIGHT_ON_LEVEL  = 1    # 点亮电平（若你的板子是低电平点亮，把它改成 0、下面改成 1）
FILL_LIGHT_OFF_LEVEL = 0    # 熄灭电平
FILL_LIGHT_LEVEL     = 0    # 上电初始电平（一般就是熄灭）

# ---------- 调试开关 ----------
DEBUG_SHOW     = True
DRAW_DETECTIONS = True     # 把所有识别到的目标都画框
DEBUG_PRINT_DETECT = True  # 每 1 秒在控制台打印一次当前检出（查 class_id 用）
# ★ 默认关闭！它会把 MaixVision 上报帧打到系统 maix 协议串口(UART0,115200)上，
#   和《视觉-电控通信》的 [AA ... BB] 帧混在一起。
DEBUG_REPORT   = False
DEBUG_PRINT_RX = False
DEBUG_PRINT_TX = False

# ==============================================================================
# 二、协议常量 + 全局状态
# ==============================================================================
FRAME_HEAD = 0xAA
FRAME_TAIL = 0xBB
CMD_IDLE  = 0x00
CMD_TASK  = 0x01
CMD_NEAR  = 0x02     # 最近物块 + 边沿静止判断（原来这里是"圆盘"）
CMD_BOXES = 0x03
CMD_RING  = 0x04
CMD_COLOR = 0x05
CMD_LINE  = 0x06     # 圆环连线校准车头方向
CMD_LREF  = 0x07     # 现场标定：把当前连线角度记为"车头正对"的参考
# 每条命令 payload 的固定长度（设计书规定）
# ★ 必须按长度收帧：dxL/dyL/颜色码完全可能是 0xBB，用"找 0xBB 当帧尾"会截断
CMD_LEN = {
    CMD_IDLE:  0,   # [AA 00 BB]
    CMD_TASK:  3,   # [AA 01 c1 c2 c3 BB]
    CMD_NEAR:  0,   # [AA 02 BB]
    CMD_BOXES: 0,   # [AA 03 BB]
    CMD_RING:  0,   # [AA 04 BB]
    CMD_COLOR: 1,   # [AA 05 color BB]
    CMD_LINE:  0,   # [AA 06 BB]
    CMD_LREF:  0,   # [AA 07 BB]
}

MODE_IDLE  = "IDLE"
MODE_TASK  = "TASK"
MODE_NEAR  = "NEAR"      # 0x02 最近物块
MODE_BOXES = "BOXES"
MODE_RING  = "RING"
MODE_COLOR = "COLOR"
MODE_LINE  = "LINE"      # 0x06 圆环连线校准

flag = 0
mode = MODE_IDLE
target_color = 0
task_code = (0, 0, 0)
reply_at_ms = None
last_near  = (NO_TARGET_DXDY, NO_TARGET_DXDY)
last_ring  = (NO_TARGET_DXDY, NO_TARGET_DXDY)
last_color = (NO_TARGET_DXDY, NO_TARGET_DXDY)
ring_num_last = 0
last_cmd_func = -1        # 上一条指令（用于判断"重复请求"）
last_cmd_ms   = -100000   # 上一条指令的时刻
line_capture_pending = False   # 0x07：等下一帧有有效连线就记录参考角
box_eval_ms   = -100000   # 0x03：上次"重新统计认到几个框"的时刻
box_n_known   = 0         # 0x03：上次统计时认到颜色的框数
box_state     = "IDLE"    # 0x03：FULL(3个) / DEDUCE(2个→排除法) / RETRY(0~1个→清锁存重来)

# ==============================================================================
# 三、硬件初始化
# ==============================================================================
if UART_OPTION not in UART_PLAN:
    raise ValueError("UART_OPTION 只能是 %s" % list(UART_PLAN.keys()))
_plan = UART_PLAN[UART_OPTION]

print("[UART] 可用设备:", uart.list_devices())
for _pin, _func in _plan["pins"]:
    _r = pinmap.set_pin_function(_pin, _func)
    err.check_raise(_r, "pinmap 失败: %s -> %s" % (_pin, _func))
    print("[UART] pinmap OK: %s -> %s" % (_pin, _func))
serial = uart.UART(_plan["dev"], UART_BAUD)
print("[UART] 使用 %s @ %d bps (方案 %s)" % (_plan["dev"], UART_BAUD, UART_OPTION))

led = None
light_state = None          # 当前电平（None = 还没写过）
if ENABLE_FILL_LIGHT:
    try:
        pinmap.set_pin_function("B3", "GPIOB3")
        led = gpio.GPIO("GPIOB3", gpio.Mode.OUT)
        led.value(FILL_LIGHT_LEVEL)
        light_state = FILL_LIGHT_LEVEL
        print("[LED] 补光灯就绪：上电初始电平 = %d" % FILL_LIGHT_LEVEL)
    except Exception as e:
        print("[LED] 补光灯初始化失败(忽略):", e)
        led = None


def set_light(on):
    """按模式控制补光灯（LIGHT_ON_IN_MODE=False 时什么都不做）

    只在电平真正变化时写一次 GPIO，避免每帧都写。
    """
    global light_state
    if (not ENABLE_FILL_LIGHT) or (led is None) or (not LIGHT_ON_IN_MODE):
        return
    lv = FILL_LIGHT_ON_LEVEL if on else FILL_LIGHT_OFF_LEVEL
    if lv == light_state:
        return
    try:
        led.value(lv)
        light_state = lv
    except Exception as e:
        print("[LED] 设置补光灯失败:", e)


def _try_load_model(path):
    """依次用 nn.YOLOv5 / YOLO11 / YOLOv8 试加载（换模型类型也不用改代码）

    返回 (detector, 类别名) 或 (None, 错误说明)
    """
    errs = []
    for cls_name in ("YOLOv5", "YOLO11", "YOLOv8"):
        cls = getattr(nn, cls_name, None)
        if cls is None:
            continue                      # 这个 MaixPy 版本没有这个类
        try:
            return cls(model=path), cls_name
        except Exception as e:
            errs.append("%s: %s" % (cls_name, e))
    return None, " | ".join(errs) if errs else "nn 里没有可用的检测器类"

def _list_models_on_device():
    """没加载成功时，把板子上能找到的 .mud 列出来，方便改路径"""
    print("[MODEL] 板子上的 .mud 文件：")
    found = False
    for d in ("/root", "/root/models", "/root/models/training1",
              "/root/models/training2", "/root/models/disk", "/root/models/training3"):
        try:
            fs = sorted(f for f in os.listdir(d) if f.lower().endswith(".mud"))
        except Exception:
            continue
        if fs:
            found = True
            print("        %-24s %s" % (d, fs))
    if not found:
        print("        （一个都没找到：模型可能还在电脑上，需要用 MaixVision 的文件管理上传到 /root/models）")

def load_detector():
    """按 MODEL_CANDIDATES 顺序找模型，逐个尝试加载；全失败才抛异常"""
    tried = []
    for p in MODEL_CANDIDATES:
        if p in tried:
            continue
        tried.append(p)
        if not os.path.exists(p):
            print("[MODEL] 不存在，跳过:", p)
            continue
        print("[MODEL] 尝试加载:", p)
        det, who = _try_load_model(p)
        if det is None:
            print("[MODEL] !! 加载失败:", p)
            print("[MODEL] !! 原因:", who)
            continue
        try:
            print("[MODEL] 加载成功(%s): %s" % (who, p))
            print("[MODEL] 输入 %dx%d 格式 %s，类别数 %d"
                  % (det.input_width(), det.input_height(),
                     det.input_format(), len(det.labels)))
        except Exception as e:
            print("[MODEL] （读模型信息失败，但不影响运行）:", e)
        return det
    print("[MODEL] **********************************************************")
    print("[MODEL] * 所有候选模型都没加载成功！")
    print("[MODEL] * 检查 ① 文件是否已上传到板子 ② 路径是否写对 ③ 模型是不是 MaixPy 支持的格式")
    print("[MODEL] **********************************************************")
    _list_models_on_device()
    raise RuntimeError("模型加载失败，请看上面的 [MODEL] 输出")

# 使用模型（换模型只需改上面 MODEL_PATH / MODEL_CANDIDATES）
detector = load_detector()
cam  = camera.Camera(detector.input_width(), detector.input_height(),
                     detector.input_format())
disp = display.Display()

TARGET_X = detector.input_width()  // 2
TARGET_Y = detector.input_height() // 2

# ---------- dx/dy 换算系数（只用于发给电控的偏差，画框/阈值不受影响） ----------
if DXDY_REF_ENABLE:
    DXDY_KX = DXDY_REF_W / float(max(1, detector.input_width()))
    DXDY_KY = DXDY_REF_H / float(max(1, detector.input_height()))
else:
    DXDY_KX = 1.0
    DXDY_KY = 1.0
print("[SCALE] dx/dy 输出坐标系: %s（输入 %dx%d → 系数 %.4f / %.4f）"
      % (("按 %dx%d 归一" % (DXDY_REF_W, DXDY_REF_H)) if DXDY_REF_ENABLE else "原始像素",
         detector.input_width(), detector.input_height(), DXDY_KX, DXDY_KY))

# ---------- 瞄准点（dx/dy 的零点 = 画面中心 + 夹爪偏置） ----------
# ★ 只影响 dx/dy 的"零点"，画框用的十字仍在画面中心（TARGET_X / TARGET_Y）
if DXDY_OFFSET_ENABLE:
    AIM_X = TARGET_X + DXDY_OFFSET_X
    AIM_Y = TARGET_Y + DXDY_OFFSET_Y
else:
    AIM_X, AIM_Y = TARGET_X, TARGET_Y
print("[AIM] dx/dy 瞄准点 = (%d, %d)：画面中心 (%d, %d) + 夹爪偏置 (%+d, %+d)"
      % (AIM_X, AIM_Y, TARGET_X, TARGET_Y,
         AIM_X - TARGET_X, AIM_Y - TARGET_Y))
if DXDY_OFFSET_ENABLE and (DXDY_OFFSET_X or DXDY_OFFSET_Y):
    print("[AIM] 说明：偏置为【正】表示落点偏右/偏下，代码里做减法 → "
          "物块要落在瞄准点上电控才读到 0")

# ---------- 三个 ROI 框按模型输入尺寸自动缩放（换模型不会框错位置） ----------
ROI_SCALE = detector.input_width() / float(max(1, ROI_BASE))
if abs(ROI_SCALE - 1.0) > 1e-3:
    ROI_BOX = [[int(round(x * ROI_SCALE)), int(round(y * ROI_SCALE)),
                int(round(w * ROI_SCALE)), int(round(h * ROI_SCALE))]
               for (x, y, w, h) in ROI_BOX]
    print("[MODEL] 输入 %dx%d → 0x03 的三个框按 %.3f 倍缩放: %s"
          % (detector.input_width(), detector.input_height(), ROI_SCALE, ROI_BOX))
    print("[MODEL] 注意: 其它像素阈值(MOTION_*/EDGE_STATIC_*/HOUGH_R_*/ROI_MOVE_THRESH)"
          " 还是按 320 写的，需要的话按同一比例放大")
else:
    print("[MODEL] 0x03 三个框: %s" % ROI_BOX)

# ==============================================================================
# 四、从模型的 labels 建立 class_id ↔ 名字 ↔ 编码 对照（换模型自动适配）
# ==============================================================================
def canon(s):
    """类别名归一化：小写 + 去掉空格/下划线/横杠
    'light blue' / 'LightBlue' / 'light-blue' / 'light_blue' → 'lightblue'"""
    return str(s).strip().lower().replace(" ", "").replace("_", "").replace("-", "")

try:
    LABELS = [str(s) for s in detector.labels]
except Exception as e:
    print("[MODEL] 读 labels 失败:", e)
    LABELS = []

if not LABELS:
    print("[MODEL] !! 读不到模型 labels，改用代码里的兜底类别表")
    LABELS = ["cls%d" % i for i in range(max(FALLBACK_LABELS) + 1)]
    for _k, _v in FALLBACK_LABELS.items():
        LABELS[_k] = _v

# 名字 → class_id
ID_BY_NAME = {}
for _i, _n in enumerate(LABELS):
    ID_BY_NAME.setdefault(canon(_n), _i)

# class_id → 颜色编码 / 颜色编码 → class_id
CODE_BY_CLASS = {}
for _nm, _code in COLOR_CODE_BY_NAME.items():
    _cid = ID_BY_NAME.get(_nm)
    if _cid is not None:
        CODE_BY_CLASS[_cid] = _code
CLASS_BY_CODE = {code: cid for cid, code in CODE_BY_CLASS.items()}

# class_id → 圆环编号 / 圆环编号 → class_id
RING_NUM_BY_CLASS = {}
for _nm, _num in RING_NUM_BY_NAME.items():
    _cid = ID_BY_NAME.get(_nm)
    if _cid is not None:
        RING_NUM_BY_CLASS[_cid] = _num
RING_CLASS_BY_NUM = {num: cid for cid, num in RING_NUM_BY_CLASS.items()}

DISK_CLASS_ID   = ID_BY_NAME.get(DISK_NAME)          # 没有 disk 这一类就是 None
BLOCK_CLASS_IDS = set(CODE_BY_CLASS)                 # 0x02 候选：6 种颜色物块
HIDE_CLASS_IDS  = set() if DISK_CLASS_ID is None else {DISK_CLASS_ID}
BOX_IGNORE_IDS  = set(RING_NUM_BY_CLASS) | HIDE_CLASS_IDS

def name_of(cid):
    try:
        cid = int(cid)
    except Exception:
        return "?"
    return LABELS[cid] if 0 <= cid < len(LABELS) else ("cls%d" % cid)

def out_code(cid):
    """模型 class_id → 发给电控的颜色编码；圆盘/数字标记/未知一律 0x00"""
    try:
        return CODE_BY_CLASS.get(int(cid), 0)
    except Exception:
        return 0

# ---- 启动打印对照表：跑一次就知道每个 class_id 对应什么 ----
print("[MODEL] 类别数 = %d，class_id → 名字 → 发给电控：" % len(LABELS))
for _i, _n in enumerate(LABELS):
    _c = canon(_n)
    if _c in COLOR_CODE_BY_NAME:
        _tag = "→ 颜色编码 %d" % COLOR_CODE_BY_NAME[_c]
    elif _c in RING_NUM_BY_NAME:
        _tag = "→ 圆环编号 %d" % RING_NUM_BY_NAME[_c]
    elif _c == canon(DISK_NAME):
        _tag = "→ 圆盘（不画框、不参与 0x02）"
    else:
        _tag = "→ !! 代码不认识这个名字，不会发给电控"
    print("      %2d: %-14s %s" % (_i, _n, _tag))

_missing = [n for n in list(COLOR_CODE_BY_NAME) + list(RING_NUM_BY_NAME) + [DISK_NAME]
            if n not in ID_BY_NAME]
if _missing:
    print("[MODEL] !! 这份模型里没有这些类别:", _missing)
    print("[MODEL] !! 要么训练时没包含它们，要么名字拼写不同（light_blue / light blue 都认）")
print("[MODEL] 0x02 候选物块 class_id = %s；圆环 one/two/three = %s；忽略/不画 = %s"
      % (sorted(BLOCK_CLASS_IDS), RING_CLASS_BY_NUM, sorted(BOX_IGNORE_IDS)))
print("[MODEL] 0x02 动/静事件: %s；动了判据 位移>=%dpx 或 速度>=%dpx/帧×%d帧；"
      "停稳判据 窗口%df 中心峰峰值<=%dpx 宽高<=%dpx"
      % ("开" if ENABLE_EDGE_STATIC else "关（跟踪到就报颜色码）",
         MOTION_FROM_ANCHOR_PX, MOTION_SPEED_PX, MOTION_SPEED_HITS,
         EDGE_STATIC_FRAMES, EDGE_STATIC_RANGE_PX, EDGE_STATIC_SIZE_PX))
print("[MODEL] 动/静重新开始: %s（同类指令间隔 < %dms 视为重复请求，不重置）"
      % ("每条指令都重置" if REARM_ALWAYS else "仅在非重复指令时重置", REARM_MIN_GAP_MS))
print("[MODEL] 0x03 排除法: %s（只把已就绪R的框算作已认到=%s）；最少就绪框=%d；"
      "每%dms重新统计；当前任务码=%s"
      % ("开" if BOX_DEDUCE_BY_TASK else "关", BOX_DEDUCE_NEED_READY,
         BOX_READY_MIN, BOX_EVAL_PERIOD_MS, task_code))
print("[MODEL] 圆环(0x04): 只认编号%s；低通alpha=%.2f 丢帧保持=%df；"
      "霍夫圆=%s（对编号%s，每%df跑一次，阈值%d）"
      % (list(RING_TARGET_NUMS),
         RING_LOWPASS_ALPHA, RING_HOLD_FRAMES,
         "开" if ENABLE_HOUGH else "关", sorted(HOUGH_FOR_RING_NUMS),
         HOUGH_EVERY_N, HOUGH_THRESHOLD))
print("[MODEL] 0x06 车头校准: %s；方法=%s 用圆环%s；最少%d点；车头参考=%.1f°；"
      "角度单位=%s；焦距=%s；最小基线=%dpx；畸变校正=%s(k1=%.3f k2=%.3f)"
      % ("开" if ENABLE_LINE_CALIB else "关",
         LINE_METHOD, list(LINE_TARGET_NUMS), LINE_MIN_POINTS,
         LINE_CAR_FORWARD_DEG,
         "0.1°" if LINE_TENTH_DEG else "1°",
         ("%dpx" % LINE_FOCAL_PX) if LINE_FOCAL_PX else "自动(画面宽)",
         LINE_MIN_BASELINE_PX,
         "开" if DIST_ENABLE else "关", DIST_K1, DIST_K2))
print("[MODEL] 补光灯: %s；按模式控制=%s（工作模式电平 %d / 空闲电平 %d）"
      % ("硬件已使能" if ENABLE_FILL_LIGHT else "关闭",
         "开" if LIGHT_ON_IN_MODE else "关（保持初始电平）",
         FILL_LIGHT_ON_LEVEL, FILL_LIGHT_OFF_LEVEL))
print("-" * 62)

reporter = comm.CommProtocol(buff_size=1024) if DEBUG_REPORT else None
APP_CMD_DETECT_RES = 0x02

# ==============================================================================
# 五、串口层
# ==============================================================================
def pack_i16(val):
    p = struct.pack(">h" if INT16_BIG_ENDIAN else "<h", int(val))
    return p[0], p[1]

def reply_dxdy(func, flag_byte, dx, dy):
    """统一出口：所有带 dx/dy 的回帧都经过这里

    在这里把偏差换算到 DXDY_REF_W x DXDY_REF_H（默认 320x320）参考坐标系，
    这样换分辨率后电控的 PID 参数不用跟着改。
    """
    if DXDY_REF_ENABLE:
        dx = int(round(dx * DXDY_KX))
        dy = int(round(dy * DXDY_KY))
    x1, x2 = pack_i16(dx)
    y1, y2 = pack_i16(dy)
    return (func, bytes([flag_byte & 0xFF, x1, x2, y1, y2]))

def cmd_send(func, payload=b""):
    frame = bytes([FRAME_HEAD, func]) + bytes(payload) + bytes([FRAME_TAIL])
    serial.write(frame)
    if DEBUG_PRINT_TX:
        print("TX", frame.hex(" "))

class CmdParser:
    """按 [AA func payload... BB] 固定长度解析。

    好处：payload 里出现 0xAA / 0xBB 也不会误判（0x05 的颜色码、将来扩展的参数）。
    """
    MAX_BUF = 512

    def __init__(self):
        self.buf = bytearray()

    def feed(self, chunk):
        out = []
        if chunk:
            self.buf += chunk
            if len(self.buf) > self.MAX_BUF:
                del self.buf[:-self.MAX_BUF]
        while True:
            if len(self.buf) < 2:
                break
            if self.buf[0] != FRAME_HEAD:
                del self.buf[0]
                continue
            func = self.buf[1]
            if func not in CMD_LEN:
                del self.buf[0]
                continue
            total = 3 + CMD_LEN[func]          # AA + func + payload + BB
            if len(self.buf) < total:
                break
            if self.buf[total - 1] != FRAME_TAIL:
                del self.buf[0]                # 帧尾不对，丢掉这个 AA 重新找
                continue
            out.append((func, bytes(self.buf[2:total - 1])))
            del self.buf[:total]
        return out

# ==============================================================================
# 六、检测层（YOLOv5）
# ==============================================================================
def obj_center(o):
    return o.x + o.w // 2, o.y + o.h // 2

def point_in_rect(px, py, rx, ry, rw, rh):
    return (rx <= px <= rx + rw) and (ry <= py <= ry + rh)

def pick_max(class_iter):
    best = None
    for o in class_iter:
        if best is None or o.w * o.h > best.w * best.h:
            best = o
    return best

def pick_class(objs, cid):
    if cid is None:
        return None
    return pick_max([o for o in objs if o.class_id == cid])

def pick_nearest(objs, only_ids=None):
    """挑离画面中心(十字准心)最近的物块；only_ids 限制候选类别"""
    best, best_d = None, None
    for o in objs:
        if only_ids is not None and o.class_id not in only_ids:
            continue
        cx, cy = obj_center(o)
        d = (cx - TARGET_X) ** 2 + (cy - TARGET_Y) ** 2
        if best is None or d < best_d:
            best, best_d = o, d
    return best

# 本帧的原始检出（未经框滤波）：边沿静止判据要用它，滤波后的框位移被压小了判不准
RAW_OBJS = []

def find_same_det(o, objs):
    """在 objs 里找和 o 同一个目标（类别相同 + 中心最近）"""
    best, best_d = None, None
    cx, cy = obj_center(o)
    for r in objs:
        if int(r.class_id) != int(o.class_id):
            continue
        rx, ry = obj_center(r)
        d = (rx - cx) ** 2 + (ry - cy) ** 2
        if best is None or d < best_d:
            best, best_d = r, d
    return best

def pick_in_roi(objs, roi, ignore_ids=()):
    rx, ry, rw, rh = roi
    hit = []
    for o in objs:
        if o.class_id in ignore_ids:
            continue
        cx, cy = obj_center(o)
        if point_in_rect(cx, cy, rx, ry, rw, rh):
            hit.append(o)
    return pick_max(hit)

def encode_objs(objs):
    """仅 DEBUG_REPORT 用，上报给 MaixVision"""
    body = b""
    for o in objs:
        body += struct.pack("<hhHHHf", o.x, o.y, o.w, o.h, o.class_id, o.score)
    return body

# ------------------------------------------------------------------------------
# 画框滤波：把抖动的检出框平滑掉（同时让发给电控的 dx/dy 也更稳）
# ------------------------------------------------------------------------------
class Det:
    """滤波后的检出（普通 Python 对象，字段和 YOLO 的一样：x/y/w/h/class_id/score）"""
    __slots__ = ("x", "y", "w", "h", "class_id", "score")
    def __init__(self, x, y, w, h, class_id, score):
        self.x = int(round(x))
        self.y = int(round(y))
        self.w = int(round(w))
        self.h = int(round(h))
        self.class_id = class_id
        self.score = score

class _Track:
    __slots__ = ("cid", "x", "y", "w", "h", "score", "hits", "miss")
    def __init__(self, o):
        self.cid   = int(o.class_id)
        self.x     = float(o.x)
        self.y     = float(o.y)
        self.w     = float(o.w)
        self.h     = float(o.h)
        self.score = float(o.score)
        self.hits  = 1
        self.miss  = 0

def _iou(a, b):
    """a, b = (x, y, w, h)"""
    ax2, ay2 = a[0] + a[2], a[1] + a[3]
    bx2, by2 = b[0] + b[2], b[1] + b[3]
    ix = min(ax2, bx2) - max(a[0], b[0])
    iy = min(ay2, by2) - max(a[1], b[1])
    if ix <= 0 or iy <= 0:
        return 0.0
    inter = ix * iy
    union = a[2] * a[3] + b[2] * b[3] - inter
    return inter / union if union > 0 else 0.0

class BoxFilter:
    """前后帧关联 + 低通，抑制框的抖动和闪烁

    关联规则：同一个 class_id，且
        IoU >= BOX_FILTER_IOU_MATCH               （正常情况）
        或 中心距离 <= max(25, 0.9 * 框尺寸)      （快速移动导致 IoU 掉到 0 时兜底）
    输出规则：连续出现 >= BOX_FILTER_MIN_HITS 帧、且本帧有检出的轨迹才输出
    """

    def __init__(self):
        self.tracks = []

    def reset(self):
        self.tracks = []

    def apply(self, objs):
        used = [False] * len(objs)
        a = BOX_FILTER_ALPHA

        for t in self.tracks:
            best_i, best_key = -1, None
            for i, o in enumerate(objs):
                if used[i] or int(o.class_id) != t.cid:
                    continue
                box_o = (o.x, o.y, o.w, o.h)
                v = _iou((t.x, t.y, t.w, t.h), box_o)
                if v >= BOX_FILTER_IOU_MATCH:
                    key = (1, v)                       # IoU 命中优先
                else:
                    tcx, tcy = t.x + t.w * 0.5, t.y + t.h * 0.5
                    ocx, ocy = o.x + o.w * 0.5, o.y + o.h * 0.5
                    d = ((tcx - ocx) ** 2 + (tcy - ocy) ** 2) ** 0.5
                    if d > max(25.0, 0.9 * max(t.w, t.h, o.w, o.h)):
                        continue
                    key = (0, -d)                      # 距离命中次之
                if best_key is None or key > best_key:
                    best_key, best_i = key, i
            if best_i < 0:
                t.miss += 1
                continue

            used[best_i] = True
            o = objs[best_i]
            nx, ny, nw, nh = float(o.x), float(o.y), float(o.w), float(o.h)
            if (abs(nx - t.x) > BOX_FILTER_MAX_JUMP
                    or abs(ny - t.y) > BOX_FILTER_MAX_JUMP
                    or abs(nw - t.w) > BOX_FILTER_MAX_JUMP * 2
                    or abs(nh - t.h) > BOX_FILTER_MAX_JUMP * 2):
                # 跳变太大：不滤波，直接重新锁定（换了目标/跟丢了）
                t.x, t.y, t.w, t.h = nx, ny, nw, nh
            else:
                t.x += a * (nx - t.x)
                t.y += a * (ny - t.y)
                t.w += a * (nw - t.w)
                t.h += a * (nh - t.h)
            t.score = float(o.score)
            t.hits += 1
            t.miss  = 0

        # 没关联上的检出 → 新建轨迹（新目标需要连续出现几帧才会被输出）
        for i, o in enumerate(objs):
            if not used[i]:
                self.tracks.append(_Track(o))

        # 丢掉消失太久的轨迹
        self.tracks = [t for t in self.tracks if t.miss <= BOX_FILTER_KEEP_MISS]

        # 输出平滑后的框
        out = []
        for t in self.tracks:
            if t.hits < BOX_FILTER_MIN_HITS or t.miss > 0:
                continue
            out.append(Det(t.x, t.y, t.w, t.h, t.cid, t.score))
        return out

box_filter = BoxFilter()

# ==============================================================================
# 七、判定工具
# ==============================================================================
def set_flag(v):
    """v=1 表示本帧条件满足；锁存模式下置 1 后不再自动掉回 0"""
    global flag
    if v:
        flag = 1
    elif not FLAG_LATCH:
        flag = 0

def clear_flag():
    global flag
    flag = 0

class Smoother:
    def __init__(self, alpha):
        self.alpha = alpha
        self.est = None
    def update(self, x, y):
        if self.est is None:
            self.est = (float(x), float(y))
        else:
            ex, ey = self.est
            if abs(x - ex) > MAX_JUMP_PX or abs(y - ey) > MAX_JUMP_PX:
                self.est = (float(x), float(y))
            else:
                self.est = (ex + self.alpha * (x - ex),
                            ey + self.alpha * (y - ey))
        return self.est
    def reset(self):
        self.est = None

def wrap_deg(a, half_span=90.0):
    """把角度折到 [-half_span, +half_span)：直线方向有 180° 二义性，用 90 折"""
    return ((a + half_span) % (2.0 * half_span)) - half_span

class AngleSmoother:
    """角度专用平滑：按最短弧做指数平滑，不会在 ±90/±180 处跳变"""
    def __init__(self, alpha, half_span=90.0):
        self.alpha = alpha
        self.half_span = half_span
        self.est = None

    def update(self, measure):
        m = wrap_deg(measure, self.half_span)
        if self.est is None:
            self.est = m
        else:
            diff = wrap_deg(m - self.est, self.half_span)
            self.est = wrap_deg(self.est + self.alpha * diff, self.half_span)
        return self.est

    def reset(self):
        self.est = None

class LostGrace:
    """丢帧宽限计数器：连续丢帧不超过 limit 时仍当作目标还在

    limit 默认取全局 LOST_GRACE_FRAMES；圆环可以单独传更大的值（RING_HOLD_FRAMES），
    避免 0x04 因为偶尔丢检而"时有时无"。
    """
    def __init__(self, limit=None):
        self.limit = LOST_GRACE_FRAMES if limit is None else limit
        self.miss = 0
    def hit(self):
        """本帧没检到目标；返回 True 表示还在宽限帧数内，可以先当作目标还在"""
        self.miss += 1
        return self.miss <= self.limit
    def clear(self):
        """本帧检到目标了"""
        self.miss = 0
    def reset(self):
        self.miss = 0

class MotionEvent:
    """0x02 的「动了 → 又停稳」事件检测（状态机）

    WAIT_MOVE  ：收到指令时物块是静止的 → flag=0，电控不动手
                 （等它动：相对指令时刻位移 > MOTION_FROM_ANCHOR_PX，
                   或单帧速度 > MOTION_SPEED_PX 连续 MOTION_SPEED_HITS 帧）
    WAIT_STILL ：已经动了 → 等它重新停稳
                 （最近 EDGE_STATIC_FRAMES 帧 中心/宽高 峰峰值都在门限内）
    DONE       ：停稳了 → flag=颜色码，锁存到下一條指令 / 换颜色

    换颜色（cid 变）会自动回到 WAIT_MOVE 重新等。
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.phase  = "WAIT_MOVE"
        self.cid    = None
        self.anchor = None      # 收到指令时的中心位置 (cx, cy)
        self.prev   = None      # 上一帧中心（算速度）
        self.hist   = []        # 停稳判定窗口
        self.speed_hits = 0
        self.last_rng   = (0, 0, 0, 0)   # 最近一次峰峰值(中心x,中心y,宽,高)
        self.last_dist  = 0.0            # 最近一次相对锚点位移

    # ---------- 调试 ----------
    def debug_line(self):
        r = self.last_rng
        return ("[STATIC] %s phase=%s 位移%.0f/%dpx 速度命中%d/%d "
                "窗口%d/%d 峰峰值 中心(%.0f,%.0f) 宽高(%.0f,%.0f) 门限(%d/%d)"
                % (name_of(self.cid) if self.cid is not None else "-",
                   self.phase, self.last_dist, MOTION_FROM_ANCHOR_PX,
                   self.speed_hits, MOTION_SPEED_HITS,
                   len(self.hist), EDGE_STATIC_FRAMES,
                   r[0], r[1], r[2], r[3],
                   EDGE_STATIC_RANGE_PX, EDGE_STATIC_SIZE_PX))

    # ---------- 内部 ----------
    def _range(self, i):
        vals = [h[i] for h in self.hist]
        return max(vals) - min(vals)

    def _restart_wait_move(self, cx, cy):
        self.phase      = "WAIT_MOVE"
        self.anchor     = (cx, cy)
        self.hist       = []
        self.speed_hits = 0

    # ---------- 主入口 ----------
    def update(self, o):
        """每帧喂一次当前目标的框；返回 True = 已经"动了又停稳"（可以报颜色码）"""
        cid = int(o.class_id)
        cx  = o.x + o.w * 0.5
        cy  = o.y + o.h * 0.5

        # 换颜色/换目标 → 重新等
        if cid != self.cid:
            self.cid  = cid
            self.prev = (cx, cy)
            self._restart_wait_move(cx, cy)
            return False

        if self.prev is None:
            self.prev = (cx, cy)
        if self.anchor is None:
            self.anchor = (cx, cy)

        # 单帧速度
        speed = ((cx - self.prev[0]) ** 2 + (cy - self.prev[1]) ** 2) ** 0.5
        self.prev = (cx, cy)
        if speed >= MOTION_SPEED_PX:
            self.speed_hits += 1
        else:
            self.speed_hits = 0

        # 相对"收到指令那一刻"的位移
        self.last_dist = ((cx - self.anchor[0]) ** 2 +
                          (cy - self.anchor[1]) ** 2) ** 0.5

        # ---- 阶段 1：等它动起来 ----
        if self.phase == "WAIT_MOVE":
            if (self.last_dist >= MOTION_FROM_ANCHOR_PX
                    or self.speed_hits >= MOTION_SPEED_HITS):
                self.phase      = "WAIT_STILL"
                self.hist       = []
                self.speed_hits = 0
                self.anchor     = (cx, cy)     # 以"开始动"的位置为静止判定起点
            return False

        # ---- 阶段 3：已完成 ----
        if self.phase == "DONE":
            return True

        # ---- 阶段 2：等它重新停稳 ----
        self.hist.append((cx, cy, float(o.w), float(o.h)))
        if len(self.hist) > EDGE_STATIC_FRAMES:
            self.hist.pop(0)
        self.last_rng = (self._range(0), self._range(1),
                         self._range(2), self._range(3))
        if (len(self.hist) >= EDGE_STATIC_FRAMES
                and self.last_rng[0] <= EDGE_STATIC_RANGE_PX
                and self.last_rng[1] <= EDGE_STATIC_RANGE_PX
                and self.last_rng[2] <= EDGE_STATIC_SIZE_PX
                and self.last_rng[3] <= EDGE_STATIC_SIZE_PX):
            self.phase = "DONE"
            return True
        return False

# ---------------- 一维卡尔曼（移植自 K230 圆环版本） ----------------
class Kalman1D:
    def __init__(self, Q, R):
        self.Q = Q
        self.R = R
        self.x_est = 0.0
        self.p = 1.0
        self.inited = False

    def update(self, measure):
        # 第一次观测直接当初值，否则 x_est 从 0 慢慢爬，首次锁定时会看到"中心点乱飞"
        if not self.inited:
            self.inited = True
            self.x_est = float(measure)
            self.p = self.R
            return self.x_est
        p_pred = self.p + self.Q
        kg = p_pred / (p_pred + self.R)
        self.x_est = self.x_est + kg * (measure - self.x_est)
        self.p = (1.0 - kg) * p_pred
        return self.x_est

    def reset(self):
        self.x_est = 0.0
        self.p = 1.0
        self.inited = False


def _cir_get(cir, name, default=None):
    """霍夫圆对象的字段读取（兼容属性/方法两种写法）"""
    try:
        v = getattr(cir, name)
    except Exception:
        return default
    try:
        return v() if callable(v) else v
    except Exception:
        return default


def _in_any_box(cx, cy, boxes, offset):
    """圆心是否落在某个框（外扩 offset）内"""
    for bx, by, bw, bh in boxes:
        if (bx - offset) <= cx <= (bx + bw + offset) and \
           (by - offset) <= cy <= (by + bh + offset):
            return True
    return False


class RingCircleTracker:
    """霍夫圆圆环跟踪（把 K230 那份算法的核心思路移植过来）

    流程：ROI 搜索 → 候选过滤 → 同心圆聚类 → 关联选目标 → 圆心/半径平滑 + 卡尔曼
      · ROI：锁定后只在上一帧位置附近找（快、也不会锁到别处的圆）；
             没锁定时用 hint（two 的 YOLO 框）外扩一圈；都没有就全图
      · 聚类：圆心相距 < HOUGH_MERGE_DIST 的多个圆（同心内外环）归成一个目标，
              半径取"最接近上一帧"的那一环 → 画出来的圆不会忽大忽小
      · 关联：锁定后以"贴着上一帧"为主（带门限 + 半径变化限制），不会在圆之间来回跳
      · 抗抖：圆心/半径指数平滑 + 卡尔曼，短暂丢失还能沿用上次结果
    """

    def __init__(self):
        self.kf_x = Kalman1D(HOUGH_KF_Q, HOUGH_KF_R)
        self.kf_y = Kalman1D(HOUGH_KF_Q, HOUGH_KF_R)
        self.reset()

    def reset(self):
        self.valid  = False
        self.cx     = 0.0
        self.cy     = 0.0
        self.r      = 0.0
        self.lost   = 0
        self.hits   = 0
        self.tick   = 0
        self.use_roi = True
        self.warned  = False
        # 调试用：每一步的结果都记下来，没画出圆时一眼看出卡在哪
        self.info = {"state": "未开始", "n_cand": 0, "n_group": 0,
                     "roi": None, "rng": (0, 0), "pick": None}
        self.kf_x.reset()
        self.kf_y.reset()

    def debug_line(self):
        i = self.info
        return ("[HOUGH] %s | 候选%d 组%d | roi=%s r=%s pick=%s | lock=%d lost=%d"
                % (i.get("state"), i.get("n_cand", 0), i.get("n_group", 0),
                   i.get("roi"), i.get("rng"), i.get("pick"),
                   1 if self.valid else 0, self.lost))

    # ---------- 对外入口 ----------
    def find(self, img, hint=None, block_boxes=()):
        """hint = (x, y, w, h)，一般是 two 的 YOLO 框；返回 (cx, cy, r) 或 None"""
        if not ENABLE_HOUGH:
            return None

        # 节流：跳过的帧直接给上次结果
        self.tick += 1
        if HOUGH_EVERY_N > 1 and (self.tick % HOUGH_EVERY_N) != 0 and self.valid:
            return self._output()

        W, H = self._size(img)
        roi = self._search_roi(W, H, hint)
        r_min, r_max = self._r_bounds(hint)
        self.info["roi"] = roi
        self.info["rng"] = (r_min, r_max)

        circles, ok = self._find_circles(img, roi, hint)
        if not ok:
            self.info["state"] = "find_circles 不可用（见上面的警告）"
            return self._output() if self.valid else None

        cand = self._filter(circles, W, H, block_boxes)
        self.info["n_cand"] = len(cand)
        pick = None
        groups = []
        if cand:
            groups = self._cluster(cand)
            pick = self._select(groups, hint)
        self.info["n_group"] = len(groups)
        self.info["pick"] = None if pick is None else \
            (int(round(pick[0])), int(round(pick[1])), int(round(pick[2])))

        if pick is not None:
            self.info["state"] = "已锁定" if self.valid else "新锁定"
            if not self.valid:
                # 新目标：直接用测量值初始化，并复位卡尔曼
                self.valid = True
                self.hits  = 0
                self.cx, self.cy, self.r = pick[0], pick[1], pick[2]
                self.kf_x.reset()
                self.kf_y.reset()
            else:
                self.cx += HOUGH_XY_ALPHA * (pick[0] - self.cx)
                self.cy += HOUGH_XY_ALPHA * (pick[1] - self.cy)
                if self.r > 1.0:
                    self.r += HOUGH_R_ALPHA * (pick[2] - self.r)
                else:
                    self.r = pick[2]
            self.lost = 0
            self.hits += 1
            return self._output()

        # 本帧没找到：短暂丢失沿用上次结果（HOUGH_HOLD_FRAMES），超过就放弃
        self.info["state"] = "没找到圆（候选/分组见左）"
        self.lost += 1
        if self.lost > HOUGH_LOST_FRAMES:
            self.valid = False
            self.hits  = 0
        if self.valid and self.lost <= HOUGH_HOLD_FRAMES:
            return self._output()
        return None

    # ---------- 内部 ----------
    def _size(self, img):
        try:
            return int(img.width()), int(img.height())
        except Exception:
            return TARGET_X * 2, TARGET_Y * 2

    def _output(self):
        if not self.valid:
            return None
        x = self.kf_x.update(self.cx)
        y = self.kf_y.update(self.cy)
        return (int(round(x)), int(round(y)), int(round(self.r)))

    def _search_roi(self, W, H, hint):
        if self.use_roi and self.valid and self.lost == 0:
            cx, cy = self.cx, self.cy
            # ROI 至少要能把整个圆包住，否则圆被切掉一半就检不到了
            pad = max(HOUGH_TRACK_ROI_PAD, int(self.r * 1.6))
        elif hint is not None:
            cx = hint[0] + hint[2] * 0.5
            cy = hint[1] + hint[3] * 0.5
            # 搜索范围要足够把整个圆包住：two 框尺寸 + 外扩
            pad = HOUGH_ROI_PAD + int(max(hint[2], hint[3]))
        else:
            return None                     # 全图搜索
        x1 = max(0, int(cx - pad))
        y1 = max(0, int(cy - pad))
        x2 = min(W, int(cx + pad))
        y2 = min(H, int(cy + pad))
        if x2 - x1 < 8 or y2 - y1 < 8:
            return None
        return [x1, y1, x2 - x1, y2 - y1]

    def _r_bounds(self, hint):
        """半径范围：默认按 two 的框自适应，换分辨率也不用改"""
        if HOUGH_R_FROM_HINT and hint is not None:
            w, h = float(hint[2]), float(hint[3])
            lo = max(4, int(min(w, h) * HOUGH_R_HINT_MIN_F))
            hi = max(lo + 2, int(max(w, h) * HOUGH_R_HINT_MAX_F))
            return lo, hi
        return HOUGH_R_MIN, HOUGH_R_MAX

    def _find_circles(self, img, roi, hint):
        """返回 (circles, ok)；ok=False 表示这个固件根本调不动霍夫圆"""
        r_min, r_max = self._r_bounds(hint)
        kw = dict(threshold=HOUGH_THRESHOLD, r_min=r_min, r_max=r_max,
                  x_margin=HOUGH_MARGIN, y_margin=HOUGH_MARGIN,
                  r_margin=HOUGH_MARGIN, r_step=HOUGH_R_STEP)
        if roi is not None and self.use_roi:
            try:
                return list(img.find_circles(roi=roi, **kw)), True
            except TypeError:
                self.use_roi = False        # 固件不支持 roi → 以后都全图
            except Exception as e:
                if not self.warned:
                    self.warned = True
                    print("[HOUGH] find_circles(roi) 失败:", e)
        try:
            return list(img.find_circles(**kw)), True
        except TypeError as e1:
            try:
                # 参数名不同的老固件：按位置传参 (roi, x_stride, y_stride, threshold, ...)
                return list(img.find_circles([0, 0, self._size(img)[0], self._size(img)[1]],
                                             2, 1, HOUGH_THRESHOLD,
                                             HOUGH_MARGIN, HOUGH_MARGIN, HOUGH_MARGIN,
                                             r_min, r_max, HOUGH_R_STEP)), True
            except Exception as e2:
                if not self.warned:
                    self.warned = True
                    print("[HOUGH] 霍夫圆不可用，改用 YOLO 框中心:", e1, "|", e2)
                return None, False
        except Exception as e:
            if not self.warned:
                self.warned = True
                print("[HOUGH] 霍夫圆不可用，改用 YOLO 框中心:", e)
            return None, False

    def _filter(self, circles, W, H, block_boxes):
        """候选过滤：半径范围、被画面边缘截断、圆心落在颜色物块框里"""
        if not circles:
            return []
        cand = []
        for c in circles:
            cx = _cir_get(c, "x")
            cy = _cir_get(c, "y")
            cr = _cir_get(c, "r")
            if cx is None or cy is None or cr is None:
                continue
            cx = float(cx); cy = float(cy); cr = float(cr)
            if cx < HOUGH_EDGE_MARGIN or cx > W - HOUGH_EDGE_MARGIN:
                continue
            if cy < HOUGH_EDGE_MARGIN or cy > H - HOUGH_EDGE_MARGIN:
                continue
            if (cy + cr) > H - 2:                       # 底部夹爪/画面外
                continue
            if _in_any_box(cx, cy, block_boxes, HOUGH_BLOCK_OFFSET):
                continue                                # 圆心在色块上，不是圆环
            mag = _cir_get(c, "magnitude", 0) or 0
            cand.append((cx, cy, cr, float(mag)))
        return cand

    def _cluster(self, cand):
        """同心圆归并：圆心相近的多个圆算同一个目标（解决内外环互相抢）"""
        md2 = HOUGH_MERGE_DIST * HOUGH_MERGE_DIST
        groups = []
        for cx, cy, cr, mag in cand:
            for g in groups:
                gx = g["sx"] / g["n"]
                gy = g["sy"] / g["n"]
                if (cx - gx) ** 2 + (cy - gy) ** 2 <= md2:
                    g["sx"] += cx
                    g["sy"] += cy
                    g["n"]  += 1
                    g["m"].append((cx, cy, cr, mag))
                    break
            else:
                groups.append({"sx": cx, "sy": cy, "n": 1,
                               "m": [(cx, cy, cr, mag)]})

        out = []
        for g in groups:
            ms = g["m"]
            if self.valid and self.r > 1.0:
                # 半径取最接近上一帧的那一环，防止忽大忽小
                r_rep = min(ms, key=lambda m: abs(m[2] - self.r))[2]
            else:
                mags = [m[3] for m in ms]
                if mags and max(mags) > 0.0:
                    r_rep = max(ms, key=lambda m: m[3])[2]      # 响应最强的环
                else:
                    rs = sorted(m[2] for m in ms)
                    r_rep = rs[len(rs) // 2]                   # 退化取中位
            out.append({"cx": g["sx"] / g["n"], "cy": g["sy"] / g["n"],
                        "r": r_rep, "n": g["n"]})
        return out

    def _select(self, groups, hint):
        """选目标：已锁定 → 贴着上一帧；未锁定 → 离 hint 中心最近；同心环多者加权"""
        locked = self.valid and self.hits >= HOUGH_CONFIRM_FRAMES
        gate2 = HOUGH_TRACK_GATE * HOUGH_TRACK_GATE
        if hint is not None:
            hx = hint[0] + hint[2] * 0.5
            hy = hint[1] + hint[3] * 0.5
        else:
            hx, hy = TARGET_X, TARGET_Y

        best, best_score = None, None
        for g in groups:
            if locked:
                dx = g["cx"] - self.cx
                dy = g["cy"] - self.cy
                d2 = dx * dx + dy * dy
                if d2 > gate2:
                    continue                                  # 超出关联门限
                if self.r > 1.0:
                    ratio = g["r"] / self.r
                    if ratio > HOUGH_R_RATIO_MAX or ratio < 1.0 / HOUGH_R_RATIO_MAX:
                        continue                              # 半径突变，判为别的圆
                score = d2
            else:
                score = (g["cx"] - hx) ** 2 + (g["cy"] - hy) ** 2
            score *= HOUGH_VOTE_BONUS ** (g["n"] - 1)          # 同心环多 → 更可信
            if best_score is None or score < best_score:
                best_score, best = score, g
        if best is None:
            return None
        return (best["cx"], best["cy"], best["r"])

class BoxSlot:
    """一个固定框的状态机，输出 R(就绪)/E(空)/!(有但不合格)"""
    def __init__(self):
        self.reset()
    def reset(self):
        self.seen_empty = False
        self.has        = False
        self.ok         = False
        self.code       = 0
        self.latch      = 0
        self.cx         = -1
        self.cy         = -1
        self.cnt        = 0
    def update(self, det):
        if det is None:
            self.seen_empty = True
            self.has   = False
            self.cnt   = 0
            self.cx    = -1
            self.cy    = -1
            self.code  = 0
            if BOX_READY_MODE == "static":
                self.ok = False
            return self.ok
        code, cx, cy = det
        self.code = code
        self.has  = True
        if BOX_READY_MODE == "empty_to_full":
            # 设计书：等待到框内从无到有
            if self.seen_empty and not self.ok:
                self.ok = True
            if self.ok and self.latch == 0:
                self.latch = code
            return self.ok
        if self.cx == -1:
            self.cx, self.cy = cx, cy
            self.cnt  = 0
            self.ok   = False
        else:
            dist = ((cx - self.cx) ** 2 + (cy - self.cy) ** 2) ** 0.5
            self.cnt = self.cnt + 1 if dist < ROI_MOVE_THRESH else 0
            self.cx, self.cy = cx, cy
            self.ok = self.cnt >= ROI_STATIC_FRAMES
        if self.ok and self.latch == 0:
            self.latch = code
        return self.ok
    def report_code(self):
        """返回发给电控的颜色编码（0 = 没有）"""
        return self.latch if self.ok else self.code
    def release(self):
        """0x03 用：清掉颜色锁存和就绪位，但保留 seen_empty

        这样框里已经有物块时，下一帧会立刻重新读一次颜色（不会因为要求
        "从无到有"而永远锁不上），只是把之前锁的旧颜色丢掉重来。
        """
        self.ok    = False
        self.latch = 0
        self.cnt   = 0
        self.cx    = -1
        self.cy    = -1
    def state_char(self):
        if self.ok:
            return "R"
        if not self.has:
            return "E"
        return "!"

# ==============================================================================
# 七之二、镜头畸变校正 + 直线拟合 + 圆环连线校准（0x06）
# ==============================================================================
def undistort_point(px, py):
    """把畸变图像上的点校正成理想针孔图像上的点（Brown-Conrady，迭代反解）

    参数全 0 时原样返回（等于没开）。弱畸变 5 次迭代足够收敛。
    """
    if (not DIST_ENABLE) or (DIST_K1 == 0.0 and DIST_K2 == 0.0
                             and DIST_P1 == 0.0 and DIST_P2 == 0.0):
        return px, py
    cxp = DIST_CX if DIST_CX else TARGET_X
    cyp = DIST_CY if DIST_CY else TARGET_Y
    fx  = DIST_FX if DIST_FX else float(max(1, TARGET_X * 2))
    fy  = DIST_FY if DIST_FY else float(max(1, TARGET_Y * 2))
    xd = (px - cxp) / fx
    yd = (py - cyp) / fy
    x, y = xd, yd
    for _ in range(5):
        r2 = x * x + y * y
        rad = 1.0 + DIST_K1 * r2 + DIST_K2 * r2 * r2
        if abs(rad) < 1e-6:
            break
        dx = 2.0 * DIST_P1 * x * y + DIST_P2 * (r2 + 2.0 * x * x)
        dy = DIST_P1 * (r2 + 2.0 * y * y) + 2.0 * DIST_P2 * x * y
        x = (xd - dx) / rad
        y = (yd - dy) / rad
    return x * fx + cxp, y * fy + cyp


def fit_line(points):
    """最小二乘直线拟合（PCA）

    points: [(x, y), ...]，≥2 个点（2 个点就是直接连线）
    返回 (cx, cy, dx, dy) —— 直线的中点 + 单位方向向量；点不够返回 None
    """
    n = len(points)
    if n < 2:
        return None
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    if n == 2:
        dx = points[1][0] - points[0][0]
        dy = points[1][1] - points[0][1]
    else:
        sxx = sum((p[0] - mx) ** 2 for p in points)
        sxy = sum((p[0] - mx) * (p[1] - my) for p in points)
        syy = sum((p[1] - my) ** 2 for p in points)
        # 主成分方向（最大特征值对应的特征向量）
        theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
        dx, dy = math.cos(theta), math.sin(theta)
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return None
    return (mx, my, dx / norm, dy / norm)


class RingLineCalib:
    """用圆环中心给车头方向做绝对校准（替代会漂的陀螺仪）

    两种方法（LINE_METHOD）：
      "bearing"（默认，推荐）：
          one/two 两个中心 → 各自畸变校正 → 指数平滑
          → 换算成"相对光轴的水平方位角" b = atan((x - cx) / f)
          → 取平均 b̄ = (b1 + b2) / 2
          → 偏差角 = b̄ - 参考角
          物理意义：b̄ 就是"这对圆环在画面里的方位"。纯偏航时所有方位角同时转 δ，
          b̄ 也正好转 δ，所以 b̄ 的变化量 = 车头转向量（与车停在哪儿关系极小）。
          这也是唯一能在两点情况下稳定测出"转角"的量。
      "segment"（旧做法）：≥2 个点拟合直线，取直线角度 - 参考角。
          直线角度是"两点位置之差"的方向，会被安装俯仰/透视/横向位置污染，精度差。

    参考角：0x07 现场记录（车摆正那一刻的值），或配置 LINE_CAR_FORWARD_DEG
    点不够时沿用上次结果最多 LINE_HOLD_FRAMES 帧
    """

    def __init__(self):
        self.reset()

    def reset(self):
        """完全复位（含 0x07 记录的参考角）"""
        self.ref_deg = None
        self.last_line_deg = None
        self.reset_filter()

    def reset_filter(self):
        """只清滤波/几何状态，保留 0x07 现场标定的参考角"""
        self.lps   = {}                                     # 圆环编号 → 中心平滑器
        self.ang_sm = AngleSmoother(LINE_ANGLE_ALPHA, 90.0)  # 角度平滑
        self.off_est = None                                  # segment 法的横向偏差平滑值
        self.last  = None        # (偏差角deg, 基线px, 点数, x1, y1, x2, y2)
        self.last_base = 0.0     # 最近一次两点在画面上的间距（px）
        self.last_mid  = None    # 最近一次两点中点（px）
        self.hold  = 0

    def set_reference(self):
        """把当前测量值记为"车头正对时"的参考（0x07 现场标定）"""
        if self.last_line_deg is None:
            print("[LINE] 参考角记录失败：当前没有有效测量（确保画面里 two 个圆环都在）")
            return False
        self.ref_deg = self.last_line_deg
        print("[LINE] 已把当前值记为参考: %.2f°（方法=%s；也可直接写进 LINE_CAR_FORWARD_DEG）"
              % (self.ref_deg, LINE_METHOD))
        return True

    def reference(self):
        return self.ref_deg if self.ref_deg is not None else LINE_CAR_FORWARD_DEG

    def debug_line(self):
        if self.last is None:
            return ("[LINE] 无有效测量（方法=%s，需要画面里 %s 同时在线）"
                    % (LINE_METHOD, list(LINE_TARGET_NUMS)))
        a, base, n, x1, y1, x2, y2 = self.last
        return ("[LINE] 方法=%s 偏差=%.1f° 基线=%.0fpx(门限%d) 参考=%.2f°(%s) "
                "端点=(%d,%d)-(%d,%d)"
                % (LINE_METHOD, a, base, LINE_MIN_BASELINE_PX, self.reference(),
                   "0x07已标定" if self.ref_deg is not None else "配置值",
                   x1, y1, x2, y2))

    # ---------- 取点（畸变校正 + 平滑） ----------
    def _collect(self, objs):
        pts = []
        for num in LINE_TARGET_NUMS:
            cid = RING_CLASS_BY_NUM.get(num)
            if cid is None:
                continue
            o = pick_class(objs, cid)
            if o is None:
                continue
            cx, cy = obj_center(o)
            # two 可以选用霍夫圆心（更准），但霍夫误检时会反而更差，默认关
            if LINE_USE_HOUGH_FOR_TWO and num == 2 and DBG["hough"] is not None:
                cx, cy = DBG["hough"][0], DBG["hough"][1]
            ux, uy = undistort_point(cx, cy)          # ★ 先做镜头畸变校正
            lp = self.lps.get(num)
            if lp is None:
                lp = Smoother(LINE_POINT_ALPHA)
                self.lps[num] = lp
            sx, sy = lp.update(ux, uy)                # ★ 再逐点平滑
            pts.append((sx, sy))
        return pts

    # ---------- 主入口 ----------
    def update(self, objs):
        """返回 (偏差角deg, 基线px, 点数, x1, y1, x2, y2) 或 None"""
        if not ENABLE_LINE_CALIB:
            return None

        pts = self._collect(objs)
        if len(pts) < 2:
            self.hold += 1
            if self.hold > LINE_HOLD_FRAMES:
                self.last = None
                self.ang_sm.reset()
                self.off_est = None
            return self.last

        self.hold = 0
        if LINE_METHOD == "segment":
            return self._update_segment(pts)
        return self._update_bearing(pts)

    # ---------- 方法 A：两点方位角平均（推荐） ----------
    def _update_bearing(self, pts):
        (x1, y1), (x2, y2) = pts[0], pts[1]
        f = LINE_FOCAL_PX if LINE_FOCAL_PX else float(max(1, TARGET_X * 2))

        # 两点在画面上的间距：太小说明太远/看不清，角分辨率不够 → 不更新
        base = math.hypot(x1 - x2, y1 - y2)
        self.last_base = base
        self.last_mid  = ((x1 + x2) * 0.5, (y1 + y2) * 0.5)
        if base < LINE_MIN_BASELINE_PX:
            return self.last

        # 水平方位角（相对光轴，右正左负）
        b1 = math.degrees(math.atan2(x1 - TARGET_X, f))
        b2 = math.degrees(math.atan2(x2 - TARGET_X, f))
        b_mean = 0.5 * (b1 + b2)

        self.last_line_deg = b_mean           # 0x07 记录的就是这个值
        err_raw = wrap_deg(b_mean - self.reference(), 90.0)
        if abs(err_raw) > LINE_MAX_ANGLE_DEG:
            return self.last

        ang = self.ang_sm.update(err_raw)
        self.last = (LINE_ANGLE_SIGN * ang, base, 2,
                     int(x1), int(y1), int(x2), int(y2))
        return self.last

    # ---------- 方法 B：多点拟合直线（旧做法） ----------
    def _update_segment(self, pts):
        if len(pts) < LINE_MIN_POINTS:
            return self.last
        fit = fit_line(pts)
        if fit is None:
            return self.last
        cx, cy, dx, dy = fit
        # ★ 统一方向朝下（dy>0；水平线则朝右）：否则同一根线的垂直偏差会在 ± 之间翻，
        #   滤波值就被来回拉，输出一顿一顿
        if dy < -1e-9 or (abs(dy) <= 1e-9 and dx < 0):
            dx, dy = -dx, -dy

        # 直线方向 → 角度（-90~90，直线无方向性）
        line_deg = math.degrees(math.atan2(dy, dx))
        self.last_line_deg = line_deg                  # 0x07 现场标定要用
        ang_raw  = wrap_deg(line_deg - self.reference(), 90.0)
        if abs(ang_raw) > LINE_MAX_ANGLE_DEG:       # 夹角太大 → 不可信
            return self.last

        # 直线到画面中心的垂直偏差（竖直方向时：正 = 直线在中心右侧）
        off_raw = (cx - TARGET_X) * dy - (cy - TARGET_Y) * dx

        ang = self.ang_sm.update(ang_raw)
        if self.off_est is None:
            self.off_est = off_raw
        else:
            self.off_est += LINE_OFFSET_ALPHA * (off_raw - self.off_est)

        # 画线端点：以拟合中心为中心，沿方向向两端各延伸 LINE_DRAW_LEN
        ln = LINE_DRAW_LEN
        x1, y1 = int(cx - dx * ln), int(cy - dy * ln)
        x2, y2 = int(cx + dx * ln), int(cy + dy * ln)
        self.last_base = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
        self.last_mid  = (cx, cy)
        self.last = (LINE_ANGLE_SIGN * ang, self.last_base, len(pts), x1, y1, x2, y2)
        return self.last

lp_near  = Smoother(LOWPASS_ALPHA)
lp_ring  = Smoother(RING_LOWPASS_ALPHA)      # 圆环比全局更稳
lp_color = Smoother(LOWPASS_ALPHA)
grace_near  = LostGrace()
grace_ring  = LostGrace(RING_HOLD_FRAMES)    # 圆环丢检保持，治"时有时无"
grace_color = LostGrace()
motion_event = MotionEvent()
hough       = RingCircleTracker()
line_calib  = RingLineCalib()
box_slots = [BoxSlot() for _ in range(3)]

def hough_update(img, objs):
    """只要画面里检到 two，就跑一次霍夫圆（与当前模式无关，方便调试/画图）

    返回 (cx, cy, r) 或 None；结果同时存进 DBG["hough"] 供绘制使用，
    0x04 模式直接复用这个结果，不会重复跑。
    """
    if not ENABLE_HOUGH:
        if hough.valid:
            hough.reset()
        return None
    cid = RING_CLASS_BY_NUM.get(2)                 # two 的 class_id
    two = pick_class(objs, cid) if cid is not None else None
    if two is None:
        hough.info["state"] = "没有 two"
        if hough.valid:
            hough.reset()
        return None
    blocks = [(b.x, b.y, b.w, b.h) for b in objs
              if b.class_id in BLOCK_CLASS_IDS]    # 圆心落在色块上的圆要滤掉
    return hough.find(img, hint=(two.x, two.y, two.w, two.h), block_boxes=blocks)

def reset_tracking():
    """收到任何指令都调用一次：清空"动/静"判定 + 跟踪滤波 + 霍夫圆状态，重新检测

    只清"判定/滤波"这类记忆，不动 box_filter（避免画面上所有框闪一下）。
    """
    clear_flag()
    motion_event.reset()       # 动/静事件回到 WAIT_MOVE，重新等"动了又停稳"
    line_calib.reset_filter()  # 连线校准：清点/角度/偏移的平滑（保留 0x07 参考角）
    lp_near.reset()
    lp_ring.reset()
    lp_color.reset()
    grace_near.reset()
    grace_ring.reset()
    grace_color.reset()
    hough.reset()

def reset_all():
    global last_near, last_ring, last_color, ring_num_last
    last_near  = (NO_TARGET_DXDY, NO_TARGET_DXDY)
    last_ring  = (NO_TARGET_DXDY, NO_TARGET_DXDY)
    last_color = (NO_TARGET_DXDY, NO_TARGET_DXDY)
    ring_num_last = 0
    reset_tracking()
    box_filter.reset()
    for s in box_slots:
        s.reset()

def slots_update(objs):
    for i, roi in enumerate(ROI_BOX):
        o = pick_in_roi(objs, roi, BOX_IGNORE_IDS)
        det = None
        if o is not None:
            cx, cy = obj_center(o)
            det = (out_code(o.class_id), cx, cy)       # ★ 转成发给电控的编码
            DBG["box_mark"][i] = (o.x, o.y, o.w, o.h)
        box_slots[i].update(det)
        DBG["box_char"][i]  = box_slots[i].state_char()
        DBG["box_code"][i]  = box_slots[i].report_code()

def _known_box_codes():
    """收集"算作已认到"的框颜色：{框下标: 颜色编码}

    BOX_DEDUCE_NEED_READY=True 时只把已就绪(R)的框算作已知，避免瞬时误检把排除法带偏
    """
    known = {}
    for i, s in enumerate(box_slots):
        c = s.report_code()
        if c == 0:
            continue
        if BOX_DEDUCE_NEED_READY and not s.ok:
            continue
        known[i] = c
    return known

def deduce_box_codes(codes):
    """0x03 的排除法：已认到 2 个框的颜色时，用 0x01 任务码推出剩下那个框的颜色

    codes: [c1, c2, c3]（左上 / 下 / 右上），原地修改并返回
    返回被补出来的框下标（0/1/2），没有补则返回 -1
    """
    if not BOX_DEDUCE_BY_TASK:
        return -1
    known = _known_box_codes()
    if len(known) != 2:                 # 只有"已知 2 个、缺 1 个"时才推
        return -1
    task = [c for c in task_code if c != 0]
    if len(task) != 3:                  # 没收到过 0x01 任务码（或不足 3 个）→ 不猜
        return -1
    rest = list(task)
    for c in known.values():
        if c in rest:
            rest.remove(c)
        else:
            return -1                   # 认到的颜色不在任务码里 → 数据可疑，不猜
    if len(rest) != 1:
        return -1
    for i in range(3):
        if i not in known:
            codes[i] = rest[0]
            return i
    return -1

def build_boxes_reply(f):
    codes = [s.report_code() for s in box_slots]
    ded = deduce_box_codes(codes)                  # ★ 排除法补第三个颜色
    DBG["box_deduce"] = ded
    DBG["box_code"]   = list(codes)                # 屏幕显示用实际要发的值
    return (CMD_BOXES, bytes([f & 0xFF, codes[0] & 0xFF,
                              codes[1] & 0xFF, codes[2] & 0xFF]))

DBG = {
    "mode":        MODE_IDLE,
    "flag":        0,
    "fps":         0.0,
    "box_char":    ["E", "E", "E"],
    "box_code":    [0, 0, 0],
    "box_mark":    [None, None, None],
    "box_deduce":  -1,        # 这次 0x03 里被"任务码排除法"补出来的框下标（-1 = 没补）
    "target_rect": None,
    "target_txt":  "",
    "hough":       None,
    "line":        None,      # (x1, y1, x2, y2) 0x06 拟合出来的直线，仅用于显示
}
def dbg_reset_frame():
    DBG["box_mark"]    = [None, None, None]
    DBG["box_deduce"]  = -1
    DBG["target_rect"] = None
    DBG["target_txt"]  = ""
    DBG["hough"]       = None
    DBG["line"]        = None

# ==============================================================================
# 八、每帧处理：每个 step 都必定返回一帧（与有没有检出目标无关）
# ==============================================================================
def step_idle(img, objs, now):
    # 设计书：0x00 空闲回 flag=1
    set_flag(1)
    DBG["flag"] = flag
    return (CMD_IDLE, bytes([flag]))

def step_task(img, objs, now):
    # 设计书：0x01 回 flag=1 表示任务码接收成功
    set_flag(1)
    DBG["flag"] = flag
    return (CMD_TASK, bytes([flag]))

def step_near(img, objs, now):
    """0x02：远离/靠近判断——离十字准心最近的【颜色物块】的位置偏差 + "动了又停稳"事件

    回帧：[AA 02 flag dxH dxL dyH dyL BB]
        flag = 该物块的颜色编码(1~6)，只在「动了 → 又停稳」之后才填：
               WAIT_MOVE  收到指令时它是静止的 → flag=0（电控别动）
               WAIT_STILL 已经动了 → 等它停稳
               DONE       停稳了 → flag=颜色码（锁存到下一條指令/换颜色）
        dx/dy = 物块中心相对画面中心的偏差（一直在发）
                     （没有物块时 flag=0 / dx=dy=0）
    """
    global flag, last_near
    o = pick_nearest(objs, BLOCK_CLASS_IDS)
    if o is None:
        if grace_near.hit() and last_near != (NO_TARGET_DXDY, NO_TARGET_DXDY):
            # 还在丢帧宽限内（LOST_GRACE_FRAMES>0 才会有这一支）：保持上次的 flag/偏差
            DBG["flag"] = flag
            return reply_dxdy(CMD_NEAR, flag, last_near[0], last_near[1])
        lp_near.reset()
        # ★ 注意：这里【不重置动/静状态机】，物块短暂丢失/重新检到时能接着原来的阶段，
        #   不会出现"发几个非 0 又变回 00"的抖动
        flag = 0
        DBG["flag"] = 0
        if KEEP_LAST_DXDY_WHEN_LOST and last_near != (NO_TARGET_DXDY, NO_TARGET_DXDY):
            return reply_dxdy(CMD_NEAR, 0, last_near[0], last_near[1])
        return reply_dxdy(CMD_NEAR, 0, NO_TARGET_DXDY, NO_TARGET_DXDY)

    grace_near.clear()
    cx, cy = obj_center(o)
    pos = lp_near.update(cx, cy)
    dx, dy = int(pos[0] - AIM_X), int(pos[1] - AIM_Y)      # ★ 相对瞄准点（含夹爪偏置）
    last_near = (dx, dy)

    code   = out_code(o.class_id)              # 颜色编码 1~6
    if ENABLE_EDGE_STATIC:
        raw = find_same_det(o, RAW_OBJS)       # 同一个目标的原始框（未滤波）
        if raw is not None:
            done = motion_event.update(raw)    # 动了→又停稳 事件检测
        else:
            done = (motion_event.phase == "DONE")   # 本帧没匹配到原始框：沿用阶段
    else:
        done = True                            # 关掉事件检测 → 只要跟踪到就报颜色码
    flag = code if done else 0                 # ★ 只有"动了又停稳"才填颜色码
    DBG["target_rect"] = (o.x, o.y, o.w, o.h)
    DBG["target_txt"]  = "NEAR %s(%d) %s" % (name_of(o.class_id), code,
                                             motion_event.phase if ENABLE_EDGE_STATIC else "IMMEDIATE")
    DBG["flag"] = flag
    return reply_dxdy(CMD_NEAR, flag, dx, dy)

def step_boxes(img, objs, now):
    global box_eval_ms, box_n_known, box_state
    ok_cnt = sum(1 for s in box_slots if s.ok)

    # ★ 每 BOX_EVAL_PERIOD_MS 重新统计一次"认到几个框"，据此决定怎么发
    if now - box_eval_ms >= BOX_EVAL_PERIOD_MS:
        box_eval_ms = now
        known = _known_box_codes()
        box_n_known = len(known)
        if box_n_known >= 3:
            box_state = "FULL"        # 三个都认到 → 原样发
        elif box_n_known == 2:
            box_state = "DEDUCE"      # 两个 → 用任务码排除法补第三个
        else:
            # 0 或 1 个：清掉三个框里的颜色锁存，继续下一轮识别
            # （release 保留 seen_empty，所以框里已经有物块时下一帧会重新读颜色）
            for s in box_slots:
                s.release()
            box_state = "RETRY"
            ok_cnt = 0
            DBG["box_char"] = ["E", "E", "E"]
            DBG["box_code"] = [0, 0, 0]

    # ★ 三个框里 ≥ BOX_READY_MIN 个就绪 → flag=1
    #   （不足 3 个也没关系：build_boxes_reply 会用 0x01 任务码把缺的那个颜色推出来）
    ready = ok_cnt >= BOX_READY_MIN
    set_flag(1 if ready else 0)
    DBG["flag"] = flag
    return build_boxes_reply(flag)

def step_ring(img, objs, now):
    """0x04：请求圆环 —— ★只返回 two 的坐标

    回帧： [AA 04 flag dxH dxL dyH dyL BB]
        flag = 2（two 的编号）；0 = 画面里没有 two（此时 dx=dy=0）
        dx/dy = two 中心相对画面中心的偏差（优先用霍夫圆心，没有就用 YOLO 框中心）
    想恢复"见谁发谁"或加上 three：改 RING_TARGET_NUMS，例如 (1, 2, 3)
    """
    global flag, last_ring, ring_num_last
    target = None
    # ★ 只认 RING_TARGET_NUMS 里的编号（默认只 two）；其它圆环一律不返回
    for num in RING_TARGET_NUMS:
        o = pick_class(objs, RING_CLASS_BY_NUM.get(num))
        if o is not None:
            target = (num, o)
            break
    if target is None:
        if grace_ring.hit() and ring_num_last:
            # 还在丢帧保持窗口内（RING_HOLD_FRAMES 帧）：继续发上次的编号和偏差
            flag = ring_num_last
            DBG["flag"] = flag
            return reply_dxdy(CMD_RING, flag, last_ring[0], last_ring[1])
        lp_ring.reset()
        # ★ 环从视野消失：flag 回 0（flag 就是圆环编号，0 = 没看到环）
        ring_num_last = 0
        flag = 0
        DBG["flag"] = 0
        if KEEP_LAST_DXDY_WHEN_LOST and last_ring != (NO_TARGET_DXDY, NO_TARGET_DXDY):
            return reply_dxdy(CMD_RING, 0, last_ring[0], last_ring[1])
        return reply_dxdy(CMD_RING, 0, NO_TARGET_DXDY, NO_TARGET_DXDY)
    grace_ring.clear()
    num, o = target

    # ★ 霍夫圆结果已经在主循环里算好了（hough_update，每帧只要画面有 two 就跑）
    hc = DBG["hough"] if num in HOUGH_FOR_RING_NUMS else None
    if hc is not None:
        cx, cy = hc[0], hc[1]                 # 霍夫圆心（内部已聚类+平滑+卡尔曼）
        pos = (float(cx), float(cy))
    else:
        cx, cy = obj_center(o)                # 没找到圆 → 退回 YOLO 框中心
        pos = lp_ring.update(cx, cy)
    dx, dy = int(pos[0] - AIM_X), int(pos[1] - AIM_Y)      # ★ 相对瞄准点（含夹爪偏置）
    last_ring = (dx, dy)
    ring_num_last = num
    flag = num & 0xFF
    DBG["target_rect"] = (o.x, o.y, o.w, o.h)
    DBG["target_txt"]  = "RING %d (%s)%s" % (num, name_of(o.class_id),
                                             " +HOUGH" if hc is not None else "")
    DBG["flag"] = flag
    return reply_dxdy(CMD_RING, flag, dx, dy)

def step_color(img, objs, now):
    global last_color
    # 电控发来的是"编码"(1~6)，反查成模型 class_id 再找
    o = pick_class(objs, CLASS_BY_CODE.get(target_color))
    if o is None:
        if grace_color.hit() and last_color != (NO_TARGET_DXDY, NO_TARGET_DXDY):
            # 还在丢帧宽限内（LOST_GRACE_FRAMES>0 才会有这一支）
            set_flag(1)
            DBG["flag"] = flag
            return reply_dxdy(CMD_COLOR, flag, last_color[0], last_color[1])
        lp_color.reset()
        set_flag(0)
        DBG["flag"] = 0
        # ★ 目标从视野消失：flag 一定是 0，不再"沿用消失前的数据"
        if KEEP_LAST_DXDY_WHEN_LOST and last_color != (NO_TARGET_DXDY, NO_TARGET_DXDY):
            return reply_dxdy(CMD_COLOR, 0, last_color[0], last_color[1])
        return reply_dxdy(CMD_COLOR, 0, NO_TARGET_DXDY, NO_TARGET_DXDY)
    grace_color.clear()
    cx, cy = obj_center(o)
    pos = lp_color.update(cx, cy)
    dx, dy = int(pos[0] - AIM_X), int(pos[1] - AIM_Y)      # ★ 相对瞄准点（含夹爪偏置）
    last_color = (dx, dy)
    set_flag(1)
    DBG["target_rect"] = (o.x, o.y, o.w, o.h)
    DBG["target_txt"]  = "COLOR %d %s" % (target_color, name_of(o.class_id))
    DBG["flag"] = flag
    return reply_dxdy(CMD_COLOR, flag, dx, dy)

def step_line(img, objs, now):
    """0x06：请求修正角度（用 one/two 两个圆环校准车头方向）

    回帧（按《视觉-电控通信》新协议）：[AA 06 flag AngleH AngleL BB]
        flag  = 1 已经识别到有效测量；0 = 没识别到/不可信（此时 Angle=0）
        Angle = 车头偏差角，int16 大端
                LINE_TENTH_DEG=True 时单位 0.1°（例如 -3.5° → -35 → FF DD）
                正 = 这对圆环在画面里偏右 = 车头偏左（电控应向右修）
                符号反了就把 LINE_ANGLE_SIGN 改成 -1
    横向偏差/基线不再发（协议里没这两个字段），只画屏幕 + 打印
    """
    global flag, line_capture_pending
    res = line_calib.update(objs)

    # 0x07（可选）：拿到一帧有效连线后，把它的角度记成参考（车此刻是"正对"的）
    captured = False
    if line_capture_pending and res is not None:
        if line_calib.set_reference():
            line_capture_pending = False
            captured = True

    if res is None:
        flag = 0
        ang_i = 0
    else:
        ang, base, n = res[0], res[1], res[2]
        if captured:
            ang = 0.0                 # 刚标定完，偏差按 0 回（给电控一个确认）
        flag = 1                      # ★ 协议：flag=1 表示已经识别到，可以对角度做 PID 修正
        scale = 10.0 if LINE_TENTH_DEG else 1.0
        ang_i = max(-32768, min(32767, int(round(ang * scale))))
        if LINE_DRAW:
            DBG["line"] = (res[3], res[4], res[5], res[6])
        DBG["target_txt"] = "ANGLE %.1f° (基线%.0fpx)%s" % (
            ang, base, " [已标定]" if captured else "")

    a1, a2 = pack_i16(ang_i)
    DBG["flag"] = flag
    return (CMD_LINE, bytes([flag & 0xFF, a1, a2]))

# ==============================================================================
# 九、命令处理（任何指令都先清一次跟踪状态，再切模式）
# ==============================================================================
def on_cmd(func, payload, now):
    global mode, target_color, task_code, reply_at_ms, ring_num_last
    global last_near, last_ring, last_color
    global last_cmd_func, last_cmd_ms
    global line_capture_pending
    if func not in CMD_LEN:
        return

    # ★ 收到指令后"重新开始判动静"：
    #   默认只在上一条同类指令已过去 REARM_MIN_GAP_MS 以上时才清空重判，
    #   这样电控反复发同一条指令不会把 8 帧判定窗口一直清零。
    repeat = (func == last_cmd_func) and ((now - last_cmd_ms) < REARM_MIN_GAP_MS)
    if REARM_ALWAYS or not repeat:
        reset_tracking()          # 清 动/静判定 + 平滑器 + 丢帧宽限 + 霍夫圆
    last_cmd_func = func
    last_cmd_ms   = now

    reply_at_ms = None
    if func == CMD_IDLE:
        mode = MODE_IDLE
        reset_all()
        return
    if func == CMD_TASK:
        if len(payload) >= 3:
            task_code = (payload[0], payload[1], payload[2])
        mode = MODE_TASK
        return
    if func == CMD_NEAR:
        last_near = (NO_TARGET_DXDY, NO_TARGET_DXDY)
        mode = MODE_NEAR
        return
    if func == CMD_BOXES:
        for s in box_slots:
            s.reset()
        mode = MODE_BOXES
        return
    if func == CMD_RING:
        last_ring = (NO_TARGET_DXDY, NO_TARGET_DXDY)
        ring_num_last = 0
        mode = MODE_RING
        return
    if func == CMD_COLOR:
        if len(payload) >= 1:
            target_color = payload[0]
        last_color = (NO_TARGET_DXDY, NO_TARGET_DXDY)
        mode = MODE_COLOR
        return
    if func == CMD_LINE:
        line_calib.reset_filter()
        mode = MODE_LINE
        return
    if func == CMD_LREF:
        # 现场标定：下一帧拿到有效连线就把此刻的角度记成参考（车此时是"正对"的）
        line_capture_pending = True
        mode = MODE_LINE
        return

# ==============================================================================
# 十、调试绘制
# ==============================================================================
SLOT_COLOR = {
    "E": image.COLOR_RED,
    "R": image.COLOR_GREEN,
    "!": image.COLOR_PURPLE,
}

def _dbg_text(img, x, y, s, color, scale=1):
    try:
        img.draw_string(int(x), int(y), s, color, scale, 2)
    except Exception:
        pass

def _dbg_rect(img, x, y, w, h, color, thickness=1):
    try:
        img.draw_rect(int(x), int(y), int(w), int(h), color, thickness=thickness)
    except Exception:
        pass

def _dbg_mark(img, cx, cy, color, half=4):
    _dbg_rect(img, cx - half, cy - half, half * 2, half * 2, color, -1)

def _dbg_cross(img, cx, cy, color, size=CROSSHAIR_SIZE):
    """十字准心（用两条细实心矩形画，兼容性好）"""
    cx, cy = int(cx), int(cy)
    size = int(size)
    _dbg_rect(img, cx - size, cy, size * 2 + 1, 1, color, -1)   # 横
    _dbg_rect(img, cx, cy - size, 1, size * 2 + 1, color, -1)   # 竖

def _dbg_circle(img, cx, cy, r, color, thickness=2):
    """画圆；固件没有 draw_circle 就退化成外接方框"""
    try:
        img.draw_circle(int(cx), int(cy), int(r), color, thickness)
    except Exception:
        _dbg_rect(img, cx - r, cy - r, r * 2, r * 2, color, thickness)

def _dbg_line(img, x1, y1, x2, y2, color, thickness=2):
    """画直线（0x06 拟合线用）；固件不支持就跳过"""
    try:
        img.draw_line(int(x1), int(y1), int(x2), int(y2), color, thickness)
    except Exception:
        pass

LINE_COLOR = getattr(image, "COLOR_CYAN", image.COLOR_GREEN)   # 画拟合线的颜色

def draw_detections(img, objs):
    """把检出的目标画框：什么颜色就画什么颜色的框 + 中心十字准心 + 标签

    圆盘(HIDE_CLASS_IDS)不画框 —— 它铺满画面，画出来会盖住所有东西。
    """
    if not DRAW_DETECTIONS:
        return
    for o in objs:
        if o.class_id in HIDE_CLASS_IDS:
            continue
        code = out_code(o.class_id)
        col  = CODE_COLOR.get(code, image.COLOR_WHITE)     # 认不出来的一律白框
        _dbg_rect(img, o.x, o.y, o.w, o.h, col, BOX_LINE_THICK)
        cx, cy = obj_center(o)
        if DRAW_CROSSHAIR:
            _dbg_cross(img, cx, cy, col)
        _dbg_text(img, o.x, max(0, o.y - 14),
                  "%s#%d>%d %.2f" % (name_of(o.class_id), o.class_id, code, o.score),
                  col)

def draw_debug(img, d, objs=()):
    draw_detections(img, objs)

    # ★ 三个固定框只在 0x03 模式下画；其它模式不画（收到别的命令自动消失）
    if d["mode"] == MODE_BOXES:
        for i, (x, y, w, h) in enumerate(ROI_BOX):
            ch  = d["box_char"][i]
            col = SLOT_COLOR.get(ch, image.COLOR_WHITE)
            dd  = d.get("box_deduce", -1)
            _dbg_rect(img, x, y, w, h, col, TARGET_LINE_THICK if ch == "R" else 2)
            _dbg_text(img, x, max(0, y - 16),
                      "%d%s%s%s" % (i + 1, ch,
                                    CODE_CHAR.get(d["box_code"][i], "?"),
                                    "*" if dd == i else ""),      # * = 任务码推断出来的
                      col)
            if d["box_mark"][i] is not None:
                bx, by, bw, bh = d["box_mark"][i]
                _dbg_rect(img, bx, by, bw, bh, image.COLOR_BLUE, 2)

    # 霍夫圆（0x04 识别到 two 时）：画真正的圆 + 圆心十字（橙色）
    if d["hough"] is not None:
        hx, hy, hr = d["hough"]
        _dbg_circle(img, hx, hy, hr, image.COLOR_ORANGE, 2)
        _dbg_cross(img, hx, hy, image.COLOR_ORANGE, 10)
        _dbg_mark(img, hx, hy, image.COLOR_ORANGE, 2)

    # 0x06 圆环校准：两点连线（青色）+ 车头参考方向（白竖线）+ 横向偏移（青线）
    if d["line"] is not None:
        lx1, ly1, lx2, ly2 = d["line"]
        _dbg_line(img, lx1, ly1, lx2, ly2, LINE_COLOR, 2)
        _dbg_line(img, TARGET_X, TARGET_Y - 80, TARGET_X, TARGET_Y + 80,
                  image.COLOR_WHITE, 1)
        mid = line_calib.last_mid
        if LINE_METHOD == "bearing" and mid is not None:
            # 中点离画面中心的横向距离 = 直观的偏航量
            _dbg_mark(img, mid[0], mid[1], LINE_COLOR, 3)
            _dbg_line(img, TARGET_X, mid[1], mid[0], mid[1], LINE_COLOR, 1)

    if d["target_rect"] is not None:
        bx, by, bw, bh = d["target_rect"]
        _dbg_rect(img, bx, by, bw, bh, image.COLOR_WHITE, TARGET_LINE_THICK)
        _dbg_cross(img, bx + bw // 2, by + bh // 2, image.COLOR_WHITE, CROSSHAIR_SIZE + 3)
        _dbg_text(img, bx, max(0, by - 28), d["target_txt"], image.COLOR_WHITE)
    _dbg_mark(img, TARGET_X, TARGET_Y, image.COLOR_WHITE, 2)
    _dbg_cross(img, TARGET_X, TARGET_Y, image.COLOR_WHITE, 12)
    # ★ 瞄准点（画面中心 + 夹爪偏置）：调 DXDY_OFFSET_X/Y 时就看这个紫色十字
    if DXDY_OFFSET_ENABLE and (AIM_X != TARGET_X or AIM_Y != TARGET_Y):
        _dbg_cross(img, AIM_X, AIM_Y, image.COLOR_PURPLE, 10)
        _dbg_mark(img, AIM_X, AIM_Y, image.COLOR_PURPLE, 2)
        _dbg_text(img, AIM_X + 8, AIM_Y + 4, "AIM", image.COLOR_PURPLE)
    _dbg_text(img, 2, 2, "%s F:%d %.0ffps" % (d["mode"], d["flag"], d["fps"]),
              image.COLOR_GREEN if d["flag"] else image.COLOR_WHITE)

# ==============================================================================
# 十一、主循环
# ==============================================================================
STEP_BY_MODE = {
    MODE_IDLE:  step_idle,
    MODE_TASK:  step_task,
    MODE_NEAR:  step_near,
    MODE_BOXES: step_boxes,
    MODE_RING:  step_ring,
    MODE_COLOR: step_color,
    MODE_LINE:  step_line,
}

def print_detections(objs):
    if not objs:
        print("[DET] 本帧没有任何检出")
        return
    print("[DET] " + " | ".join(
        "%s#%d>%d %.2f @%d,%d %dx%d" % (name_of(o.class_id), o.class_id,
                                        out_code(o.class_id), o.score,
                                        o.x, o.y, o.w, o.h)
        for o in objs))

def main():
    global reply_at_ms, RAW_OBJS
    parser = CmdParser()
    last_det_print = 0
    while not app.need_exit():
        now = time.ticks_ms()
        # 串口接收：按固定长度收帧
        for func, payload in parser.feed(serial.read()):
            if DEBUG_PRINT_RX:
                print("RX", hex(func), payload.hex(" "))
            on_cmd(func, payload, now)

        # ★ 补光灯：工作模式（除 0x00 空闲外）点亮，回空闲熄灭
        #   只在电平变化时写 GPIO，所以放在循环里没有开销
        set_light(mode != MODE_IDLE)

        img = cam.read()
        raw_objs = detector.detect(img, conf_th=CONF_TH, iou_th=IOU_TH)
        RAW_OBJS = raw_objs        # 给 0x02 的边沿静止判据用（原始框，未滤波）

        # ★ 框滤波：平滑 + 去闪烁。后面画图、取色、算 dx/dy 全部用滤波后的框
        if BOX_FILTER_ON:
            objs = box_filter.apply(raw_objs)
        else:
            objs = raw_objs

        # 只在你明确要往 MaixVision 上报时才打开（会占用系统 maix 协议串口）
        if DEBUG_REPORT and reporter is not None and len(objs) > 0:
            reporter.report(APP_CMD_DETECT_RES, encode_objs(objs))

        # 每 1 秒打印一次检出，方便核对 class_id
        if DEBUG_PRINT_DETECT and (now - last_det_print) >= 1000:
            last_det_print = now
            print_detections(objs)
            if ENABLE_HOUGH and hough.info.get("state") != "没有 two":
                print(hough.debug_line())      # 霍夫圆诊断（上一帧结果）
            if mode == MODE_NEAR:
                print(motion_event.debug_line())   # 0x02 动静事件诊断
            if mode == MODE_LINE:
                print(line_calib.debug_line())     # 0x06 连线校准诊断
            if mode == MODE_BOXES:
                _bc = DBG["box_code"]
                _bd = DBG["box_deduce"]
                print("[BOX] %s 认到%d个 | 发 c1=%d c2=%d c3=%d %s | 就绪=%d/3 任务码=%s"
                      % (box_state, box_n_known,
                         _bc[0], _bc[1], _bc[2],
                         ("(第%d个由任务码排除法补出)" % (_bd + 1)) if _bd >= 0
                         else "(三个都是实际认到的)",
                         sum(1 for s in box_slots if s.ok), task_code))

        DBG["mode"] = mode
        dbg_reset_frame()
        slots_update(objs)

        # ★ 只要画面里有 two 就跑霍夫圆（与当前模式无关），结果给画图和 0x04 复用
        DBG["hough"] = hough_update(img, objs)

        step = STEP_BY_MODE.get(mode)
        tx = step(img, objs, now) if step is not None else None

        # ★ 任何模式、无论是否检出目标，都按 REPLY_HZ 循环回发当前模式的帧
        if tx is not None:
            if reply_at_ms is None or now >= reply_at_ms:
                cmd_send(*tx)
                reply_at_ms = now + REPLY_PERIOD_MS

        if DEBUG_SHOW:
            DBG["fps"] = time.fps()
            try:
                draw_debug(img, DBG, objs)
            except Exception as e:
                print("draw_debug error:", e)
            disp.show(img)
        time.sleep_ms(1)

if __name__ == "__main__":
    try:
        main()
    finally:
        try:
            serial.deinit()
        except Exception:
            pass
        try:
            set_light(False)        # 退出时把补光灯关掉
        except Exception:
            pass
