# -*- coding: utf-8 -*-
"""
================================================================================
 MaixCAM / MaixCAM-Pro 视觉框架 v5
 配套协议：《视觉-电控通信》
 ------------------------------------------------------------------------------
 相对 v4 的修改（对应你提的三个问题）：

  ① 串口引脚：A0 / A1 在 MaixCAM(Pro) 上【根本不存在】。
     芯片引出来的引脚只有 A14~A19 / A22~A29 / B3 / B26 / B27 / C2 / P18~P24 / BL。
     原来 pinmap.set_pin_function("A0", "UART2_RX") 会打印 "pin A0 not valid"
     并返回错误码，但代码没有检查 → 失败是静默的，数据根本没上引脚。
     现在按真实存在的 UART 引脚组合配置，并用 err.check_raise 让失败直接报错。

  ② 波特率：官方文档明确说 MaixCAM / MaixCAM-Pro 只验证过 115200，
     其它波特率误码率可能很高甚至驱动不支持。默认改为 115200（两端必须一致）。

  ③ 你在线路上抓到的其实是 comm.CommProtocol 的 MaixVision 上报帧
     (帧头 AA CA AC BB ...)，它自动使用系统 maix 协议串口 UART0(/dev/ttyS0)、115200，
     而原代码是 `if len(objs) > 0` 才上报 → 表现就是"只有框里有东西才发"。
     现在 DEBUG_REPORT 默认关闭；回帧完全由《视觉-电控通信》决定，
     任何模式都按固定频率循环回发，与有没有检出目标无关。

  顺带修掉的坑：
     * 收帧改成按固定长度解析（dxL/dyL 完全可能是 0xBB，原来的"找 0xBB 当帧尾"会截断）
     * flag 锁存：收到命令后清 0，条件满足置 1 后保持到下次命令，避免 1/0 抖动
     * 0x03 判据按设计书改成"框内从无到有"(empty_to_full)，原来用的是 static
     * 去掉"任何模式下自动补发 0x03"的私货帧，避免电控收到协议外的帧
================================================================================
"""
from maix import camera, display, image, nn, app, comm, time, uart, pinmap, gpio, err
import os
import struct

# ==============================================================================
# 一、配置区（标定只改这一段）
# ==============================================================================
# ---------- 模型 ----------
MODEL_PATH     = "model_322427.mud"
MODEL_FALLBACK = "/root/models/disk/model_322427.mud"
CONF_TH        = 0.5
IOU_TH         = 0.45

# ---------- 串口接线方案（先选这个！） ----------
# MaixCAM / MaixCAM-Pro 实际存在的串口（Sipeed 官方文档 + MaixCDK pinmap 源码）：
#   "A16A17"     UART0  A16=TX  A17=RX   /dev/ttyS0
#                与 Type-C 转接小板引出的是同一组线；开机有系统日志、
#                系统默认的 maix protocol 口也是它（MaixVision 上报就走这里）
#   "A19A18"     UART1  A19=TX  A18=RX   /dev/ttyS1   ★官方推荐自定义通信，干净
#   "A28A29"     UART1  A28=TX  A29=RX   /dev/ttyS1   1.25mm 6pin 排线上的串口
#   "A28A29_U2"  UART2  A28=TX  A29=RX   /dev/ttyS2   同一组引脚映射到 UART2
#
# ★ A0 / A1 不存在！所以原代码无论怎么接都不可能出数据。
# ★ 你的电控现在能收到数据的那组线，一定在 A16/A17（或 Type-C 转接小板）上，
#   因为 MaixVision 上报帧只能从 UART0 出来。想省事就先保持 "A16A17"；
#   想干净（无开机日志、不与系统抢占）就改接 A19/A18 并把这里改成 "A19A18"。
UART_OPTION = "A16A17"
UART_PLAN = {
    "A16A17":    {"dev": "/dev/ttyS0", "pins": (("A16", "UART0_TX"), ("A17", "UART0_RX"))},
    "A19A18":    {"dev": "/dev/ttyS1", "pins": (("A19", "UART1_TX"), ("A18", "UART1_RX"))},
    "A28A29":    {"dev": "/dev/ttyS1", "pins": (("A28", "UART1_TX"), ("A29", "UART1_RX"))},
    "A28A29_U2": {"dev": "/dev/ttyS2", "pins": (("A28", "UART2_TX"), ("A29", "UART2_RX"))},
}
UART_BAUD = 9600          # 官方对 MaixCAM/Pro 只验证过 115200，两端必须一致

# ---------- dx/dy 字节序 ----------
# True  = 先发高字节 = 设计书写的 dxH dxL
# False = 先发低字节
INT16_BIG_ENDIAN = True

# ---------- 回帧节流（任何模式都按这个频率循环回发） ----------
REPLY_HZ        = 50
REPLY_PERIOD_MS = max(1, 1000 // REPLY_HZ)

# ---------- 目标丢失时 dx/dy 填什么 ----------
NO_TARGET_DXDY = 0

# ---------- flag 锁存 ----------
# True : 收到命令后 flag=0，条件满足置 1 后一直保持 1（设计书："只有 flag=1 电控执行下一步"）
# False: 每帧实时反映（会 1/0 抖动）
FLAG_LATCH = True

# ---------- 位置低通 ----------
LOWPASS_ALPHA = 0.4
MAX_JUMP_PX   = 30

# ---------- 模型类别表 ----------
CLASS_NAME = {
    0: "three", 1: "red", 2: "two", 3: "one", 4: "blue",
    5: "light blue", 6: "black", 7: "yellow", 8: "green", 9: "disk",
}
CLASS_CHAR = {
    0: "3", 1: "R", 2: "2", 3: "1", 4: "B",
    5: "c", 6: "K", 7: "Y", 8: "G", 9: "D",
}
# 框内取色时要忽略的类别：数字标记 one/two/three + 圆盘
BOX_IGNORE_IDS = {0, 2, 3, 9}
# 圆盘类别（0x02 用）
DISK_CLASS_ID = 9
# 圆环：模型 class_id → 圆环编号（编号只能是 1/2/3，0 留给"没检到"）
RING_CLASS_TO_NUM = {3: 1, 2: 2, 0: 3}
# 圆环优先顺序（同时检到多个环时先报哪个）
RING_PRIORITY = [3, 2, 0]

# ---------- 0x03 的三个固定框：c1=左上 c2=下 c3=右上 ----------
ROI_BOX = [
    [10,  20,  100, 100],      # c1 左上
    [110, 200,  100, 100],     # c2 下
    [250, 20,   100, 100],     # c3 右上
]
# ---------- 三框就绪判据 ----------
# 设计书 0x03 写的是"等待到框内从无到有"，所以默认 empty_to_full；
# 想用"静止N帧"的旧逻辑就改成 "static"
BOX_READY_MODE    = "empty_to_full"
ROI_STATIC_FRAMES = 12
ROI_MOVE_THRESH   = 4.0

# ---------- 补光灯（MaixCAM-Pro: B3 是照明 LED，高电平使能） ----------
ENABLE_FILL_LIGHT = True
FILL_LIGHT_LEVEL  = 0

# ---------- 调试开关 ----------
DEBUG_SHOW     = True
# ★ 默认关闭！它会把 MaixVision 上报帧打到系统 maix 协议串口(UART0,115200)上，
#   和《视觉-电控通信》的 [AA ... BB] 帧混在一起；而且原代码只在检出目标时上报。
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
CMD_DISC  = 0x02
CMD_BOXES = 0x03
CMD_RING  = 0x04
CMD_COLOR = 0x05
# 每条命令 payload 的固定长度（设计书规定）
# ★ 必须按长度收帧：dxL/dyL/颜色码完全可能是 0xBB，用"找 0xBB 当帧尾"会截断
CMD_LEN = {
    CMD_IDLE:  0,   # [AA 00 BB]
    CMD_TASK:  3,   # [AA 01 c1 c2 c3 BB]
    CMD_DISC:  0,   # [AA 02 BB]
    CMD_BOXES: 0,   # [AA 03 BB]
    CMD_RING:  0,   # [AA 04 BB]
    CMD_COLOR: 1,   # [AA 05 color BB]
}

MODE_IDLE  = "IDLE"
MODE_TASK  = "TASK"
MODE_DISC  = "DISC"
MODE_BOXES = "BOXES"
MODE_RING  = "RING"
MODE_COLOR = "COLOR"

flag = 0
mode = MODE_IDLE
target_color = 0
task_code = (0, 0, 0)
reply_at_ms = None
last_disc  = (NO_TARGET_DXDY, NO_TARGET_DXDY)
last_ring  = (NO_TARGET_DXDY, NO_TARGET_DXDY)
last_color = (NO_TARGET_DXDY, NO_TARGET_DXDY)
ring_hold  = 0

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
if ENABLE_FILL_LIGHT:
    try:
        pinmap.set_pin_function("B3", "GPIOB3")
        led = gpio.GPIO("GPIOB3", gpio.Mode.OUT)
        led.value(FILL_LIGHT_LEVEL)
    except Exception as e:
        print("[LED] 补光灯初始化失败(忽略):", e)
        led = None

# 使用圆盘模型
_model = MODEL_PATH if os.path.exists(MODEL_PATH) else MODEL_FALLBACK
detector = nn.YOLOv5(model=_model)
cam  = camera.Camera(detector.input_width(), detector.input_height(),
                     detector.input_format())
disp = display.Display()

TARGET_X = detector.input_width()  // 2
TARGET_Y = detector.input_height() // 2

reporter = comm.CommProtocol(buff_size=1024) if DEBUG_REPORT else None
APP_CMD_DETECT_RES = 0x02

# ==============================================================================
# 四、串口层
# ==============================================================================
def pack_i16(val):
    p = struct.pack(">h" if INT16_BIG_ENDIAN else "<h", int(val))
    return p[0], p[1]

def reply_dxdy(func, flag_byte, dx, dy):
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
# 五、检测层（YOLOv5）
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
    return pick_max([o for o in objs if o.class_id == cid])

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

# ==============================================================================
# 六、判定工具
# ==============================================================================
def set_flag(v):
    """v=1 表示本帧条件满足；锁存模式下置 1 后不再自动掉回 0"""
    global flag
    if v:
        flag = 1
    elif not FLAG_LATCH:
        flag = 0
    # FLAG_LATCH 且 v=0：保持上次的 flag

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

class BoxSlot:
    """一个固定框的状态机，输出 R(就绪)/E(空)/!(有但不合格)"""
    def __init__(self):
        self.reset()
    def reset(self):
        self.seen_empty = False
        self.has        = False
        self.ok         = False
        self.color      = 0
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
            self.color = 0
            if BOX_READY_MODE == "static":
                self.ok = False
            return self.ok
        color, cx, cy = det
        self.color = color
        self.has   = True
        if BOX_READY_MODE == "empty_to_full":
            # 设计书：等待到框内从无到有
            if self.seen_empty and not self.ok:
                self.ok = True
            if self.ok and self.latch == 0:
                self.latch = color
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
            self.latch = color
        return self.ok
    def report_color(self):
        return self.latch if self.ok else self.color
    def state_char(self):
        if self.ok:
            return "R"
        if not self.has:
            return "E"
        return "!"

lp_disc  = Smoother(LOWPASS_ALPHA)
lp_ring  = Smoother(LOWPASS_ALPHA)
lp_color = Smoother(LOWPASS_ALPHA)
box_slots = [BoxSlot() for _ in range(3)]

def reset_all():
    global ring_hold, last_disc, last_ring, last_color
    ring_hold = 0
    last_disc  = (NO_TARGET_DXDY, NO_TARGET_DXDY)
    last_ring  = (NO_TARGET_DXDY, NO_TARGET_DXDY)
    last_color = (NO_TARGET_DXDY, NO_TARGET_DXDY)
    clear_flag()
    lp_disc.reset()
    lp_ring.reset()
    lp_color.reset()
    for s in box_slots:
        s.reset()

def slots_update(objs):
    for i, roi in enumerate(ROI_BOX):
        o = pick_in_roi(objs, roi, BOX_IGNORE_IDS)
        det = None
        if o is not None:
            cx, cy = obj_center(o)
            det = (o.class_id, cx, cy)
            DBG["box_mark"][i] = (cx, cy)
        box_slots[i].update(det)
        DBG["box_char"][i]  = box_slots[i].state_char()
        DBG["box_color"][i] = box_slots[i].report_color()

def build_boxes_reply(f):
    c1, c2, c3 = [s.report_color() for s in box_slots]
    return (CMD_BOXES, bytes([f & 0xFF, c1 & 0xFF, c2 & 0xFF, c3 & 0xFF]))

DBG = {
    "mode":        MODE_IDLE,
    "flag":        0,
    "fps":         0.0,
    "box_char":    ["E", "E", "E"],
    "box_color":   [0, 0, 0],
    "box_mark":    [None, None, None],
    "disc":        None,
    "ring":        None,
    "color":       None,
}
def dbg_reset_frame():
    DBG["box_mark"] = [None, None, None]
    DBG["disc"] = None
    DBG["ring"] = None
    DBG["color"] = None

# ==============================================================================
# 七、每帧处理：每个 step 都必定返回一帧（与有没有检出目标无关）
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

def step_disc(img, objs, now):
    global last_disc
    o = pick_class(objs, DISK_CLASS_ID)
    if o is None:
        lp_disc.reset()
        set_flag(0)
        DBG["flag"] = flag
        # 已经锁存到目标 → 继续发上一次的偏差，避免电控看到 0,0 以为对准了
        if flag:
            return reply_dxdy(CMD_DISC, flag, last_disc[0], last_disc[1])
        return reply_dxdy(CMD_DISC, 0, NO_TARGET_DXDY, NO_TARGET_DXDY)
    cx, cy = obj_center(o)
    pos = lp_disc.update(cx, cy)
    dx, dy = int(pos[0] - TARGET_X), int(pos[1] - TARGET_Y)
    last_disc = (dx, dy)
    set_flag(1)
    DBG["disc"] = (int(pos[0]), int(pos[1]))
    DBG["flag"] = flag
    return reply_dxdy(CMD_DISC, flag, dx, dy)

def step_boxes(img, objs, now):
    ready = all(s.ok for s in box_slots)
    set_flag(1 if ready else 0)
    DBG["flag"] = flag
    return build_boxes_reply(flag)

def step_ring(img, objs, now):
    global flag, last_ring, ring_hold
    target = None
    for cid in RING_PRIORITY:
        o = pick_class(objs, cid)
        if o is not None:
            target = (cid, o)
            break
    if target is None:
        lp_ring.reset()
        if FLAG_LATCH and ring_hold:
            # 已经识别到过圆环 → flag 保持圆环编号，偏差沿用上一次
            flag = ring_hold
            DBG["flag"] = flag
            return reply_dxdy(CMD_RING, flag, last_ring[0], last_ring[1])
        flag = 0
        DBG["flag"] = 0
        return reply_dxdy(CMD_RING, 0, NO_TARGET_DXDY, NO_TARGET_DXDY)
    cid, o = target
    num = RING_CLASS_TO_NUM.get(cid, 0)
    cx, cy = obj_center(o)
    pos = lp_ring.update(cx, cy)
    dx, dy = int(pos[0] - TARGET_X), int(pos[1] - TARGET_Y)
    last_ring = (dx, dy)
    ring_hold = num
    flag = num & 0xFF
    DBG["ring"] = (num, int(pos[0]), int(pos[1]))
    DBG["flag"] = flag
    return reply_dxdy(CMD_RING, flag, dx, dy)

def step_color(img, objs, now):
    global last_color
    o = pick_class(objs, target_color)
    if o is None:
        lp_color.reset()
        set_flag(0)
        DBG["flag"] = flag
        if flag:
            return reply_dxdy(CMD_COLOR, flag, last_color[0], last_color[1])
        return reply_dxdy(CMD_COLOR, 0, NO_TARGET_DXDY, NO_TARGET_DXDY)
    cx, cy = obj_center(o)
    pos = lp_color.update(cx, cy)
    dx, dy = int(pos[0] - TARGET_X), int(pos[1] - TARGET_Y)
    last_color = (dx, dy)
    set_flag(1)
    DBG["color"] = (o.class_id, int(pos[0]), int(pos[1]))
    DBG["flag"] = flag
    return reply_dxdy(CMD_COLOR, flag, dx, dy)

# ==============================================================================
# 八、命令处理（收到命令后 flag 清 0）
# ==============================================================================
def on_cmd(func, payload, now):
    global mode, target_color, task_code, reply_at_ms, ring_hold
    global last_disc, last_ring, last_color
    clear_flag()
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
    if func == CMD_DISC:
        lp_disc.reset()
        last_disc = (NO_TARGET_DXDY, NO_TARGET_DXDY)
        mode = MODE_DISC
        return
    if func == CMD_BOXES:
        for s in box_slots:
            s.reset()
        mode = MODE_BOXES
        return
    if func == CMD_RING:
        lp_ring.reset()
        ring_hold = 0
        last_ring = (NO_TARGET_DXDY, NO_TARGET_DXDY)
        mode = MODE_RING
        return
    if func == CMD_COLOR:
        if len(payload) >= 1:
            target_color = payload[0]
        lp_color.reset()
        last_color = (NO_TARGET_DXDY, NO_TARGET_DXDY)
        mode = MODE_COLOR
        return

# ==============================================================================
# 九、调试绘制
# ==============================================================================
SLOT_COLOR = {
    "E": image.COLOR_RED,
    "R": image.COLOR_GREEN,
    "!": image.COLOR_PURPLE,
}

def _dbg_text(img, x, y, s, color):
    try:
        img.draw_string(int(x), int(y), s, color, 1, 2)
    except Exception:
        pass

def _dbg_rect(img, x, y, w, h, color, thickness=1):
    try:
        img.draw_rect(int(x), int(y), int(w), int(h), color, thickness=thickness)
    except Exception:
        pass

def _dbg_mark(img, cx, cy, color, half=4):
    _dbg_rect(img, cx - half, cy - half, half * 2, half * 2, color, -1)

def draw_debug(img, d):
    for i, (x, y, w, h) in enumerate(ROI_BOX):
        ch  = d["box_char"][i]
        col = SLOT_COLOR.get(ch, image.COLOR_WHITE)
        _dbg_rect(img, x, y, w, h, col, 1)
        _dbg_text(img, x, max(0, y - 12),
                  "%d%s%s" % (i + 1, ch, CLASS_CHAR.get(d["box_color"][i], "?")),
                  col)
        if d["box_mark"][i] is not None:
            _dbg_mark(img, d["box_mark"][i][0], d["box_mark"][i][1],
                      image.COLOR_BLUE)
    if d["disc"] is not None:
        _dbg_mark(img, d["disc"][0], d["disc"][1], image.COLOR_BLUE, 6)
    if d["ring"] is not None:
        num, cx, cy = d["ring"]
        _dbg_mark(img, cx, cy, image.COLOR_BLUE, 5)
        _dbg_text(img, cx + 8, cy - 6, "n%d" % num, image.COLOR_BLUE)
    if d["color"] is not None:
        cid, cx, cy = d["color"]
        _dbg_mark(img, cx, cy, image.COLOR_BLUE, 5)
        _dbg_text(img, cx + 8, cy - 6, CLASS_CHAR.get(cid, "?"), image.COLOR_BLUE)
    _dbg_mark(img, TARGET_X, TARGET_Y, image.COLOR_WHITE, 2)
    _dbg_text(img, 2, 2, "%s F:%d %.0ffps" % (d["mode"], d["flag"], d["fps"]),
              image.COLOR_GREEN if d["flag"] else image.COLOR_WHITE)

# ==============================================================================
# 十、主循环
# ==============================================================================
STEP_BY_MODE = {
    MODE_IDLE:  step_idle,
    MODE_TASK:  step_task,
    MODE_DISC:  step_disc,
    MODE_BOXES: step_boxes,
    MODE_RING:  step_ring,
    MODE_COLOR: step_color,
}

def main():
    global reply_at_ms
    parser = CmdParser()
    while not app.need_exit():
        now = time.ticks_ms()
        # 串口接收：按固定长度收帧
        for func, payload in parser.feed(serial.read()):
            if DEBUG_PRINT_RX:
                print("RX", hex(func), payload.hex(" "))
            on_cmd(func, payload, now)

        img = cam.read()
        objs = detector.detect(img, conf_th=CONF_TH, iou_th=IOU_TH)

        # 只在你明确要往 MaixVision 上报时才打开（会占用系统 maix 协议串口）
        if DEBUG_REPORT and reporter is not None and len(objs) > 0:
            reporter.report(APP_CMD_DETECT_RES, encode_objs(objs))

        DBG["mode"] = mode
        dbg_reset_frame()
        slots_update(objs)

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
                draw_debug(img, DBG)
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


