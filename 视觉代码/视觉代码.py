import csi
import time
import math
from pyb import UART

# ===================== 参数配置区 =====================
# 【物块独立LAB阈值】0绿色，1红色，2蓝色
block_threshold = [
    (36, 76, -53, 12, 13, 62),    # 0‑绿色物块
    (18, 57, 25, 41, -3, 46),     # 1‑红色物块
    (17, 36, 27, -59, -53, -5)     # 2‑蓝色物块
]

IMG_W = 160
IMG_H = 120
center_x = IMG_W // 2
center_y = IMG_H // 2

min_pixel = 25
min_area = 25

# --------霍夫圆参数（检测黑色圆环）--------
circle_r_min = 10
circle_r_max = 25
circle_threshold = 1500
x_margin = 15
y_margin = 15
r_margin = 10

# 距离阈值，平方值！过滤离物块过近的圆
block_ring_dist_sq_thresh = 1225
# 色块黑名单膨胀余量，圆心在色块框外多少像素才算安全
block_border_offset = 8

# ================ 车身歪斜角（仅物块使用，圆环角度全部移除） ================
skew_enable = True
skew_sign = 1
skew_block_alpha = 0.40

# ---------------- 一维卡尔曼滤波 ----------------
class Kalman1D:
    def __init__(self, Q, R):
        self.x_est = 0.0
        self.p = 1.0
        self.Q = Q
        self.R = R

    def update(self, measure):
        x_pred = self.x_est
        p_pred = self.p + self.Q
        kg = p_pred / (p_pred + self.R)
        self.x_est = x_pred + kg * (measure - x_pred)
        self.p = (1.0 - kg) * p_pred
        return self.x_est

    def reset(self):
        self.x_est = 0.0
        self.p = 1.0


kf_block_dx = Kalman1D(Q=0.08, R=1.5)
kf_block_dy = Kalman1D(Q=0.08, R=1.5)
kf_ring_dx = Kalman1D(Q=0.03, R=6.5)
kf_ring_dy = Kalman1D(Q=0.03, R=6.5)


# ---------------- 角度平滑（仅物块歪斜角） ----------------
class AngleSmoother:
    def __init__(self, alpha, half_span):
        self.alpha = alpha
        self.half_span = half_span
        self.est = None

    def update(self, measure):
        measure = self._wrap(measure)
        if self.est is None:
            self.est = measure
        else:
            diff = self._wrap(measure - self.est)
            self.est = self._wrap(self.est + self.alpha * diff)
        return self.est

    def reset(self):
        self.est = None

    def _wrap(self, a):
        span = self.half_span
        return ((a + span) % (2 * span)) - span


smoother_block_skew = AngleSmoother(skew_block_alpha, 45)

# 工作模式开关
enable_block = False
enable_ring = False
track_target_id = 3

csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.QQVGA)
csi0.snapshot(time=2000)
csi0.auto_gain(False)
csi0.auto_whitebal(False)

clock = time.clock()
uart = UART(3, 9600)
uart.init(9600, bits=8, parity=None, stop=1)


def wrap_deg(a, half_span):
    return ((a + half_span) % (2 * half_span)) - half_span


def find_max(blobs):
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob.pixels > max_size:
            max_blob = blob
            max_size = blob.pixels
    return max_blob


# 判断圆心是否落在任意色块包围盒内
def circle_overlap_any_block(cx, cy, all_block_blobs, offset):
    for blob in all_block_blobs:
        x1 = blob.x - offset
        y1 = blob.y - offset
        x2 = blob.x + blob.w + offset
        y2 = blob.y + blob.h + offset
        if (x1 <= cx <= x2) and (y1 <= cy <= y2):
            return True
    return False


def calc_block_skew_deg(blob):
    return wrap_deg(math.degrees(blob.rotation), 45.0)


def draw_angle_arrow(img, cx, cy, deg, length, color):
    rad = math.radians(deg)
    x2 = int(cx + length * math.cos(rad))
    y2 = int(cy + length * math.sin(rad))



def cir_field(cir, name, default=None):
    try:
        v = getattr(cir, name)
    except AttributeError:
        return default
    return v() if callable(v) else v


def handle_stm32_command():
    global track_target_id, enable_block, enable_ring
    while uart.any():
        cmd_byte = uart.read(1)
        if cmd_byte == b'4':
            track_target_id = 0
            enable_block = True
            enable_ring = False
            kf_block_dx.reset()
            kf_block_dy.reset()
            smoother_block_skew.reset()
            print("--> 切换追踪：绿色物块 id=0 | 关闭圆环")
        elif cmd_byte == b'1':
            track_target_id = 1
            enable_block = True
            enable_ring = False
            kf_block_dx.reset()
            kf_block_dy.reset()
            smoother_block_skew.reset()
            print("--> 切换追踪：红色物块 id=1 | 关闭圆环")
        elif cmd_byte == b'3':
            track_target_id = 2
            enable_block = True
            enable_ring = False
            kf_block_dx.reset()
            kf_block_dy.reset()
            smoother_block_skew.reset()
            print("--> 切换追踪：蓝色物块 id=2 | 关闭圆环")
        elif cmd_byte == b'0':
            enable_block = False
            enable_ring = True
            kf_ring_dx.reset()
            kf_ring_dy.reset()
            print("--> 切换追踪：圆环模式 | 关闭物块识别")


# ----------------主循环----------------
while True:
    clock.tick()
    img = csi0.snapshot()

    handle_stm32_command()

    block_dx_raw = None
    block_dy_raw = None
    block_cx = 0
    block_cy = 0
    all_blobs = []
    skew_raw = None

    # ========== 物块识别 ==========
    if enable_block:
        all_blobs = img.find_blobs(block_threshold,
                                   pixels_threshold=min_pixel,
                                   area_threshold=min_area,
                                   merge=False)
        target_blobs = []
        for b in all_blobs:
            if b.code == (1 << track_target_id):
                target_blobs.append(b)

        if target_blobs:
            target_blob = find_max(target_blobs)
            block_cx = target_blob.cx
            block_cy = target_blob.cy
            block_dx_raw = block_cx - center_x
            block_dy_raw = block_cy - center_y
            angle = int(math.degrees(target_blob.rotation))
            img.draw_keypoints([(block_cx, block_cy, angle)], size=5, color=(255, 255, 255))
            skew_raw = calc_block_skew_deg(target_blob)
            draw_angle_arrow(img, block_cx, block_cy,
                             math.degrees(target_blob.rotation), 18, (255, 255, 0))

    # ========== 霍夫圆环检测：只保留基础过滤，选离画面中心最近圆环 ==========
    ring_dx_raw = None
    ring_dy_raw = None
    target_ring = None
    ring_n_cand = 0
    ring_n_valid = 0

    if enable_ring:
        circles = img.find_circles(threshold=circle_threshold,
                                   r_min=circle_r_min,
                                   r_max=circle_r_max,
                                   x_margin=x_margin,
                                   y_margin=y_margin,
                                   r_margin=r_margin)
        if circles:
            valid = []
            for cir in circles:
                cx = cir_field(cir, 'x')
                cy = cir_field(cir, 'y')
                cr = cir_field(cir, 'r')
                # 过滤：圆心落在物块色块内
                if circle_overlap_any_block(cx, cy, all_blobs, block_border_offset):
                    continue
                # 过滤：距离物块太近
                if block_dx_raw is not None:
                    dx_ = cx - block_cx
                    dy_ = cy - block_cy
                    if dx_ * dx_ + dy_ * dy_ < block_ring_dist_sq_thresh:
                        continue
                # 边缘截断、底部夹爪过滤
                if (cx < 6 or cx > IMG_W - 6 or cy < 6 or cy > IMG_H - 14):
                    continue
                if (cy + cr) > IMG_H - 2:
                    continue
                # 存入有效列表，附带到画面中心距离平方
                dist_sq = (cx - center_x)**2 + (cy - center_y)**2
                valid.append((dist_sq, cx, cy, cr))

            ring_n_cand = len(circles)
            ring_n_valid = len(valid)
            # 【核心】按距离画面中心从小到大排序，取最近的圆环
            if valid:
                valid.sort(key=lambda x: x[0])
                _, rx, ry, rr = valid[0]
                target_ring = type('obj', (object,), {'x': rx, 'y': ry, 'r': rr})

            # 调试绘制所有有效候选圆（蓝色）
            for item in valid:
                _, vx, vy, vr = item
                img.draw_circle((int(vx), int(vy), int(vr)), color=(0, 100, 255))

    if target_ring is not None:
        ring_dx_raw = target_ring.x - center_x
        ring_dy_raw = target_ring.y - center_y
        img.draw_keypoints([(int(target_ring.x), int(target_ring.y), 0)], size=6, color=(255, 255, 255))
        img.draw_circle((int(target_ring.x), int(target_ring.y), int(target_ring.r)), color=(255, 255, 255))

    # ========== 卡尔曼滤波更新，保证圆环输出稳定 ==========
    out_block_dx = None
    out_block_dy = None
    out_ring_dx = None
    out_ring_dy = None

    if enable_block and block_dx_raw is not None and block_dy_raw is not None:
        out_block_dx = kf_block_dx.update(block_dx_raw)
        out_block_dy = kf_block_dy.update(block_dy_raw)

    if enable_ring and ring_dx_raw is not None and ring_dy_raw is not None:
        out_ring_dx = kf_ring_dx.update(ring_dx_raw)
        out_ring_dy = kf_ring_dy.update(ring_dy_raw)

    # 物块歪斜角平滑（圆环不再输出角度）
    skew_out = None
    if skew_enable and skew_raw is not None and enable_block:
        skew_out = smoother_block_skew.update(skew_raw)
    if skew_out is not None:
        skew_out = skew_sign * skew_out

    def fmt_val(v):
        if v is None:
            return "-999"
        return str(int(round(v)))

    if enable_block:
        mode_tag = str(track_target_id)
    elif enable_ring:
        mode_tag = "8"
    else:
        mode_tag = "6"

    tx_str = "{},{},{},{},{}\n".format(fmt_val(out_block_dx), fmt_val(out_block_dy),
                                        fmt_val(out_ring_dx), fmt_val(out_ring_dy),
                                        mode_tag)
    uart.write(tx_str)

    print("fps:{:.1f} | blk_en={} ring_en={} | block dx={} dy={} | ring dx={} dy={} | tag={} | skew={} | ring_cand={}/{}".format(
        clock.fps(), enable_block, enable_ring,
        fmt_val(out_block_dx), fmt_val(out_block_dy),
        fmt_val(out_ring_dx), fmt_val(out_ring_dy),
        mode_tag, fmt_val(skew_out),
        ring_n_valid, ring_n_cand))
    time.sleep_ms(30)
