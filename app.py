# ================================================================
# CleanBot v1.0 INDUSTRIAL — Zero Delay · Hardware E-Stop
# ================================================================
# Motors:  BTS7960 #1 Left  GPIO 18(RPWM) 23(LPWM) 24(EN)
#          BTS7960 #2 Right GPIO 19(RPWM) 20(LPWM) 21(EN)
# Relays:  Brush GPIO 17  Water GPIO 26  (gpiozero active-HIGH)
# EStop:   GPIO 5 (Button pull_up=True)
#
# ARCHITECTURE:
#   • _estop_thread    — HIGHEST priority, polls every 1ms, kills GPIO directly
#   • _movement_loop   — 200Hz, manual only, ZERO edge protection
#   • _auto_clean      — edge protection ON, _isleep interruptible
#   • _pattern_worker  — NO edge protection, interruptible
#   • _capture_loop    — dedicated grab thread, no decode block
#   • _vision_loop     — processes latest frame only
#   • _stats_loop      — 5Hz WebSocket broadcast
# ================================================================

import os, sys, time, math, json, csv, io, sqlite3, threading, atexit
from datetime import datetime
from collections import deque

from flask import Flask, render_template, Response, jsonify, request, send_file
from flask_socketio import SocketIO
import cv2, numpy as np
import lgpio

try:
    from gpiozero import OutputDevice, Button
    from gpiozero.pins.lgpio import LGPIOFactory
    from gpiozero import Device
    Device.pin_factory = LGPIOFactory()
    _gz_ok = True
except Exception as e:
    print(f"⚠ gpiozero: {e}"); _gz_ok = False

try:
    from ultralytics import YOLO
    _yolo_avail = True
except:
    _yolo_avail = False

app = Flask(__name__)
app.config['SECRET_KEY'] = 'cb42'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# ================================================================
# PIN MAP
# ================================================================
# BTS7960: RPWM=forward PWM, LPWM=backward PWM, EN=enable
LEFT_RPWM=18;  LEFT_LPWM=23;  LEFT_EN=24
RIGHT_RPWM=19; RIGHT_LPWM=20; RIGHT_EN=21
BRUSH_PIN=17; WATER_PIN=26; ESTOP_PIN=5

# ================================================================
# TUNING
# ================================================================
SPEED_FULL    = 75
SPEED_CAUTION = 45
SPEED_CRAWL   = 25
SPEED_AUTO    = 50
EDGE_CRITICAL = 8
EDGE_WARN     = 20
EDGE_SAFE     = 40
MIN_CONF      = 60
TRIPLE_WINDOW = 1.5
MODEL_PATH    = '/home/solarbot/cleanbot/best.pt'

# ================================================================
# lgpio — MOTOR CONTROL ONLY
# ================================================================
h = None
gpio_ok = False
try:
    h = lgpio.gpiochip_open(0)
    for p in [LEFT_RPWM, LEFT_LPWM, LEFT_EN, RIGHT_RPWM, RIGHT_LPWM, RIGHT_EN]:
        lgpio.gpio_claim_output(h, p, 0)
    lgpio.tx_pwm(h, LEFT_RPWM,  1000, 0)
    lgpio.tx_pwm(h, LEFT_LPWM,  1000, 0)
    lgpio.tx_pwm(h, RIGHT_RPWM, 1000, 0)
    lgpio.tx_pwm(h, RIGHT_LPWM, 1000, 0)
    lgpio.gpio_write(h, LEFT_EN,  1)
    lgpio.gpio_write(h, RIGHT_EN, 1)
    gpio_ok = True
    print("✅ BTS7960: LEFT(18,23,24) RIGHT(19,20,21)")
except Exception as e:
    print(f"❌ lgpio: {e}")

# ================================================================
# FORCE STOP — DIRECT GPIO, NO LOCKS, ZERO DELAY
# Called from ANY thread including estop thread
# ================================================================
def _force_stop():
    """Bypass every lock. Write GPIO directly. Must never raise."""
    try:
        if gpio_ok and h:
            lgpio.tx_pwm(h, LEFT_RPWM,  1000, 0)
            lgpio.tx_pwm(h, LEFT_LPWM,  1000, 0)
            lgpio.tx_pwm(h, RIGHT_RPWM, 1000, 0)
            lgpio.tx_pwm(h, RIGHT_LPWM, 1000, 0)
    except:
        pass

# ================================================================
# MOTOR PRIMITIVES
# ================================================================
def _spd(val):
    return max(0, min(100, int(val)))

def robot_forward(s=SPEED_FULL):
    if not gpio_ok: return
    lgpio.tx_pwm(h, LEFT_RPWM,  1000, _spd(s)); lgpio.tx_pwm(h, LEFT_LPWM,  1000, 0)
    lgpio.tx_pwm(h, RIGHT_RPWM, 1000, _spd(s)); lgpio.tx_pwm(h, RIGHT_LPWM, 1000, 0)

def robot_backward(s=SPEED_FULL):
    if not gpio_ok: return
    lgpio.tx_pwm(h, LEFT_RPWM,  1000, 0); lgpio.tx_pwm(h, LEFT_LPWM,  1000, _spd(s))
    lgpio.tx_pwm(h, RIGHT_RPWM, 1000, 0); lgpio.tx_pwm(h, RIGHT_LPWM, 1000, _spd(s))

def robot_turn_left(s=SPEED_FULL):
    if not gpio_ok: return
    lgpio.tx_pwm(h, LEFT_RPWM,  1000, 0);      lgpio.tx_pwm(h, LEFT_LPWM,  1000, _spd(s))
    lgpio.tx_pwm(h, RIGHT_RPWM, 1000, _spd(s)); lgpio.tx_pwm(h, RIGHT_LPWM, 1000, 0)

def robot_turn_right(s=SPEED_FULL):
    if not gpio_ok: return
    lgpio.tx_pwm(h, LEFT_RPWM,  1000, _spd(s)); lgpio.tx_pwm(h, LEFT_LPWM,  1000, 0)
    lgpio.tx_pwm(h, RIGHT_RPWM, 1000, 0);      lgpio.tx_pwm(h, RIGHT_LPWM, 1000, _spd(s))

def robot_stop():
    _force_stop()  # BTS7960: RPWM=0, LPWM=0 on both drivers

# ================================================================
# gpiozero — RELAYS + ESTOP BUTTON
# ================================================================
brush_relay  = None
water_valve  = None
estop_button = None
try:
    brush_relay  = OutputDevice(BRUSH_PIN, initial_value=False)
    water_valve  = OutputDevice(WATER_PIN, initial_value=False)
    estop_button = Button(ESTOP_PIN, pull_up=True, bounce_time=0.02)
    print(f"✅ Relays brush=GPIO{BRUSH_PIN} water=GPIO{WATER_PIN} estop=GPIO{ESTOP_PIN}")
except Exception as e:
    print(f"❌ Relays: {e}")

# ================================================================
# GLOBAL STATE
# ================================================================
# ── E-Stop state — ONE atomic flag, set by multiple paths ──
emergency_stop    = False        # master kill flag
_estop_event      = threading.Event()  # wakes sleeping threads instantly

# ── Operation state ──
auto_cleaning     = False
pattern_running   = False
movement_mode     = 'STOP'       # STOP FORWARD BACKWARD LEFT RIGHT
current_speed     = SPEED_FULL
movement_lock     = threading.Lock()

# ── Triple-press counters ──
forward_press_count  = 0
backward_press_count = 0
last_fwd_press       = 0.0
last_bwd_press       = 0.0

# ── Actuator state ──
brush_active = False
water_active = False

# ── Vision / detection ──
detection_active  = False
panel_dirt_level  = 0
current_detections = []
damage_count      = 0

# ── Edge detection (used by auto-clean ONLY) ──
edge_distances   = {'front': 99, 'back': 99, 'left': 99, 'right': 99}
barrier_detected = {'front': False, 'back': False, 'left': False, 'right': False}
edge_confidence  = {'front': 0,  'back': 0,  'left': 0,  'right': 0}

# ── Session / telemetry ──
session_start  = None
area_covered   = 0.0
last_clean_time = None
robot_x = 50.0; robot_y = 50.0
path_history   = deque(maxlen=400)
water_level    = 100.0

# ── Chart ring buffers (5Hz fill) ──
dirt_timeline  = deque(maxlen=60)
eff_timeline   = deque(maxlen=60)
speed_timeline = deque(maxlen=60)

# ── Camera ──
camera     = None
camera_ok  = False
_raw_frame = None
_ann_frame = None
_raw_lock  = threading.Lock()
_ann_lock  = threading.Lock()
_cap_event = threading.Event()

# ── YOLO ──
model = None

# ── Edge detection buf ──
_edge_buf = {d: deque(maxlen=3) for d in ['front','back','left','right']}

# ================================================================
# RELAY CONTROL
# ================================================================
def brush_on():
    global brush_active
    if emergency_stop or not brush_relay: return
    brush_relay.on(); brush_active = True
    print(f"🔄 BRUSH ON  GPIO{BRUSH_PIN}")

def brush_off():
    global brush_active
    if brush_relay: brush_relay.off()
    brush_active = False
    print(f"🔄 BRUSH OFF GPIO{BRUSH_PIN}")

def water_on():
    global water_active
    if emergency_stop or not water_valve: return
    water_valve.on(); water_active = True
    print(f"💧 WATER ON  GPIO{WATER_PIN}")

def water_off():
    global water_active
    if water_valve: water_valve.off()
    water_active = False
    print(f"💧 WATER OFF GPIO{WATER_PIN}")

def _kill_all():
    """Full hardware kill — motors first (direct GPIO), then relays."""
    global brush_active, water_active
    _force_stop()
    try:
        if brush_relay: brush_relay.off()
        if water_valve: water_valve.off()
    except: pass
    brush_active = False
    water_active = False

# ================================================================
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# INDUSTRIAL E-STOP THREAD — 1ms poll, HIGHEST priority
# Watches emergency_stop flag. The MOMENT it's True:
#   1. _force_stop() — GPIO PWM=0, all direction pins LOW
#   2. relays off
#   3. Clears movement_mode
#   4. Sets _estop_event to wake all blocked _isleep() calls
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# ================================================================
def _estop_watchdog():
    """
    Dedicated watchdog — polls every 1ms.
    Keeps killing motors as long as emergency_stop is True.
    Nothing can override this thread — it runs forever.
    """
    while True:
        if emergency_stop:
            _force_stop()          # direct GPIO, zero delay
            _estop_event.set()     # wake all _isleep() calls
            # Keep hammering _force_stop every 1ms while estop is active
            # This ensures no other thread can restart motors
        time.sleep(0.001)          # 1ms — 1000Hz watchdog

_estop_wdg = threading.Thread(target=_estop_watchdog, daemon=True, name='estop_wdg')
_estop_wdg.start()

# Raise watchdog thread priority (Linux only — best effort)
try:
    import ctypes
    _SCHED_FIFO = 1
    _param = ctypes.c_int(99)  # max priority
    _pthread_lib = ctypes.CDLL('libpthread.so.0', use_errno=True)
    _pthread_lib.pthread_setschedparam(
        ctypes.c_ulong(_estop_wdg.ident),
        _SCHED_FIFO, ctypes.byref(_param)
    )
    print("✅ E-Stop watchdog: SCHED_FIFO priority 99")
except Exception as e:
    print(f"⚠ Thread priority (non-root, ok): {e}")

# ================================================================
# HARDWARE ESTOP BUTTON CALLBACK
# ================================================================
def _hw_estop_pressed():
    global emergency_stop
    _force_stop()            # ABSOLUTE FIRST — kills GPIO before Python state
    emergency_stop = True    # flag — watchdog picks this up within 1ms
    _estop_event.set()       # wake all _isleep()
    _kill_all()              # clean up relays
    print("🚨 HARDWARE E-STOP ACTIVATED")

if estop_button:
    estop_button.when_pressed = _hw_estop_pressed

# ================================================================
# INTERRUPTIBLE SLEEP — for auto-clean sequences
# Returns True if completed, False if interrupted by estop
# ================================================================
def _isleep(seconds):
    """Estop-interruptible sleep. Checks every 5ms max."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if emergency_stop:
            return False
        remaining = deadline - time.monotonic()
        _estop_event.wait(timeout=min(0.005, remaining))
        # Note: do NOT clear _estop_event here — other threads may be waiting
    return True

# ================================================================
# CAMERA — ZERO-LAG DUAL THREAD
# Thread 1: _capture_loop — grab() at full FPS (no decode)
# Thread 2: _vision_loop  — retrieve() + process latest only
# Stream:   reads _ann_frame — pure MJPEG serve, no processing
# ================================================================
def init_camera():
    global camera, camera_ok
    for idx in [0, 1, 2]:
        try:
            cam = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            cam.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
            cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cam.set(cv2.CAP_PROP_FPS,          30)
            cam.set(cv2.CAP_PROP_BUFFERSIZE,   1)   # KEY: discard old frames
            time.sleep(0.6)
            if cam.isOpened():
                ok, _ = cam.read()
                if ok:
                    camera = cam; camera_ok = True
                    print(f"✅ Camera idx={idx}")
                    return
            cam.release()
        except Exception as e:
            print(f"⚠ cam{idx}: {e}")
    print("❌ No camera found")

def _capture_loop():
    """Thread 1: grab at full camera FPS, never decode. Always newest frame."""
    global _raw_frame
    while True:
        try:
            if not camera_ok or camera is None:
                time.sleep(0.05); continue
            grabbed = camera.grab()       # <1ms — no decode
            if not grabbed:
                time.sleep(0.005); continue
            ret, frame = camera.retrieve()   # decode only after grab
            if ret and frame is not None:
                with _raw_lock:
                    _raw_frame = frame    # atomic replace
                _cap_event.set()
        except Exception as e:
            print(f"capture err: {e}")
            time.sleep(0.1)  # brief pause before retry

threading.Thread(target=_capture_loop, daemon=True, name='capture').start()

# ================================================================
# EDGE DETECTION — 4-METHOD (vision data for auto-clean only)
# ================================================================
def _detect_edges(frame):
    """Returns (dists, confs) dicts. All 99/0 on failure."""
    empty = {'front':99,'back':99,'left':99,'right':99}
    zero  = {'front':0, 'back':0, 'left':0, 'right':0}
    if frame is None: return empty, zero
    try:
        fh, fw = frame.shape[:2]
        gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur   = cv2.GaussianBlur(gray, (5,5), 0)

        # Canny triple-threshold
        e1 = cv2.Canny(blur, 30, 90)
        e2 = cv2.Canny(blur, 60, 180)
        e3 = cv2.Canny(blur, 100, 250)
        edges = cv2.dilate(cv2.bitwise_or(e1, cv2.bitwise_or(e2, e3)),
                           np.ones((3,3), np.uint8), iterations=1)

        # HSV panel mask
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        md   = cv2.inRange(hsv, np.array([0,0,0]),     np.array([180,255,90]))
        mb   = cv2.inRange(hsv, np.array([85,20,20]),  np.array([135,255,200]))
        pmsk = cv2.morphologyEx(cv2.bitwise_or(md, mb),
                                cv2.MORPH_CLOSE, np.ones((3,3),np.uint8), iterations=2)

        # Sobel gradient
        sx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        gm = np.sqrt(sx**2 + sy**2)
        mx = gm.max()
        gn = np.uint8(255*gm/mx) if mx > 0 else gm.astype(np.uint8)
        _, gt = cv2.threshold(gn, 50, 255, cv2.THRESH_BINARY)

        # Contour bbox
        cnts, _ = cv2.findContours(pmsk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bbox = None
        if cnts:
            lg = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(lg) > fw*fh*0.15:
                bbox = cv2.boundingRect(lg)

        def _analyse(re, rm, rg, bd):
            ed = np.count_nonzero(re) / re.size
            pr = np.count_nonzero(rm) / rm.size
            gd = np.count_nonzero(rg) / rg.size
            if ed > 0.08 or gd > 0.08:
                cs = (ed*0.5 + gd*0.5)
                return max(1, int(99*(1-cs*4))), min(100, int(cs*800))
            elif pr < 0.4:
                return max(1, int(50*pr)), min(100, int((1-pr)*120))
            elif bd is not None:
                return bd, 75
            return 99, 0

        zh = int(fh*0.20); zw = int(fw*0.20)
        regions = {
            'front': (edges[fh-zh:,:], pmsk[fh-zh:,:], gt[fh-zh:,:],
                      max(1,(fh-(bbox[1]+bbox[3]))//2) if bbox else None),
            'back':  (edges[:zh,:],    pmsk[:zh,:],    gt[:zh,:],
                      max(1, bbox[1]//2) if bbox else None),
            'left':  (edges[:,:zw],    pmsk[:,:zw],    gt[:,:zw],
                      max(1, bbox[0]//2) if bbox else None),
            'right': (edges[:,fw-zw:], pmsk[:,fw-zw:], gt[:,fw-zw:],
                      max(1,(fw-(bbox[0]+bbox[2]))//2) if bbox else None),
        }
        dists = {}; confs = {}
        for d, (re, rm, rg, bd) in regions.items():
            dist, conf = _analyse(re, rm, rg, bd)
            _edge_buf[d].append(dist)
            dists[d] = int(np.median(_edge_buf[d]))
            confs[d] = conf
        return dists, confs

    except Exception as e:
        print(f"edge_detect err: {e}")
        return empty, zero

# ================================================================
# VISION THREAD — processes latest frame, updates HUD + edge data
# ================================================================
def _vision_loop():
    global _ann_frame, edge_distances, barrier_detected, edge_confidence
    global panel_dirt_level, damage_count
    last_ptr = None
    while True:
        _cap_event.wait(timeout=0.1)
        _cap_event.clear()
        with _raw_lock:
            frame = _raw_frame
        if frame is None or frame is last_ptr:
            continue
        last_ptr = frame
        ann = frame.copy()

        # Always run edge detection — auto-clean reads these globals
        dists, confs = _detect_edges(frame)
        edge_distances.update(dists)
        edge_confidence.update(confs)
        for d in ['front','back','left','right']:
            barrier_detected[d] = (dists[d] < EDGE_CRITICAL and confs[d] >= MIN_CONF)

        fh, fw = frame.shape[:2]

        # Draw edge zones on HUD
        zmap = {
            'front': (int(fw*0.25), int(fh*0.75), int(fw*0.75), fh),
            'back':  (int(fw*0.25), 0,             int(fw*0.75), int(fh*0.25)),
            'left':  (0,            int(fh*0.25),  int(fw*0.25), int(fh*0.75)),
            'right': (int(fw*0.75), int(fh*0.25),  fw,           int(fh*0.75)),
        }
        for d, (x1,y1,x2,y2) in zmap.items():
            dist = dists[d]; conf = confs[d]
            col = ((0,0,255) if dist < EDGE_CRITICAL else
                   (0,165,255) if dist < EDGE_WARN else (0,200,100))
            cv2.rectangle(ann, (x1,y1), (x2,y2), col, 2)
            cx, cy = (x1+x2)//2, (y1+y2)//2
            cv2.putText(ann, f"{d[0].upper()}:{dist}", (cx-22, cy-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)
            cv2.putText(ann, f"{conf}%", (cx-14, cy+10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, col, 1)

        # Dirt estimate
        g2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, th2 = cv2.threshold(cv2.GaussianBlur(g2,(5,5),0), 60, 255, cv2.THRESH_BINARY_INV)
        panel_dirt_level = min(int(np.count_nonzero(th2)/th2.size*300), 100)

        # YOLO
        if detection_active and model:
            try:
                for r in model(frame, conf=0.45, verbose=False):
                    for box in r.boxes:
                        x1,y1,x2,y2 = map(int, box.xyxy[0])
                        nm = model.names[int(box.cls[0])]; cf = float(box.conf[0])
                        col = {'damage':(0,0,255),'dropping':(0,200,255),
                               'dirt':(50,255,50),'clean':(100,255,100)}.get(nm,(0,180,255))
                        cv2.rectangle(ann, (x1,y1), (x2,y2), col, 2)
                        cv2.putText(ann, f"{nm} {cf:.0%}", (x1, y1-5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255,255,255), 1)
                        if 'damage' in nm: damage_count += 1
            except: pass

        # Mode label
        if emergency_stop:
            lbl = "E-STOP"; lc = (0,0,255)
        elif auto_cleaning:
            lbl = "AUTO-CLEAN [EDGE PROTECT ON]"; lc = (0,200,255)
        elif pattern_running:
            lbl = "PATTERN RUN"; lc = (0,165,255)
        else:
            lbl = f"MANUAL {movement_mode}"; lc = (180,180,180)

        cv2.putText(ann, f"CleanBot v1.0 | {lbl}", (8,20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, lc, 1)
        cv2.putText(ann, datetime.now().strftime('%H:%M:%S'), (fw-80, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120,120,120), 1)

        # Dirt bar
        bs = 110; be = fw-90
        bl = int((be-bs)*(panel_dirt_level/100))
        bc = (0,255,0) if panel_dirt_level<40 else (0,165,255) if panel_dirt_level<70 else (0,0,255)
        cv2.rectangle(ann, (bs,5), (be,18), (40,40,40), -1)
        if bl > 0: cv2.rectangle(ann, (bs,5), (bs+bl,18), bc, -1)
        cv2.putText(ann, f"DIRT:{panel_dirt_level}%", (8,35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180,180,180), 1)

        # Actuator status
        if brush_active:
            cv2.putText(ann, "BRUSH", (fw-180, fh-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100,255,100), 1)
        if water_active:
            cv2.putText(ann, "WATER", (fw-100, fh-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255,200,0), 1)
        if emergency_stop:
            cv2.putText(ann, "!!! E-STOP !!!", (fw//2-80, fh//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,0,255), 3)

        with _ann_lock:
            _ann_frame = ann

threading.Thread(target=_vision_loop, daemon=True, name='vision').start()

# ================================================================
# MJPEG STREAM — reads _ann_frame only, zero processing
# ================================================================
def _gen_frames():
    while True:
        with _ann_lock:
            frame = _ann_frame
        if frame is None:
            ph = np.zeros((480,640,3), dtype=np.uint8)
            cv2.putText(ph, "Waiting for camera...", (150,240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80,80,80), 2)
            frame = ph
        ret, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ret:
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n'
        time.sleep(0.033)  # 30fps cap

# ================================================================
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# MANUAL MOVEMENT EXECUTOR — 200Hz, ZERO EDGE PROTECTION
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Rules:
#   • NEVER checks edge_distances — that is AUTO-CLEAN only
#   • ONLY blocked by emergency_stop (watchdog handles it anyway)
#   • Stands down when auto_cleaning or pattern_running
#   • 5ms cycle = 200Hz — fastest possible in Python threading
# ================================================================
def _movement_loop():
    global movement_mode
    while True:
        # E-Stop: watchdog already killed GPIO. Just clear mode.
        if emergency_stop:
            with movement_lock:
                movement_mode = 'STOP'
            time.sleep(0.005)
            continue

        # Hands off to auto/pattern — they drive motors directly
        if auto_cleaning or pattern_running:
            time.sleep(0.005)
            continue

        # Read mode under lock (lock is held for < 1µs)
        with movement_lock:
            mode = movement_mode

        # ── ZERO DELAY — direct motor command, NO edge checks ──
        try:
            if   mode == 'FORWARD':  robot_forward(current_speed)
            elif mode == 'BACKWARD': robot_backward(current_speed)
            elif mode == 'LEFT':     robot_turn_left(current_speed)
            elif mode == 'RIGHT':    robot_turn_right(current_speed)
            else:                    _force_stop()   # STOP
        except Exception as e:
            print(f"motor cmd err: {e}")
            _force_stop()  # safe fallback

        time.sleep(0.005)   # 200Hz = 5ms

threading.Thread(target=_movement_loop, daemon=True, name='mover').start()

# ================================================================
# SQLITE — SESSION HISTORY
# ================================================================
DB = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cleanbot.db')

def _init_db():
    c = sqlite3.connect(DB); cur = c.cursor()
    cur.execute('''CREATE TABLE IF NOT EXISTS sessions(
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        date TEXT, start_time TEXT, duration_sec INTEGER,
        area_sqm REAL, dirt_removed INTEGER, damage_count INTEGER)''')
    cur.execute('''CREATE TABLE IF NOT EXISTS patterns(
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT, data TEXT, created_at TEXT)''')
    c.commit(); c.close()
_init_db()

def _save_session(dur, area, dirt, dmg):
    n = datetime.now()
    try:
        c = sqlite3.connect(DB)
        c.execute('INSERT INTO sessions VALUES(NULL,?,?,?,?,?,?)',
                  (n.strftime('%Y-%m-%d'), n.strftime('%H:%M:%S'),
                   int(dur), round(area,2), int(dirt), int(dmg)))
        c.commit(); c.close()
    except Exception as e:
        print(f"DB save err: {e}")

def _get_history():
    try:
        c = sqlite3.connect(DB)
        rows = c.execute('SELECT * FROM sessions ORDER BY id DESC LIMIT 50').fetchall()
        c.close()
        out = []
        for r in rows:
            m, s = r[4]//60, r[4]%60
            out.append({'id':r[0],'date':r[1],'start':r[2],
                        'duration':f"{m}:{s:02d}",'area':r[5],'dirt':r[6],'damage':r[7]})
        return out
    except: return []

# ================================================================
# AUTO-CLEAN — EDGE PROTECTION ON, fully e-stop interruptible
# Every time.sleep replaced with _isleep() → estop kills in <5ms
# ================================================================
def _auto_clean_worker():
    global auto_cleaning, last_clean_time, area_covered, session_start
    auto_cleaning = True
    session_start = time.monotonic()
    area_covered  = 0.0
    print("🤖 AUTO-CLEAN START — edge protection ACTIVE")
    try:
        # Phase 1: Pre-wet
        water_on()
        if not _isleep(2.0): raise InterruptedError
        water_off()
        if not _isleep(0.3): raise InterruptedError

        # Phase 2: Brush on
        brush_on()
        if not _isleep(0.3): raise InterruptedError

        # Phase 3: S-pattern WITH edge protection
        rows = 0; max_rows = 7
        while rows < max_rows and not emergency_stop:

            # Forward
            t0 = time.monotonic()
            while time.monotonic()-t0 < 6.0:
                if emergency_stop: raise InterruptedError
                # EDGE CHECK — only in auto-clean
                if (edge_distances['front'] < EDGE_CRITICAL and
                    edge_confidence['front'] >= MIN_CONF):
                    print(f"  → Front edge at {edge_distances['front']}")
                    break
                robot_forward(SPEED_AUTO)
                time.sleep(0.01)
            _force_stop()
            if not _isleep(0.15): raise InterruptedError
            area_covered = round(min(30.0, rows*1.5+0.5), 2); rows += 1
            if rows >= max_rows: break

            # Shift right
            t0 = time.monotonic()
            while time.monotonic()-t0 < 1.0:
                if emergency_stop: raise InterruptedError
                if (edge_distances['right'] < EDGE_CRITICAL and
                    edge_confidence['right'] >= MIN_CONF):
                    robot_turn_left(SPEED_AUTO)
                    if not _isleep(0.5): raise InterruptedError
                    break
                robot_turn_right(SPEED_AUTO)
                time.sleep(0.01)
            _force_stop()
            if not _isleep(0.15): raise InterruptedError

            # Backward
            t0 = time.monotonic()
            while time.monotonic()-t0 < 6.0:
                if emergency_stop: raise InterruptedError
                # EDGE CHECK — only in auto-clean
                if (edge_distances['back'] < EDGE_CRITICAL and
                    edge_confidence['back'] >= MIN_CONF):
                    print(f"  → Back edge at {edge_distances['back']}")
                    break
                robot_backward(SPEED_AUTO)
                time.sleep(0.01)
            _force_stop()
            if not _isleep(0.15): raise InterruptedError
            area_covered = round(min(30.0, rows*1.5+0.5), 2); rows += 1
            if rows >= max_rows: break

            # Shift right again
            t0 = time.monotonic()
            while time.monotonic()-t0 < 1.0:
                if emergency_stop: raise InterruptedError
                if (edge_distances['right'] < EDGE_CRITICAL and
                    edge_confidence['right'] >= MIN_CONF):
                    robot_turn_left(SPEED_AUTO)
                    if not _isleep(0.5): raise InterruptedError
                    break
                robot_turn_right(SPEED_AUTO)
                time.sleep(0.01)
            _force_stop()
            if not _isleep(0.15): raise InterruptedError

        # Phase 4: Final rinse
        brush_off()
        if not _isleep(0.3): raise InterruptedError
        water_on()
        if not _isleep(2.0): raise InterruptedError
        water_off()

        dur = time.monotonic() - session_start
        _save_session(dur, area_covered, panel_dirt_level, damage_count)
        print("✅ AUTO-CLEAN COMPLETE")

    except InterruptedError:
        print("🚨 AUTO-CLEAN KILLED BY E-STOP")
    except Exception as e:
        print(f"autoclean err: {e}")
    finally:
        _force_stop()
        brush_off(); water_off()
        auto_cleaning    = False
        last_clean_time  = time.strftime('%H:%M:%S')
        print("🏁 AUTO-CLEAN ENDED")

# ================================================================
# PATTERN PLAYER — NO edge protection, fully interruptible
# Operator-defined path = operator responsibility for safety
# ================================================================
def _pattern_worker(path_data):
    global pattern_running
    pattern_running = True
    print(f"▶ PATTERN: {len(path_data)} pts — NO edge protection (operator path)")
    try:
        for i in range(1, len(path_data)):
            if emergency_stop or not pattern_running: break

            dx = path_data[i]['x'] - path_data[i-1]['x']
            dy = path_data[i]['y'] - path_data[i-1]['y']
            if abs(dx) < 2 and abs(dy) < 2: continue

            dur = max(0.1, min(1.5, math.hypot(dx,dy)/30))
            act = ('forward'  if dy < 0 else
                   'backward' if dy > 0 else
                   'left'     if dx < 0 else 'right')
            {'forward':  robot_forward,
             'backward': robot_backward,
             'left':     robot_turn_left,
             'right':    robot_turn_right}[act](current_speed)

            # Interruptible segment — checks estop every 10ms
            deadline = time.monotonic() + dur
            while time.monotonic() < deadline:
                if emergency_stop or not pattern_running:
                    _force_stop(); return
                time.sleep(0.01)

            _force_stop()
            # 50ms gap — interruptible
            gap = time.monotonic() + 0.05
            while time.monotonic() < gap:
                if emergency_stop: return
                time.sleep(0.01)

    except Exception as e:
        print(f"pattern err: {e}")
    finally:
        _force_stop()
        pattern_running = False
        print("✅ Pattern done")

# ================================================================
# WEBSOCKET STATS BROADCASTER — 5Hz
# ================================================================
def _stats_loop():
    global water_level, robot_x, robot_y
    t = 0
    while True:
        t += 1
        now = int(time.time()*1000)

        # Simulated battery
        bv = round(12.4 + math.sin(time.monotonic()/60)*0.3, 2)
        bp = max(0, min(100, int((bv-10.5)/(13.2-10.5)*100)))

        # Water drain during clean
        if auto_cleaning:
            water_level = max(0.0, water_level - 0.015)

        # Robot position sim
        if auto_cleaning:
            robot_x = min(95.0, max(5.0, robot_x + (0.5 if t%30<15 else -0.5)))
            robot_y = min(95.0, max(5.0, robot_y + 0.15))
        elif pattern_running:
            robot_x = min(95.0, max(5.0, robot_x + (0.3 if t%20<10 else -0.3)))
        if auto_cleaning or pattern_running:
            path_history.append({'x': round(robot_x,1), 'y': round(robot_y,1)})

        # Chart buffers
        dirt_timeline.append({'t': now, 'v': panel_dirt_level})
        eff_timeline.append({'t': now,  'v': round(area_covered, 2)})
        speed_timeline.append({'t': now, 'v': current_speed})

        # Session timer
        ss = (int(time.monotonic()-session_start)
              if session_start and (auto_cleaning or pattern_running) else 0)

        # WiFi signal
        sig_pct = 75
        try:
            for ln in open('/proc/net/wireless').read().split('\n'):
                if 'wlan' in ln:
                    parts = ln.split()
                    if len(parts) > 3:
                        sig = int(float(parts[3].rstrip('.')))
                        sig_pct = max(0, min(100, int((sig+100)/70*100)))
                    break
        except: pass

        # Edge protection mode label
        edge_mode = 'AUTO-CLEAN' if auto_cleaning else 'OFF'

        payload = {
            'time': datetime.now().strftime('%H:%M:%S'),
            'date': datetime.now().strftime('%Y-%m-%d'),
            'battery_v': bv, 'battery_pct': bp,
            'water_pct': round(water_level, 1),
            'signal': sig_pct,
            'area': area_covered,
            'dirt': panel_dirt_level,
            'damage': damage_count,
            'session_sec': ss,
            'cleaning': auto_cleaning,
            'pattern': pattern_running,
            'speed': current_speed,
            'auto_clean': auto_cleaning,
            'emergency': emergency_stop,
            'brush': brush_active,
            'water': water_active,
            'movement_mode': movement_mode,
            'edge_mode': edge_mode,
            'edges': edge_distances,
            'confidence': edge_confidence,
            'barrier': barrier_detected,
            'robot_pos': {'x': robot_x, 'y': robot_y},
            'path': list(path_history)[-50:],
            'dirt_chart':  list(dirt_timeline)[-30:],
            'eff_chart':   list(eff_timeline)[-30:],
            'speed_chart': list(speed_timeline)[-30:],
            'gpio_ok':    gpio_ok,
            'camera_ok':  camera_ok,
            'model_ok':   model is not None,
        }
        socketio.emit('stats', payload)
        time.sleep(0.2)   # 5Hz

threading.Thread(target=_stats_loop, daemon=True, name='stats').start()

# ================================================================
# FLASK ROUTES
# ================================================================
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(_gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# ── /control — MOVEMENT ONLY (triple-press forward/backward) ──
@app.route('/control', methods=['POST'])
def control():
    global movement_mode, forward_press_count, backward_press_count
    global last_fwd_press, last_bwd_press
    if emergency_stop:
        return jsonify({'status':'error','msg':'estop'}), 400
    data = request.get_json(force=True)
    cmd  = data.get('command','')
    now  = time.monotonic()
    with movement_lock:
        if cmd == 'forward':
            if now - last_fwd_press < TRIPLE_WINDOW:
                forward_press_count += 1
            else:
                forward_press_count = 1
            last_fwd_press  = now
            movement_mode   = 'FORWARD'
            if forward_press_count >= 3:
                forward_press_count = 0   # continuous — JS won't send stop
        elif cmd == 'backward':
            if now - last_bwd_press < TRIPLE_WINDOW:
                backward_press_count += 1
            else:
                backward_press_count = 1
            last_bwd_press  = now
            movement_mode   = 'BACKWARD'
            if backward_press_count >= 3:
                backward_press_count = 0
        elif cmd == 'left':   movement_mode = 'LEFT'
        elif cmd == 'right':  movement_mode = 'RIGHT'
        elif cmd == 'stop':
            movement_mode        = 'STOP'
            forward_press_count  = 0
            backward_press_count = 0
            _force_stop()    # immediate GPIO kill on explicit STOP
        else:
            return jsonify({'status':'error','msg':f'unknown:{cmd}'}), 400
    return jsonify({'status':'ok','mode':movement_mode,'fwd_n':forward_press_count})

# ── /cmd — all non-movement commands ──
@app.route('/cmd', methods=['POST'])
def cmd_route():
    global movement_mode, current_speed, water_level, auto_cleaning
    global pattern_running, emergency_stop, damage_count, session_start
    global area_covered, robot_x, robot_y, detection_active
    data    = request.get_json(force=True)
    command = data.get('cmd','')

    if emergency_stop and command not in ('estop_reset',):
        return jsonify({'ok':False,'msg':'estop active'})

    if   command == 'brush_on':   brush_on()
    elif command == 'brush_off':  brush_off()
    elif command == 'water_on':   water_on()
    elif command == 'water_off':  water_off()
    elif command == 'water_refill': water_level = 100.0
    elif command == 'auto_start':
        if not auto_cleaning and not emergency_stop:
            threading.Thread(target=_auto_clean_worker, daemon=True).start()
    elif command == 'auto_stop':
        auto_cleaning = False
        _force_stop()
        with movement_lock: movement_mode = 'STOP'
    elif command == 'pattern_play':
        pts = data.get('pattern', [])
        if pts and not pattern_running and not emergency_stop:
            threading.Thread(target=_pattern_worker, args=(pts,), daemon=True).start()
    elif command == 'pattern_stop':
        pattern_running = False
        _force_stop()
    elif command == 'estop':
        emergency_stop = True
        _estop_event.set()
        _kill_all()
        with movement_lock: movement_mode = 'STOP'
        print("🚨 SOFTWARE E-STOP")
    elif command == 'estop_reset':
        emergency_stop = False
        _estop_event.clear()
        print("✅ E-STOP RESET")
    elif command == 'speed':
        current_speed = max(20, min(100, int(data.get('value', 75))))
    elif command == 'detection_on':
        detection_active = True
    elif command == 'detection_off':
        detection_active = False
    elif command == 'reset_session':
        session_start = None; area_covered = 0.0
        damage_count = 0; path_history.clear()
        robot_x = 50.0; robot_y = 50.0
    return jsonify({'ok': True})

@app.route('/brush', methods=['POST'])
def brush_ctrl():
    if emergency_stop: return jsonify({'status':'error'}), 400
    c = request.get_json(force=True).get('command','')
    if c == 'on': brush_on()
    elif c == 'off': brush_off()
    return jsonify({'status':'ok','brush':brush_active})

@app.route('/water', methods=['POST'])
def water_ctrl():
    if emergency_stop: return jsonify({'status':'error'}), 400
    c = request.get_json(force=True).get('command','')
    if c == 'on': water_on()
    elif c == 'off': water_off()
    return jsonify({'status':'ok','water':water_active})

@app.route('/autoclean', methods=['POST'])
def autoclean_route():
    if emergency_stop or auto_cleaning: return jsonify({'status':'error'}), 400
    threading.Thread(target=_auto_clean_worker, daemon=True).start()
    return jsonify({'status':'ok'})

@app.route('/reset_emergency', methods=['POST'])
def reset_emergency():
    global emergency_stop
    emergency_stop = False
    _estop_event.clear()
    return jsonify({'status':'ok'})

@app.route('/detection', methods=['POST'])
def det_ctrl():
    global detection_active
    detection_active = bool(request.get_json(force=True).get('enabled', False))
    return jsonify({'status':'ok','detection_active':detection_active})

@app.route('/status')
def status():
    return jsonify({
        'gpio_ok':gpio_ok,'camera_ok':camera_ok,'model_ok':model is not None,
        'emergency':emergency_stop,'movement_mode':movement_mode,
        'brush':brush_active,'water':water_active,
        'auto_cleaning':auto_cleaning,'pattern_running':pattern_running,
        'speed':current_speed,
        'edge_distances':edge_distances,'edge_confidence':edge_confidence,
        'barrier':barrier_detected,'dirt':panel_dirt_level,
        'damage':damage_count,'last_clean':last_clean_time or 'Never',
        'edge_protection': 'AUTO-CLEAN ONLY',
    })

@app.route('/history')
def history():
    return jsonify(_get_history())

@app.route('/history/csv')
def history_csv():
    rows = _get_history()
    out  = io.StringIO()
    w    = csv.writer(out)
    w.writerow(['ID','Date','Start','Duration','Area(m2)','Dirt%','Damage'])
    for r in rows:
        w.writerow([r['id'],r['date'],r['start'],r['duration'],r['area'],r['dirt'],r['damage']])
    out.seek(0)
    return send_file(io.BytesIO(out.getvalue().encode()), mimetype='text/csv',
                     as_attachment=True,
                     download_name=f"cleanbot_{datetime.now().strftime('%Y%m%d')}.csv")

@app.route('/patterns', methods=['GET','POST','DELETE'])
def patterns_route():
    if request.method == 'GET':
        c = sqlite3.connect(DB)
        rows = [{'id':r[0],'name':r[1],'data':json.loads(r[2]),'created_at':r[3]}
                for r in c.execute('SELECT id,name,data,created_at FROM patterns ORDER BY id DESC').fetchall()]
        c.close(); return jsonify(rows)
    elif request.method == 'POST':
        d = request.json; c = sqlite3.connect(DB)
        cur = c.execute('INSERT INTO patterns VALUES(NULL,?,?,?)',
                        (d.get('name','Pattern'), json.dumps(d.get('data',[])),
                         datetime.now().strftime('%Y-%m-%d %H:%M')))
        pid = cur.lastrowid; c.commit(); c.close()
        return jsonify({'ok':True,'id':pid})
    elif request.method == 'DELETE':
        body = request.get_json(force=True, silent=True) or {}
        pid = body.get('id'); c = sqlite3.connect(DB)
        if pid: c.execute('DELETE FROM patterns WHERE id=?', (pid,)); c.commit()
        c.close(); return jsonify({'ok':True})

# ================================================================
# SHUTDOWN
# ================================================================
def _shutdown():
    print("Shutting down CleanBot v1.0...")
    _kill_all()
    try:
        if gpio_ok and h: lgpio.gpiochip_close(h)
    except: pass
    if camera: camera.release()
    print("✅ Shutdown complete")
atexit.register(_shutdown)

# ================================================================
# MAIN
# ================================================================
if __name__ == '__main__':
    print('\n' + '='*65)
    print('  CleanBot v1.0 INDUSTRIAL — Zero Delay · Hardware E-Stop')
    print(f'  Brush=GPIO{BRUSH_PIN}  Water=GPIO{WATER_PIN}  EStop=GPIO{ESTOP_PIN}')
    print('  BTS7960#1: RPWM=GPIO18 LPWM=GPIO23 EN=GPIO24')
    print('  BTS7960#2: RPWM=GPIO19 LPWM=GPIO20 EN=GPIO21')
    print('  Edge protection: AUTO-CLEAN ONLY')
    print('  Manual & Pattern: FULL SPEED, NO protection')
    print('  E-Stop watchdog: 1ms poll, SCHED_FIFO priority')
    print('='*65)

    init_camera()

    try:
        if os.path.exists(MODEL_PATH) and _yolo_avail:
            model = YOLO(MODEL_PATH)
            print(f"✅ YOLO: {MODEL_PATH}")
        elif _yolo_avail:
            model = YOLO('yolov8n.pt')
            print("✅ YOLO: yolov8n.pt")
        else:
            model = None
            print("⚠ YOLO not available")
    except Exception as e:
        print(f"❌ YOLO: {e}"); model = None

    print(f"\n🌐  http://0.0.0.0:5000\n" + '='*65)

    socketio.run(app, host='0.0.0.0', port=5000,
                 debug=False, use_reloader=False,
                 allow_unsafe_werkzeug=True)
