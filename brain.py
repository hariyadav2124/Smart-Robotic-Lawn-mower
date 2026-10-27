"""
GRASSGUARD AI — PI-BRAIN (d5pi 1.3 — ZIGZAG-GPS-MOW + NARZO-AUTO-FIND + BACK-5cm)
  ✅ Cleaned & Optimized Production Evaluation:
     1. ZIGZAG MOW: dashboard se 4 CORNER set karo (Narzo haath me) -> START ->
        robot 4 konon ke beech ZIGZAG line-kati karta hai (spacing set karke)!
     2. Rukawat aaye -> wahi 2s-SOCH + side-dekh ke avoid -> WAPAS usi line ke
        waypoint pe wapas (straight path follow karta hai)
     3. BACK FIX: peechhe ab 5cm tak allowed tha (20 nahi!) — back le ke
        ghoom jaata hai
     4. NARZO AUTO-FIND: Narzo ka IP badal jaye (Poco-hotspot pe) to Pi khud
        network scan kar ke beacon dhoondh leta hai — koi edit nahi!
     5. YOLO 80-cheezein detect (person/dog/bottle/chair...) + 4 US radars
  ✅ v1.2 se: TAGDA-UI (radar/gauge/saw/E-STOP), login, ramp+arc-motors (v4)
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import hashlib
import math
import os
import re
import socket
import threading
import time

import cv2
from flask import Flask, make_response, redirect, request, Response
import requests
import serial
from ultralytics import YOLO

try:
    from tts_speaker import speak as tts_speak
except Exception:
    def tts_speak(key, force=False):
        pass

try:
    import torch
    torch.set_num_threads(4)
except Exception:
    pass

# ---------------- TUNING ----------------
HARD_CM = 40.0          # 40cm tak aaye -> RUK ke SOCH
SOFTF_CM = 30           # YOLO-detect ke waqt CAM-US limit (30cm) -> ruk ke soch
SIDE_NEAR_CM = 40.0     # L/R 40cm tak -> turn
OBJ_STOP_CM = 25        # Non-living object distance threshold
CAM_OBJ_SEC = 2.0       # Time to track object in center before action
AVOID_MARGIN = 20.0     # L/R bypass kam-se-kam itni doori se (chipka-ke nahi!)
BLADE_CLEAR_SEC = 0.1   # 2.5s khali hona chahiye (Toggle fix)
CONF = 0.5
TURN_STEP = 0.45
TURN_ARC_S = 0.55
TURN_SWAP = False
TILT_DEG = 45.0         # 30 degree tilt threshold (offset ke baad)
TILT_PIT_OFFSET = 5.8   # Resting PIT baseline
TILT_ROL_OFFSET = 16.1  # Resting ROL baseline
OBST_WAIT_SEC = 2.0
BACK_REACT_CM = 25.0    # 25cm safe

# v1.3.1: HAR JAGAH MAX SPEED (user order!)
SPDL = {'1': 200, '2': 220, '3': 235, '4': 245, '5': 255,
        '6': 255, '7': 255, '8': 255, '9': 255, 'V': 220}
LIVING = {'person', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
          'elephant', 'bear', 'zebra', 'giraffe'}

# ---------- PAKKE PORTS + NARZO + CAMERA ----------
USB_CANDIDATES = ('/dev/ttyUSB0', '/dev/ttyUSB1')
NARZO_URL = 'http://10.150.248.88:5001/gps'
CAM_FIRST = 'picam'
COLOR_SWAP = False

# ---------- LOGIN ----------
AUTH_USER = 'shubham27'
AUTH_PASS = '270308'
AUTH_TOKEN = hashlib.sha256(('gg1:' + AUTH_USER + ':' + AUTH_PASS + ':pi-brain').encode()).hexdigest()[:32]
COOKIE = 'ggauth'

app = Flask(__name__)
START_T = time.time()

S = {'F': '-', 'L': '-', 'R': '-', 'B': '-', 'RAIN': '-', 'MPU': '-', 'ACS': '-',
     'PIT': '-', 'ROL': '-', 'C': '-',
     'esp_age': 999, 'ard': 'dhundh raha hu...', 'esp_raw': '...',
     'mode': 'stop', 'state': 'READY', 'blade': 'CUT', 'spd': '-',
     'det_name': '-', 'det_conf': 0, 'det_zone': '-', 'det_total': 0, 'vfps': 0,
     'cam_src': 'khul raha...', 'rep': '(ready)'}
LOG = deque(maxlen=14)
VJPG = None
ser_lock = threading.Lock()
esp_lock = threading.Lock()
MODE = 'stop'
LAST_CMD = time.time()
ard = None
ard_path = None
esp = None
esp_path = None

# ---------------- MOW STATE (zigzag) ----------------
MOWW = {'corners': {}, 'wps': [], 'wi': 0, 'spacing': 1.0,
        'grid_n': 5, 'grid_ok': 0, 'grid_bad': 0}
GPS_HIST = []

# ---------------- TELEMETRY & GPS STATE ----------------
T = {'f': 999.0, 'l': 999.0, 'r': 999.0, 'b': 999.0, 'rain': 0, 'pit': 0.0,
     'rol': 0.0, 'acs': 0.0, 'age': 999.0, 'c': 999.0, 'last_rx': 0.0}
F_PIT = TILT_PIT_OFFSET
F_ROL = TILT_ROL_OFFSET
ARD_ST = {'age': 0.0}   # CAM-US (Arduino '?') ka fresh-hone-timestamp
G = {'lat': '-', 'lon': '-', 'acc': '-', 'raw': 'shuru ho raha...', 'last_ok': 0.0}

def is_valid_distance(val):
    if val is None:
        return False
    try:
        f = float(val)
        return not math.isnan(f) and not math.isinf(f) and f >= 0.0
    except Exception:
        return False

def is_tilted():
    return abs(T['pit'] - TILT_PIT_OFFSET) > TILT_DEG or abs(T['rol'] - TILT_ROL_OFFSET) > TILT_DEG

def get_esp_age():
    if T.get('last_rx', 0.0) == 0.0:
        return 999.0
    return round(time.time() - T['last_rx'], 1)

def mow_grid_check():
    # Field boundary guard: checks if robot is within lawn perimeter (+ GPS tolerance)
    c = MOWW['corners']
    if len(c) < 4 or G['last_ok'] == 0:
        return None
    try:
        glat, glon = float(G['lat']), float(G['lon'])
    except Exception:
        return None
    pts = [c[i] for i in (1, 2, 3, 4)]
    clat = sum(p[0] for p in pts) / 4.0
    clon = sum(p[1] for p in pts) / 4.0
    dmax = 0.0
    for p in pts:
        dmax = max(dmax, hav_m((clat, clon), p))
    limit = dmax + 3.0   # 3.0m GPS drift buffer beyond furthest corner
    d = hav_m((glat, glon), (clat, clon))
    return d <= limit, d, limit

def log(msg):
    LOG.appendleft(time.strftime('%H:%M:%S') + '  ' + msg)
    print('[BRAIN]', msg)

def authed():
    return request.cookies.get(COOKIE, '') == AUTH_TOKEN

# ---------------- GPS MATH ----------------
def hav_m(a, b):
    dy = (b[0] - a[0]) * 110540.0
    dx = (b[1] - a[1]) * 111320.0 * math.cos(math.radians(a[0]))
    return math.hypot(dx, dy)

def bearing_deg(a, b):
    dy = (b[0] - a[0]) * 110540.0
    dx = (b[1] - a[1]) * 111320.0 * math.cos(math.radians(a[0]))
    return math.degrees(math.atan2(dx, dy)) % 360.0

def mow_course():
    # chalte-hue GPS-history se direction (0=N, 90=E); 0.4m se kam move = None
    now = time.time()
    if G['last_ok'] == 0:
        return None
    GPS_HIST.append((now, G['lat'], G['lon']))
    while GPS_HIST and now - GPS_HIST[0][0] > 6:
        GPS_HIST.pop(0)
    if len(GPS_HIST) < 2:
        return None
    _, la0, lo0 = GPS_HIST[0]
    _, la1, lo1 = GPS_HIST[-1]
    dy = (la1 - la0) * 110540.0
    dx = (lo1 - lo0) * 111320.0 * math.cos(math.radians(la0))
    if math.hypot(dx, dy) < 0.4:
        return None
    return math.degrees(math.atan2(dx, dy)) % 360.0

def mow_build():
    c = MOWW['corners']
    if len(c) < 4:
        return 0
    A = c[1]; B = c[2]; C = c[3]; D = c[4]
    lat0 = A[0]
    def en(p):
        return ((p[1] - A[1]) * 111320.0 * math.cos(math.radians(lat0)),
                (p[0] - A[0]) * 110540.0)
    ax, ay = en(B)
    L = math.hypot(ax, ay) or 1.0
    ux, uy = ax / L, ay / L           # line-direction (C1->C2)
    px, py = -uy, ux                  # across (C1->C4 taraf)
    cx, cy = en(C); dxx, dyy = en(D)
    Vend = ((cx * px + cy * py) + (dxx * px + dyy * py)) / 2.0
    if Vend < 0:
        px, py = -px, -py
        Vend = -Vend
    sp = MOWW['spacing']
    n = max(1, int(Vend / sp) + 1)
    def to_ll(x, y):
        return (A[0] + y / 110540.0,
                A[1] + x / (111320.0 * math.cos(math.radians(lat0))))
    wps = []
    for i in range(n):
        v = min(i * sp, Vend)
        if i % 2 == 0:
            wps.append(to_ll(ax + px * v, ay + py * v))   # C2-side end
        else:
            wps.append(to_ll(px * v, py * v))             # C1-side end
    MOWW['wps'] = wps
    MOWW['wi'] = 0
    return len(wps)

def mow_poll():
    # (wlat, wlon, dist, line#, total) ya None (field done)
    wps = MOWW['wps']
    tol = max(1.0, MOWW['spacing'] * 0.8)
    try:
        cur = (float(G['lat']), float(G['lon']))
    except Exception:
        return None
    while MOWW['wi'] < len(wps):
        w = wps[MOWW['wi']]
        d = hav_m(cur, w)
        if d <= tol:
            MOWW['wi'] += 1
            log('MOW: waypoint %d/%d pauhanch gaya (%.1fm)' % (MOWW['wi'], len(wps), d))
            continue
        return (w[0], w[1], d, MOWW['wi'] + 1, len(wps))
    return None

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        local_ip = s.getsockname()[0]
    except Exception:
        try:
            local_ip = socket.gethostbyname(socket.gethostname())
        except Exception:
            local_ip = None
    finally:
        s.close()
    return local_ip

def find_narzo():
    # WiFi badalne par Narzo ka IP badal jata hai — subnet scan se dhoondo
    target_ip = get_local_ip()
    if not target_ip or target_ip.startswith('127.'):
        return None
    base = '.'.join(target_ip.split('.')[:3]) + '.'
    found = []
    def probe(i):
        if found:
            return
        u = 'http://' + base + str(i) + ':5001/'
        try:
            r = requests.get(u, timeout=0.6)
            if 'ZINDA' in r.text or 'beacon' in r.text or 'gps' in r.text:
                found.append(base + str(i))
        except Exception:
            pass
    try:
        with ThreadPoolExecutor(max_workers=30) as executor:
            executor.map(probe, range(1, 255))
    except Exception:
        pass
    return found[0] if found else None

# ---------------- SERIAL (handshake pehchan) ----------------
def _probe(path, baud, wait_s, probe_cmd, markers):
    try:
        s = serial.Serial(path, baud, timeout=0.3)
    except Exception as e:
        print('(port', path, 'khula nahi:', str(e)[:60], ')')
        return None, False, ''
    buf = ''
    t0 = time.time()
    next_tx = 0.0   # Arduino port-kholne par RESET hota hai — pehla '?' bootloader me KHO sakta hai
    while time.time() - t0 < wait_s:
        if probe_cmd and time.time() - t0 >= next_tx:
            try:
                s.write(probe_cmd)
            except Exception:
                pass
            next_tx = time.time() - t0 + 0.5
        if s.in_waiting:
            buf += s.read(s.in_waiting).decode(errors='ignore')
            if any(m in buf for m in markers):
                return s, True, buf
        time.sleep(0.05)
    return s, False, buf

def find_arduino():
    for path in USB_CANDIDATES:
        s, ok, _ = _probe(path, 9600, 3.5, b'?', ('ARDUINO', 'STATUS'))
        if ok:
            print('>>> ARDUINO MIL GAYA:', path)
            return s, path
        if s:
            s.close()
            print('(port', path, 'pe Arduino ka jawab nahi)')
    print('X Arduino kisi USB port par nahi mila')
    return None, None

def find_esp32(skip_path):
    for path in USB_CANDIDATES:
        if path == skip_path:
            continue
        s, ok, _ = _probe(path, 115200, 8.0, None, ('T:',))
        if ok:
            print('>>> ESP32 MIL GAYA  :', path)
            return s, path
        if s:
            s.close()
            print('(port', path, 'par ESP32 ki lines nahi aayin)')
    print('X ESP32 kisi USB port par nahi mila')
    return None, None

def radar_mode_cmd():
    return 'C' if MODE == 'stop' else 'A'

def arduino_keeper():
    global ard, ard_path
    while True:
        if ard is None:
            time.sleep(3)
            s2, p2 = find_arduino()
            if s2:
                ard, ard_path = s2, p2
                S['ard'] = 'JUD GAYA (' + str(ard_path) + ')'
                log('ARDUINO wapas mil gaya')
                ard_send('K'); ard_send('S'); ard_send(radar_mode_cmd())
        else:
            time.sleep(2)

def esp_send(c):
    with esp_lock:
        if esp is None:
            return
        try:
            esp.write(c.encode())
        except Exception:
            pass

def esp_loop(s):
    global esp
    while True:
        if s is None:
            time.sleep(5)
            s, _p = find_esp32(ard_path)
            if s is None:
                S['esp_raw'] = 'ESP32 dhundh raha hu...'
                continue
            with esp_lock:
                esp = s
            esp_send(radar_mode_cmd())
        try:
            line = s.readline().decode(errors='ignore').strip()
        except Exception:
            try:
                s.close()
            except Exception:
                pass
            s = None
            with esp_lock:
                esp = None
            S['esp_raw'] = 'ESP32 toot gaya — retry'
            continue
        if line.startswith('T:'):
            now_rx = time.time()
            for m in re.finditer(r'([A-Za-z]+):(-?[0-9.]+)', line):
                tag, val = m.group(1), m.group(2)
                if tag in ('F', 'L', 'R', 'B', 'RAIN', 'MPU', 'PIT', 'ROL', 'ACS'):
                    S[tag] = val
                    k = tag.lower()
                    if k in T:
                        try:
                            T[k] = float(val)
                        except Exception:
                            pass
            T['last_rx'] = now_rx
            T['age'] = 0.0
            S['esp_age'] = 0.0
            S['esp_raw'] = line

# ---------------- CAMERA ----------------
def _open_picam():
    try:
        from picamera2 import Picamera2
        cam2 = Picamera2()
        cfg = cam2.create_preview_configuration(
            main={"format": "RGB888", "size": (1280, 720)})
        cam2.configure(cfg)
        cam2.start()
        _ = cam2.capture_array()
        S['cam_src'] = 'PI CAMERA (picamera2)'
        print('OK: PI CAMERA chalu (picamera2)')
        return ('picam', cam2)
    except Exception as e:
        print('(picamera2 nahi khula:', str(e)[:70], ')')
        return None

def _open_usb():
    for idx in (0, 1):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            ok, fr = cap.read()
            if ok and fr is not None:
                S['cam_src'] = 'USB webcam %d (V4L2)' % idx
                print('OK: USB webcam %d chalu (OpenCV)' % idx)
                return ('cv', cap)
            cap.release()
    return None

def open_camera():
    doosra = 'usb' if CAM_FIRST == 'picam' else 'picam'
    for k in (CAM_FIRST, doosra):
        r = _open_picam() if k == 'picam' else _open_usb()
        if r:
            return r
    S['cam_src'] = 'CAMERA NAHI MILI — US-only mode'
    print('X camera nahi mili (robot chalega — sirf US-sensors par!)')
    return None

print('Camera khul raha hai...')
CAM = open_camera()

def read_frame():
    if CAM is None:
        return False, None
    kind, obj = CAM
    try:
        if kind == 'picam':
            return True, obj.capture_array()
        ok, fr = obj.read()
        return ok, fr
    except Exception:
        return False, None

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'yolov8n.pt')
print('YOLO load ho raha hai...')
yolo = None
try:
    yolo = YOLO(MODEL_PATH)
    print('OK: Aankh (YOLO) teyar!')
except Exception as e:
    print('X YOLO load nahi hua — US-only chalunga:', str(e)[:70])

def zone_of(x, w):
    return 'LEFT' if x < w // 3 else ('RIGHT' if x > 2 * w // 3 else 'CENTER')

def vision_loop():
    global VJPG
    n = 0; t0 = time.time()
    while True:
        try:
            frame_ok, fr = read_frame()
            if not frame_ok or fr is None:
                time.sleep(0.3)
                continue
            fr = cv2.resize(fr, (320, 240))
            if CAM is not None and CAM[0] == 'picam':
                fr = cv2.cvtColor(fr, cv2.COLOR_RGB2BGR)
            elif COLOR_SWAP:
                fr = cv2.cvtColor(fr, cv2.COLOR_RGB2BGR)
            det = None
            if yolo is not None:
                res = yolo.predict(fr, imgsz=640, conf=CONF, verbose=False)[0]
                for c, cf, box in zip(res.boxes.cls.tolist(), res.boxes.conf.tolist(),
                                      res.boxes.xyxy.tolist()):
                    if det is None or cf > det[1]:
                        det = (yolo.names[int(c)], cf, box)
            img = fr
            h, w = img.shape[:2]
            cv2.line(img, (w // 3, 0), (w // 3, h - 1), (255, 200, 0), 1)
            cv2.line(img, (2 * w // 3, 0), (2 * w // 3, h - 1), (255, 200, 0), 1)
            if det:
                bx1, by1, bx2, by2 = map(int, det[2])
                z = zone_of((bx1 + bx2) // 2, w)
                cv2.rectangle(img, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
                cv2.putText(img, '%s %d%% %s' % (det[0], det[1] * 100, z),
                            (bx1, max(16, by1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                S['det_name'], S['det_conf'], S['det_zone'] = det[0], round(det[1], 2), z
                S['det_total'] += 1
            else:
                S['det_name'], S['det_conf'], S['det_zone'] = '-', 0, '-'
            # --- CAMERA ZOOM OUT (Digital Padding) ---
            pad_x = int(w * 0.05) # 5% black bars on left/right
            pad_y = int(h * 0.05) # 5% black bars on top/bottom
            img = cv2.copyMakeBorder(img, pad_y, pad_y, pad_x, pad_x, cv2.BORDER_CONSTANT, value=[0, 0, 0])
            # ----------------------------------------
            ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                VJPG = buf.tobytes()
            n += 1
            if time.time() - t0 >= 1.0:
                S['vfps'] = n; n = 0; t0 = time.time()
        except Exception:
            time.sleep(0.2)

@app.route('/video')
def video():
    if not authed():
        return Response('login pehle', 401)
    def gen():
        while True:
            if VJPG:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + VJPG + b'\r\n')
            time.sleep(0.15)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

# ---------------- CAM-US POLLER (Arduino v5 '?') ----------------
def ard_status_loop():
    global ard
    while True:
        time.sleep(0.5)
        out = ''
        with ser_lock:
            if ard is None:
                continue
            try:
                ard.write(b'?')
                time.sleep(0.12)
                if ard and ard.in_waiting:
                    out = ard.read(ard.in_waiting).decode(errors='ignore')
            except Exception:
                try:
                    ard.close()
                except Exception:
                    pass
                ard = None
                S['ard'] = 'Arduino status toot gaya'
                continue
        m = re.search(r'C:([0-9.]+)', out)
        if m:
            try:
                c_val = float(m.group(1))
                if is_valid_distance(c_val):
                    T['c'] = c_val
                    S['C'] = m.group(1)
                    ARD_ST['age'] = time.time()
            except Exception:
                pass

# ---------------- GPS (Narzo beacon + AUTO-FIND) ----------------
def gps_loop():
    global NARZO_URL
    fails = 0
    while True:
        try:
            r = requests.get(NARZO_URL, timeout=2.5)
            j = r.json()
            if 'latitude' in j and 'longitude' in j:
                lat_f = float(j['latitude'])
                lon_f = float(j['longitude'])
                if -90.0 <= lat_f <= 90.0 and -180.0 <= lon_f <= 180.0:
                    G['lat'] = round(lat_f, 6)
                    G['lon'] = round(lon_f, 6)
                    G['acc'] = round(float(j.get('accuracy', -1)), 1)
                    G['raw'] = 'NARZO LIVE'
                    G['last_ok'] = time.time()
                    fails = 0
                else:
                    G['raw'] = 'NARZO invalid coordinates'
                    fails += 1
            else:
                G['raw'] = 'NARZO ne location nahi bheji'
                fails += 1
        except Exception as e:
            G['raw'] = 'NARZO GAYAB: ' + str(e)[:30]
            fails += 1
        if fails >= 3:
            log('NARZO gayab — network scan (auto-find)...')
            ipn = find_narzo()
            if ipn:
                NARZO_URL = 'http://' + ipn + ':5001/gps'
                log('NARZO MIL GAYA: ' + NARZO_URL)
            else:
                G['raw'] = 'NARZO scan me nahi mila'
                time.sleep(2.0)
            fails = 0
        time.sleep(1.0)

# ---------------- ARDUINO SEND ----------------
def _swap(c):
    if TURN_SWAP:
        if c == 'L': return 'R'
        if c == 'R': return 'L'
        if c == 'G': return 'H'
        if c == 'H': return 'G'
    return c

def ard_send(c, wait=0.2):
    global LAST_CMD, ard
    c = _swap(c)
    if c in '123456789V': c = '5'  # FORCE MAX SPEED EVERYWHERE
    LAST_CMD = time.time()
    with ser_lock:
        if ard is None:
            return 'ARDUINO GAYAB'
        try:
            ard.write(c.encode())
        except Exception:
            try:
                ard.close()
            except Exception:
                pass
            ard = None
            S['ard'] = 'Arduino se baat toot gayi'
            return 'Arduino se baat toot gayi'
        time.sleep(wait)
        out = ''
        try:
            if ard and ard.in_waiting:
                out = ard.read(ard.in_waiting).decode(errors='ignore').strip()
        except Exception:
            pass
    return out

def back_blocked():
    if not is_valid_distance(T.get('b')):
        return True   # Invalid or missing back reading defaults to blocked (safe!)
    return float(T['b']) < BACK_REACT_CM

def brain_send(c, wait=0.15, force=False):
    global LAST_CMD, ard
    c = _swap(c)
    if c in '123456789V': c = '5'  # FORCE MAX SPEED EVERYWHERE
    # GUARD: BACK command sirf tab jab B >= 25cm
    if c == 'B' and not force and back_blocked():
        b_val = float(T['b']) if is_valid_distance(T['b']) else 999.0
        log('BACK BLOCKED: B:%.0fcm < 25cm — nahi jaaunga' % b_val)
        return
    LAST_CMD = time.time()
    with ser_lock:
        if ard is None:
            return
        try:
            ard.write(c.encode())
        except Exception:
            try:
                ard.close()
            except Exception:
                pass
            ard = None
            S['ard'] = 'Arduino toot gaya'
            return
        time.sleep(wait)
        try:
            if ard and ard.in_waiting:
                ard.read(ard.in_waiting)
        except Exception:
            pass

# ---------------- BRAIN KA DIL ----------------
def _brain_inner():
    global MODE
    state = ''
    last_speed = ''
    cam_obj, cam_since = '', 0.0
    beat = 0.0
    clear_t = 0.0
    tilt_confirm_t = 0.0
    acs_bad = 0
    blade_t = 0.0
    person_close_at = 0.0
    flow_last_close = 0.0
    flow_t = 0.0
    hooter_t = 0.0
    stuck_t = 0.0
    side_l_t = 0.0
    side_r_t = 0.0
    triple_t = 0.0
    acs_lock_until = 0.0
    soch_t = 0.0
    soch_cnt = 0

    while True:
        if MODE not in ('auto', 'mow'):
            time.sleep(0.2)
            continue
        if ard is None:
            S['state'] = 'ARDUINO GAYAB — retry chal raha'
            time.sleep(1.0)
            continue

        # Real-time sensor age check
        esp_age = get_esp_age()
        T['age'] = esp_age
        S['esp_age'] = esp_age

        now = time.time()
        # ---- 1. SAFETY (STALE / RAIN / TILT) ----
        if esp_age > 2.0:
            if state != 'STALE':
                log('SENSORS STALE (%.1fs) — STOP & BLADE CUT' % esp_age)
                state = 'STALE'
            S['state'] = 'STALE STOP'
            brain_send('K'); S['blade'] = 'CUT'; brain_send('S')
            time.sleep(0.4); continue

        if T['rain'] == 1:
            if state != 'RAIN':
                log('BARISH — STOP & BLADE CUT (rain)')
                state = 'RAIN'
            S['state'] = 'RAIN STOP'
            brain_send('K'); S['blade'] = 'CUT'; brain_send('S')
            time.sleep(0.4); continue

        tilt_now = is_tilted()
        if tilt_now:
            if tilt_confirm_t == 0.0:
                tilt_confirm_t = now
            clear_t = 0.0  # Blade restart timer reset karo
        else:
            tilt_confirm_t = 0.0

        if tilt_confirm_t > 0.0 and (now - tilt_confirm_t > 1.0):
            if state != 'TILT':
                log('ROBOT JHUKA (P:%.1f R:%.1f) - STOP & BLADE CUT' % (F_PIT, F_ROL))
                state = 'TILT'
            S['state'] = 'TILT STOP'
            brain_send('K'); S['blade'] = 'CUT'; brain_send('S')
            time.sleep(0.5); continue

        # ---- RAW SENSOR VALIDITY CHECK ----
        if not is_valid_distance(T['f']) or not is_valid_distance(T['l']) or not is_valid_distance(T['r']):
            if state != 'SENSOR_INVALID':
                log('SENSOR DATA INVALID/NaN — SAFE STOP & CUT')
                state = 'SENSOR_INVALID'
            S['state'] = 'SENSOR INVALID STOP'
            brain_send('K'); S['blade'] = 'CUT'; brain_send('S')
            time.sleep(0.4); continue

        # ---- UNIFIED SENSOR DISTANCE EXTRACTION ----
        emg_l = float(T['l'])
        emg_r = float(T['r'])
        emg_f = float(T['f'])
        c_fresh = (now - ARD_ST['age'] < 3.0 and is_valid_distance(T['c']) and float(T['c']) < 900)
        emg_c = float(T['c']) if c_fresh else 999.0
        emg_b = float(T['b']) if is_valid_distance(T['b']) else 999.0
        front_val = min(emg_f, emg_c)
        emg_min = min(emg_l, emg_r, front_val)

        # ---- 1.5. UNIFIED BOXED-IN & EMERGENCY LOGIC ----
        is_boxed_in = (front_val < 30.0 and emg_b < 25.0)
        is_emergency = (emg_min < 10.0)

        if is_boxed_in or is_emergency:
            # Blade OFF turant
            if S['blade'] == 'ON':
                brain_send('K'); S['blade'] = 'CUT'
                try: tts_speak('blade_off')
                except Exception: pass

            # CASE 1: Boxed In (Front < 30 AND Back < 25)
            if is_boxed_in:
                esp_send('C') # Servo Lock
                if emg_b < 15.0:
                    # TOTAL FREEZE
                    brain_send('S')
                    if state != 'FREEZE':
                        state = 'FREEZE'
                        log('FREEZE: Front:%.0f < 30 AND Back:%.0f < 15' % (front_val, emg_b))
                        try: tts_speak('stopping')
                        except Exception: pass
                    time.sleep(0.1)
                    continue
                else:
                    # BACK to get clearance at MIN SPEED, stop at 10cm
                    if state != 'BOXED_BACK':
                        state = 'BOXED_BACK'
                        log('BOXED IN: F:%.0f < 30, B:%.0f. Backing up SLOWLY to 10cm' % (front_val, emg_b))
                        try: tts_speak('rear_backing')
                        except Exception: pass

                    brain_send('1')  # MINIMUM SPEED
                    t_end = time.time() + 2.5  # Max time to avoid infinite loop
                    b_chk = emg_b
                    while time.time() < t_end:
                        b_chk = float(T['b']) if is_valid_distance(T['b']) else 999.0
                        if b_chk <= 10.0:
                            log('BACK LIMIT REACHED: B:%.0fcm - STOP' % b_chk)
                            break
                        brain_send('B', 0.1, force=True)

                    brain_send('S')

                    # SAFETY CHECK
                    log('SAFETY CHECK: Front:%.0f Back:%.0f Left:%.0f Right:%.0f' % (front_val, b_chk, emg_l, emg_r))
                    time.sleep(0.5) # Pause to evaluate

                    # Turn logic
                    if emg_l > AVOID_MARGIN and emg_l >= emg_r:
                        log('ESCAPE: LEFT (%.0fcm) - ARC LEFT' % emg_l)
                        brain_send('G', 0.5)
                    elif emg_r > AVOID_MARGIN:
                        log('ESCAPE: RIGHT (%.0fcm) - ARC RIGHT' % emg_r)
                        brain_send('H', 0.5)
                    else:
                        log('ESCAPE: Both sides tight - KHADA')

                    brain_send('S')
                    state = ''
                    time.sleep(0.4)
                    continue

            # CASE 2: Just Emergency (< 10cm side/front, Back clear)
            elif is_emergency:
                esp_send('C')
                brain_send('1')
                t_end = time.time() + 1.0
                while time.time() < t_end:
                    b_chk = float(T['b']) if is_valid_distance(T['b']) else 999.0
                    if b_chk < 25.0:
                        break
                    brain_send('B', 0.1, force=True)
                brain_send('S')
                esp_send('A')
                continue

        else:
            # All clear
            if state in ('BOXED_BACK', 'FREEZE'):
                log('CLEAR: F:%.0fcm B:%.0fcm — resume' % (front_val, emg_b))
                try: tts_speak('resume_mow')
                except Exception: pass
                state = ''

        # ---- 2. NAZAR ----
        name, zone = S['det_name'], S['det_zone']

        # ---- 2z. SMART-BLADE (auto AUR mow dono me) ----
        kyun = ''
        if name in LIVING:
            pd_2z = emg_c if c_fresh else emg_f
            if pd_2z <= SOFTF_CM:
                person_close_at = now  # person yaad tab rakho jab paas ho
                kyun = '%s CLOSE (%.0fcm)' % (name, pd_2z)
        elif front_val < OBJ_STOP_CM:
            kyun = 'F:%.0fcm paas' % front_val
        elif T['rain'] == 1:
            kyun = 'barish'
        elif is_tilted():
            kyun = 'robot jhuka'

        sab_saaf = (kyun == '')
        if sab_saaf:
            if clear_t == 0.0:
                clear_t = now
        else:
            clear_t = 0.0

        # Person memory check — 0.1s blade OFF
        if now - person_close_at < 0.1:
            sab_saaf = False

        if (S['blade'] == 'CUT' and MODE in ('auto', 'mow') and sab_saaf
                and clear_t > 0.0 and now - clear_t >= BLADE_CLEAR_SEC and now >= acs_lock_until):
            brain_send('M'); S['blade'] = 'ON'
            try: tts_speak('blade_on')
            except Exception: pass
            log('SAB SAAF %.1fs — blade KHUD ON' % (now - clear_t))
        elif S['blade'] == 'CUT' and MODE in ('auto', 'mow') and now - blade_t >= 4.0:
            blade_t = now
            if kyun:
                log('blade OFF hai (%s) — saaf hote hi KHUD ON hoga' % kyun)

        # ---- ACS OVERLOAD GUARD ----
        if S['blade'] == 'ON' and esp_age <= 2.0:
            if abs(T['acs']) > 25.0:
                acs_bad += 1
            else:
                acs_bad = 0
            if acs_bad >= 2:
                brain_send('K'); S['blade'] = 'CUT'; acs_bad = 0
                acs_lock_until = now + 10.0  # 10s lock
                log('ACS OVERLOAD! %.1fA — BLADE OFF (10s LOCK)' % T['acs'])
        else:
            acs_bad = 0

        # ---- 2b. CAMERA-YAAD ----
        if name != '-' and name not in LIVING and zone == 'CENTER':
            if name != cam_obj:
                cam_obj, cam_since = name, now
        else:
            cam_obj, cam_since = '', 0.0
        cam_long = (cam_obj != '' and now - cam_since >= CAM_OBJ_SEC)

        # ---- 3. LIVING (person/pet) — 5s wait + Side Escape / Center Back ----
        if name in LIVING:
            # Distance source: MULTI-SENSOR (zone ke hisaab se)
            if zone == 'LEFT':
                pd = min(emg_f, emg_l, emg_c)
            elif zone == 'RIGHT':
                pd = min(emg_f, emg_r, emg_c)
            else:
                pd = min(emg_f, emg_c)

            if pd > 55.0:
                if state == 'LIVING_FLOW':
                    if now - flow_last_close > 3.0:
                        log('%s door ho gaya (%.0fcm) - resume' % (name.upper(), pd))
                        try: tts_speak('resume_mow')
                        except Exception: pass
                        state = ''
                else:
                    state = ''
            else:
                person_close_at = now
                flow_last_close = now
                if S['blade'] == 'ON':
                    brain_send('K'); S['blade'] = 'CUT'
                    try: tts_speak('blade_off')
                    except Exception: pass

                if state != 'LIVING_FLOW':
                    state = 'LIVING_FLOW'
                    flow_t = now
                    hooter_t = 0.0
                    brain_send('S')
                    try: tts_speak('stopping')
                    except Exception: pass
                    log('%s paas (%.0fcm) - 2s SPEAKER + 3s WAIT' % (name.upper(), pd))

                elapsed = now - flow_t

                # Phase 1: 2s Speaker Warning
                if elapsed < 2.0:
                    S['state'] = 'LIVING SPEAKER (%.0fcm, %.1fs)' % (pd, elapsed)
                    brain_send('S')
                    time.sleep(0.3); continue

                # Phase 2: 3s Wait + Speaker
                if elapsed < 5.0:
                    S['state'] = 'LIVING WAIT (%.0fcm, %.1fs)' % (pd, elapsed - 2.0)
                    brain_send('S')
                    if not hooter_t or (now - hooter_t) > 2.0:
                        hooter_t = now
                        try: tts_speak('person_ahead')
                        except Exception: pass
                    time.sleep(0.3); continue

                # Phase 3: Action (Escape or Back)
                if zone == 'LEFT' or (emg_l < AVOID_MARGIN and zone != 'RIGHT'):
                    log('LIVING LEFT: Turning RIGHT to escape')
                    brain_send('H', TURN_ARC_S + 0.2)
                    brain_send('F', 0.5)
                    brain_send('S')
                    state = ''
                    time.sleep(1.0) # Let it move
                    continue

                elif zone == 'RIGHT' or (emg_r < AVOID_MARGIN and zone != 'LEFT'):
                    log('LIVING RIGHT: Turning LEFT to escape')
                    brain_send('G', TURN_ARC_S + 0.2)
                    brain_send('F', 0.5)
                    brain_send('S')
                    state = ''
                    time.sleep(1.0)
                    continue

                else:
                    # Center — 2s Back (Chunked)
                    if emg_b >= 25.0:
                        log('LIVING CENTER: Backing up 2s')
                        brain_send('1')
                        t_end = time.time() + 2.0
                        while time.time() < t_end:
                            b_chk = float(T['b']) if is_valid_distance(T['b']) else 999.0
                            if b_chk < 25.0:
                                log('SAFE_BACK: B:%.0fcm - ABORTED' % b_chk)
                                break
                            brain_send('B', 0.1, force=True)
                        brain_send('S')
                    else:
                        log('LIVING CENTER: Back blocked (B:%.0fcm) - KHADA' % emg_b)
                        brain_send('S')

                    state = ''
                    time.sleep(0.5)
                    continue

        elif state == 'LIVING_FLOW':
            if now - flow_last_close > 3.0:
                state = ''

        # ---- 3c9. SIDE STUCK - F + L/R <30cm -> back 1.0s + redirect ----
        side_stuck = (front_val < SOFTF_CM and emg_l < SOFTF_CM and emg_r < SOFTF_CM)

        if side_stuck:
            if S['blade'] == 'ON':
                brain_send('K'); S['blade'] = 'CUT'
                try: tts_speak('blade_off')
                except Exception: pass
            if state != 'SIDE_STUCK':
                state = 'SIDE_STUCK'
                stuck_t = now
                brain_send('S')
                log('SIDE-STUCK: F:%.0f L:%.0f R:%.0f - BACK 1.0s + redirect' % (front_val, emg_l, emg_r))
                time.sleep(0.2); continue
            elapsed = now - stuck_t
            if elapsed < 1.0:
                t_end = time.time() + 1.0
                while time.time() < t_end:
                    b_chk = float(T['b']) if is_valid_distance(T['b']) else 999.0
                    if b_chk < 25.0:
                        log('SAFE_BACK: B:%.0fcm - ABORTED' % b_chk)
                        break
                    brain_send('B', 0.1, force=True)

                brain_send('S')
                S['state'] = 'SIDE-STUCK BACK (%.1fs)' % elapsed
                time.sleep(0.1); continue
            if elapsed < 1.5:  # Redirect phase
                if emg_l >= emg_r and emg_l > AVOID_MARGIN:
                    log('redirect: LEFT (%.0f)' % emg_l)
                    brain_send('G', TURN_ARC_S); brain_send('S')
                elif emg_r > AVOID_MARGIN:
                    log('redirect: RIGHT (%.0f)' % emg_r)
                    brain_send('H', TURN_ARC_S); brain_send('S')
                else:
                    brain_send('S')
                S['state'] = 'SIDE-STUCK REDIRECT'
                time.sleep(0.3); continue
            state = ''
            time.sleep(0.2); continue

        # ---- 3d. SIDE OBSTACLE (L/R) — smart behavior ----
        if front_val >= HARD_CM:  # Sirf jab front clear ho
            l_near = emg_l < SIDE_NEAR_CM
            r_near = emg_r < SIDE_NEAR_CM

            if l_near:
                # Left obstacle — servo left lock 1s + back + move
                if state != 'SIDE_L':
                    state = 'SIDE_L'
                    side_l_t = now
                    esp_send('A')  # servo scan ON
                    brain_send('S')
                    log('L:%.0fcm paas — servo scan 1s' % emg_l)
                if now - side_l_t < 1.0:
                    S['state'] = 'SERVO-L (%.1fs)' % (now - side_l_t)
                    brain_send('S')
                    time.sleep(0.2); continue
                esp_send('C')  # servo idle
                brain_send('B', 0.3)
                brain_send('S')
                time.sleep(0.3)
                brain_send('H', 0.4)  # move right
                brain_send('S')
                brain_send('H', 0.3)  # ARC-RIGHT
                brain_send('S')
                S['state'] = 'SIDE-AVOID: L:%.0fcm -> RIGHT' % emg_l
                time.sleep(0.3)
                state = ''
                continue

            if r_near:
                # Right obstacle — servo right lock 1s + back + move
                if state != 'SIDE_R':
                    state = 'SIDE_R'
                    side_r_t = now
                    esp_send('A')
                    brain_send('S')
                    log('R:%.0fcm paas — servo scan 1s' % emg_r)
                if now - side_r_t < 1.0:
                    S['state'] = 'SERVO-R (%.1fs)' % (now - side_r_t)
                    brain_send('S')
                    time.sleep(0.2); continue
                esp_send('C')
                brain_send('B', 0.3)
                brain_send('S')
                time.sleep(0.3)
                brain_send('G', 0.4)
                brain_send('S')
                brain_send('G', 0.3)  # ARC-LEFT
                brain_send('S')
                S['state'] = 'SIDE-AVOID: R:%.0fcm -> LEFT' % emg_r
                time.sleep(0.3)
                state = ''
                continue

        # ---- 3e. 3-SIDE BLOCKED -> BACK-TRACK ----
        if front_val < HARD_CM and emg_l < HARD_CM and emg_r < HARD_CM:
            if S['blade'] == 'ON':
                brain_send('K'); S['blade'] = 'CUT'
                try: tts_speak('blade_off')
                except Exception: pass
            if state != 'TRIPLE_BLOCK':
                state = 'TRIPLE_BLOCK'
                triple_t = now
                brain_send('S')
                try: tts_speak('back_track')
                except Exception: pass
                log('3-SIDE BLOCKED (F:%.0f L:%.0f R:%.0f) - back-track check' % (front_val, emg_l, emg_r))
                time.sleep(0.3); continue
            S['state'] = '3-SIDE (F:%.0f L:%.0f R:%.0f B:%.0f)' % (front_val, emg_l, emg_r, emg_b)
            if now - triple_t < 1.5:
                brain_send('S')
                time.sleep(0.3); continue
            if emg_b >= BACK_REACT_CM:
                log('peeche %.0fcm khali - BACK jaa raha hu' % emg_b)
                brain_send('B', 0.8)
                brain_send('S')
                time.sleep(0.4)
                if emg_l >= emg_r and emg_l > AVOID_MARGIN:
                    log('back ke baad LEFT (%.0fcm) - wahan se nikala' % emg_l)
                    brain_send('G', TURN_ARC_S); brain_send('S')
                elif emg_r > AVOID_MARGIN:
                    log('back ke baad RIGHT (%.0fcm) - wahan se nikala' % emg_r)
                    brain_send('H', TURN_ARC_S); brain_send('S')
                else:
                    log('back ke baad dono side band - khada')
                    brain_send('S')
            else:
                log('peeche bhi band (B:%.0fcm) - KHADA hu' % emg_b)
                brain_send('S')
            state = ''
            time.sleep(0.4); continue

        # ---- 4. CAM-US (FIXED) HARD-BLOCK @40cm — RUK ke SOCH -> AVOID ----
        if front_val < HARD_CM:
            brain_send('S', wait=0.2)
            if state != 'SOCH':
                state = 'SOCH'; soch_t = now
                soch_cnt += 1
                log('CAM-US: saamne %0.fcm — RUK ke 2s SOCHunga (try %d)' % (front_val, soch_cnt))
            S['state'] = 'SOCH (fixed-radar %.0fcm)' % front_val
            if now - soch_t < OBST_WAIT_SEC:
                time.sleep(0.2); continue
            if front_val < HARD_CM:
                if soch_cnt >= 3:
                    log('CAM-US STUCK: 3 baar same obstacle — BACK & HARD TURN')
                    brain_send('B', 1.2, force=True)
                    brain_send('S')
                    time.sleep(0.4)
                    brain_send('H' if emg_r > emg_l else 'G', 1.5)
                    brain_send('S')
                    soch_cnt = 0
                elif emg_l > AVOID_MARGIN and emg_l >= emg_r + 20:
                    log('avoid: ARC-LEFT (L:%.0fcm, door-se)' % emg_l)
                    brain_send('G', TURN_ARC_S + 0.15); brain_send('S')
                elif emg_r > AVOID_MARGIN:
                    log('avoid: ARC-RIGHT (R:%.0fcm, door-se)' % emg_r)
                    brain_send('H', TURN_ARC_S + 0.15); brain_send('S')
                elif emg_l > emg_r:
                    log('left thoda behtar — ARC-LEFT fir bhi (L:%.0f R:%.0f)' % (emg_l, emg_r))
                    brain_send('G', TURN_ARC_S); brain_send('S')
                else:
                    log('right try — ARC-RIGHT (L:%.0f R:%.0f)' % (emg_l, emg_r))
                    brain_send('H', TURN_ARC_S); brain_send('S')
            else:
                log('SOCH poora: rasta KHUD khul gaya — aage!')
                soch_cnt = 0
            state = ''
            time.sleep(0.25); continue

        # ---- 5. YOLO-OBJECT (bottle/chair...) — detect pe CAM-US 30cm rule ----
        obj_near = front_val < SOFTF_CM
        if name != '-' and name not in LIVING and zone == 'CENTER' and (obj_near or cam_long):
            brain_send('S')
            if state != 'OBJ':
                state = 'OBJ'; soch_t = now
                log('%s dikha + CAM-US %.0fcm — RUK ke sochunga' % (name.upper(), front_val))
            S['state'] = 'SOCH (object %.0fcm)' % front_val
            if now - soch_t < OBST_WAIT_SEC:
                time.sleep(0.2); continue
            if obj_near or cam_long:
                if emg_l > AVOID_MARGIN and emg_l >= emg_r + 20:
                    log('avoid: ARC-LEFT object ke (%.0fcm, door-se)' % front_val)
                    brain_send('G', TURN_ARC_S + 0.15); brain_send('S')
                elif emg_r > AVOID_MARGIN:
                    log('avoid: ARC-RIGHT object ke (%.0fcm, door-se)' % emg_r)
                    brain_send('H', TURN_ARC_S + 0.15); brain_send('S')
                elif emg_l > emg_r:
                    brain_send('G', TURN_ARC_S); brain_send('S')
                    log('left-behtar — ARC-LEFT (L:%.0f R:%.0f)' % (emg_l, emg_r))
                else:
                    brain_send('H', TURN_ARC_S); brain_send('S')
                    log('right-behtar — ARC-RIGHT (L:%.0f R:%.0f)' % (emg_l, emg_r))
            else:
                log('soch poora: %s HAT GAYA — aage!' % name.upper())
            state = ''
            time.sleep(0.25); continue

        # ==== 6m. MOW NAVIGATION v2 (zigzag + GRID 5x5/10x10 backup) ====
        if MODE == 'mow':
            if G['last_ok'] == 0 or now - G['last_ok'] > 6.0:
                S['state'] = 'MOW: GPS GAYAB — KHADA (safety)'
                brain_send('S'); brain_send('K'); S['blade'] = 'CUT'
                time.sleep(0.5); continue
            if not MOWW['wps']:
                S['state'] = 'MOW: pehle 4 CORNER set karo!'
                brain_send('S'); time.sleep(0.4); continue
            # --- GRID GUARD: perimeter check ---
            gc = mow_grid_check()
            if gc:
                inside, gd, limit = gc
                if inside:
                    MOWW['grid_ok'] = MOWW.get('grid_ok', 0) + 1
                else:
                    MOWW['grid_bad'] = MOWW.get('grid_bad', 0) + 1
                    brain_send('S')
                    S['state'] = 'MOW GRID-BAHAR! (%.1fm > limit %.1fm) — wapas ghuss' % (gd, limit)
                    if now - beat >= 3.0:
                        log('GRID-BAHAR (%.1fm > %.1fm) — back+turn karke wapas' % (gd, limit))
                        beat = now
                    brain_send('B', 0.6); brain_send('S')
                    brain_send('R' if (MOWW['wi'] % 2) else 'L', TURN_STEP); brain_send('S')
                    time.sleep(0.3); continue
            r = mow_poll()
            if r is None:
                log('MOW: POORA FIELD KAT GAYA — SHABASH! (grid ok=%d bad=%d)' % (MOWW.get('grid_ok', 0), MOWW.get('grid_bad', 0)))
                brain_send('K'); S['blade'] = 'CUT'; brain_send('S')
                MODE = 'stop'; S['mode'] = 'stop'
                S['state'] = 'FIELD DONE'
                continue
            wlat, wlon, dist, ln, tot = r
            brg = bearing_deg((G['lat'], G['lon']), (wlat, wlon))
            crs = mow_course()
            if crs is None:
                # GPS-course nahi ban raha — aage pulse do
                brain_send('F', 0.4)
                S['state'] = 'MOW line %d/%d — course nikal raha (%.1fm baki)' % (ln, tot, dist)
            else:
                err = (brg - crs + 540.0) % 360.0 - 180.0
                if abs(err) > 30:
                    brain_send('R' if err > 0 else 'L', 0.35)
                    S['state'] = 'MOW line %d/%d — turn %.0f° (aligning)' % (ln, tot, err)
                else:
                    brain_send('F', 0.35)
                    S['state'] = 'MOW line %d/%d — seedha (%.1fm baki)' % (ln, tot, dist)
            if now - beat >= 8.0:
                log('MOW: line %d/%d, %.1fm baki (err %.0f°)' % (ln, tot, dist, err if crs else 0))
                beat = now
            time.sleep(0.15); continue

        # ---- 6. SPEED (auto free-roam) ----
        want = '5'   # Speed 5: MAX for 30RPM high-torque motors
        if want != last_speed:
            brain_send(want); last_speed = want
            S['spd'] = '%s (~%d PWM)' % (want, SPDL.get(want, 0))

        # ---- 7. GO ----
        if state != 'GO':
            log('saaf (F:%.0f L:%.0f R:%.0f) — AAGE' % (front_val, emg_l, emg_r))
            state = 'GO'; beat = now
        S['state'] = 'FOLLOW PATH'
        if now - beat >= 6.0:
            log('chal raha... (F:%.0f L:%.0f R:%.0f | dekh: %s)' % (front_val, emg_l, emg_r, name))
            beat = now
        brain_send('F', 0.25)

def brain_loop():
    while True:
        try:
            _brain_inner()
        except Exception as e:
            brain_send('K')
            S['blade'] = 'CUT'
            brain_send('S')
            S['state'] = 'BRAIN ERROR — STOP & CUT'
            log('BRAIN ERROR: %s — safe STOP & BLADE CUT' % e)
            time.sleep(2)

def manual_watchdog():
    global LAST_CMD
    while True:
        time.sleep(0.2)  # Fast loop for safety
        if MODE == 'manual' and ard is not None:
            esp_age = get_esp_age()

            # 1. Stale sensors
            if esp_age > 2.0:
                if S['state'] != 'MANUAL: SENSORS STALE':
                    ard_send('K'); ard_send('S')
                    S['blade'] = 'CUT'
                    S['state'] = 'MANUAL: SENSORS STALE'
                    log('MANUAL STALE! Motors & Blade cut.')
                continue

            # 2. Rain
            if T['rain'] == 1:
                if S['state'] != 'MANUAL: RAIN STOP':
                    ard_send('K'); ard_send('S')
                    S['blade'] = 'CUT'
                    S['state'] = 'MANUAL: RAIN STOP'
                    log('MANUAL RAIN! Motors & Blade cut.')
                continue

            # 3. Tilt
            if is_tilted():
                if S['state'] != 'MANUAL: TILT STOP':
                    ard_send('K'); ard_send('S')
                    S['blade'] = 'CUT'
                    S['state'] = 'MANUAL: TILT STOP'
                    log('MANUAL TILT! Motors & Blade cut.')
                continue

            # 4. ACS Overload
            if S['blade'] == 'ON' and abs(T['acs']) > 25.0:
                if S['state'] != 'MANUAL: ACS OVERLOAD':
                    ard_send('K'); S['blade'] = 'CUT'
                    S['state'] = 'MANUAL: ACS OVERLOAD'
                    log('MANUAL ACS OVERLOAD! Blade cut.')
                continue

            # Clear safety state if all good
            if S['state'] in ('MANUAL: SENSORS STALE', 'MANUAL: RAIN STOP', 'MANUAL: TILT STOP', 'MANUAL: ACS OVERLOAD'):
                S['state'] = 'MANUAL'

            # Continuous mode: Watchdog disabled
            if time.time() - LAST_CMD >= 9999.0:
                brain_send('S')
                brain_send('K')
                S['blade'] = 'CUT'
                S['state'] = 'WATCHDOG — SAB OFF'
                log('WATCHDOG: TIMEOUT — MOTORS + BLADE OFF')
                LAST_CMD = time.time()

# ---------------- ROUTES ----------------
@app.route('/data')
def data():
    if not authed():
        return Response('login pehle', 401)
    d = dict(S)
    d['LAT'] = G['lat']
    d['LON'] = G['lon']
    d['GACC'] = G['acc']
    d['GAGE'] = round(time.time() - G['last_ok'], 1) if G['last_ok'] else 999
    d['GRAW'] = G['raw']
    d['UP'] = int(time.time() - START_T)
    d['MC'] = len(MOWW['corners'])
    d['MWI'] = MOWW['wi']
    d['MWN'] = len(MOWW['wps'])
    d['MSP'] = MOWW['spacing']
    d['MWPTS'] = [[round(a, 6), round(b, 6)] for a, b in MOWW['wps']]
    d['MCC'] = [[round(v[0], 6), round(v[1], 6)] for _, v in sorted(MOWW['corners'].items())]
    d['MROT'] = [G['lat'], G['lon']] if G['last_ok'] else []
    return d

@app.route('/gps')
def gpsroute():
    if not authed():
        return Response('login pehle', 401)
    return {'url': NARZO_URL, 'lat': G['lat'], 'lon': G['lon'],
            'acc': G['acc'], 'raw': G['raw'],
            'age': round(time.time() - G['last_ok'], 1) if G['last_ok'] else 999}

@app.route('/log')
def logroute():
    if not authed():
        return Response('login pehle', 401)
    return '<br>'.join(LOG)

@app.route('/mode')
def mode():
    global MODE
    if not authed():
        return Response('login pehle', 401)
    m = request.args.get('m', 'stop')
    if m not in ('auto', 'manual', 'stop', 'mow'):
        return 'galat mode'
    MODE = m
    S['mode'] = m
    brain_send('S')
    if m == 'stop':
        brain_send('K')
        S['blade'] = 'CUT'
        try: tts_speak('mode_stop')
        except Exception: pass
    elif m == 'auto':
        try: tts_speak('mode_auto')
        except Exception: pass
    elif m == 'manual':
        try: tts_speak('mode_manual')
        except Exception: pass
    elif m == 'mow':
        try: tts_speak('mode_mow')
        except Exception: pass
    ard_send(radar_mode_cmd())
    esp_send(radar_mode_cmd())
    S['state'] = {'manual': 'MANUAL', 'stop': 'STOPPED',
                  'auto': 'AUTO — soch raha hu...',
                  'mow': 'MOW — corners ka intezar...' if not MOWW['wps'] else 'MOW — zigzag taiyar!'}[m]
    S['spd'] = '-'
    log('MODE -> %s' % m.upper())
    return 'MODE: ' + m

@app.route('/corner')
def corner():
    if not authed():
        return Response('login pehle', 401)
    try:
        i = int(request.args.get('i', '0'))
    except Exception:
        return 'i=1..4'
    if i not in (1, 2, 3, 4):
        return 'i=1..4'
    if G['last_ok'] == 0 or time.time() - G['last_ok'] > 30 or G['lat'] == '-' or G['lon'] == '-':
        return 'GPS purana/gayab — Narzo dekho'
    try:
        acc = float(G['acc'])
        glat = float(G['lat'])
        glon = float(G['lon'])
    except Exception:
        return 'accuracy ya GPS data galat'
    if acc > 30:
        return 'accuracy kharab (%.0f m) — khule asman me karo' % acc
    MOWW['corners'][i] = (glat, glon)
    log('MOW: CORNER %d set (%s, acc %.0fm)' % (i, G['lat'], acc))
    return 'C%d set OK (acc %.0f m)' % (i, acc)

@app.route('/mow')
def mowroute():
    global MODE
    if not authed():
        return Response('login pehle', 401)
    a = request.args.get('a', '')
    if a == 'grid':
        try:
            n = int(request.args.get('n', '5'))
        except Exception:
            n = 5
        MOWW['grid_n'] = n if n in (5, 10) else 5
        return 'GRID %dx%d chalu (guard ON)' % (MOWW['grid_n'], MOWW['grid_n'])
    if a == 'start':
        try:
            sp = float(request.args.get('sp', '1.0'))
        except Exception:
            sp = 1.0
        sp = max(0.4, min(3.0, sp))
        MOWW['spacing'] = sp
        n = mow_build()
        if n == 0:
            return 'pehle 4 CORNER set karo!'
        MODE = 'mow'
        S['mode'] = 'mow'
        del GPS_HIST[:]
        brain_send('S'); brain_send('5')   # '5' = MAX for 30RPM
        ard_send('A'); esp_send('A')
        log('MOW START: %d lines, spacing %.1fm — ZIGZAG chalu!' % (n, sp))
        return 'MOW START (%d lines, %.1fm)' % (n, sp)
    if a == 'stop':
        MODE = 'stop'
        S['mode'] = 'stop'
        brain_send('S'); brain_send('K'); S['blade'] = 'CUT'
        ard_send('C'); esp_send('C')
        log('MOW STOP — robot khada, blade cut')
        return 'MOW STOP'
    return 'a=start|stop'

@app.route('/cmd')
def cmd():
    if not authed():
        return Response('login pehle', 401)
    raw_c = (request.args.get('c') or '').strip()
    if not raw_c:
        return 'galat command'
    c = raw_c[0]
    if c not in 'FBLRS123456789MKVGHAC':
        return 'galat command'
    if c not in 'SKACM' and S['mode'] != 'manual':
        return 'pehle MANUAL mode kholo (AUTO chal raha hai)'

    # ---- MANUAL SAFETY OVERRIDE ----
    if get_esp_age() > 2.0:
        return 'SENSORS STALE! Command reject.'

    if T['rain'] == 1 and c == 'M':
        return 'BARISH! Blade ON nahi hoga'

    if is_tilted() and c in 'FBLR123456789M':
        return 'TILT! Robot jhuka hua hai. Command reject.'

    if c == 'B' and back_blocked():
        return 'PICHE RUKAWAT! B:%s cm (<25) - nahi jaunga' % T['b']

    f_val = float(T['f']) if is_valid_distance(T['f']) else 999.0
    c_val = float(T['c']) if is_valid_distance(T['c']) else 999.0
    l_val = float(T['l']) if is_valid_distance(T['l']) else 999.0
    r_val = float(T['r']) if is_valid_distance(T['r']) else 999.0

    if c == 'F' and min(f_val, c_val) < 40.0:
        return 'SAAMNE RUKAWAT! F:%.0f C:%.0f - nahi jaunga' % (f_val, c_val)

    if c == 'L' and l_val < 25.0:
        return 'LEFT RUKAWAT! L:%.0f - nahi jaunga' % l_val

    if c == 'R' and r_val < 25.0:
        return 'RIGHT RUKAWAT! R:%.0f - nahi jaunga' % r_val
    # --------------------------------

    if c in ('A', 'C'):
        esp_send(c)
        out = 'ESP32 Servo OK'
    else:
        out = ard_send(c)
    log('MANUAL CMD: %s -> %s' % (c, out.strip() if out else 'ok'))
    if c == 'M':
        S['blade'] = 'ON'
        try: tts_speak('blade_on')
        except Exception: pass
    elif c == 'K':
        S['blade'] = 'CUT'
        try: tts_speak('blade_off')
        except Exception: pass
    S['rep'] = c + ' -> ' + (out if out else '(ok)')
    return S['rep']

@app.route('/login', methods=['POST'])
def login():
    u = (request.form.get('u') or (request.json or {}).get('u', '')).strip()
    p = (request.form.get('p') or (request.json or {}).get('p', '')).strip()
    if u == AUTH_USER and p == AUTH_PASS:
        resp = make_response('ok')
        resp.set_cookie(COOKIE, AUTH_TOKEN, max_age=7 * 24 * 3600, httponly=True)
        log('LOGIN: %s' % u)
        return resp
    return ('galat username ya password', 401)

@app.route('/logout')
def logout():
    resp = make_response(redirect('/'))
    resp.delete_cookie(COOKIE)
    return resp

LOGO_SVG = '<svg viewBox="0 0 48 48" width="36" height="36"><defs><linearGradient id="lg1" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="#34d399"/><stop offset="1" stop-color="#059669"/></linearGradient></defs><path d="M24 3 L42 10 V24 C42 35 34 42.5 24 45 C14 42.5 6 35 6 24 V10 Z" fill="url(#lg1)"/><path d="M24 12 C17 16 15 22 16 28 C22 27 27 23 28 15 C27 14 25.5 13 24 12 Z M30 16 C30.5 24 26 30 19 32 C20 34 22 36 24 37 C32 33 36 25 34 15 Z" fill="#052e16"/></svg>'

LOGIN_PAGE = '''<!doctype html><html><head><meta charset="utf-8">
<title>GrassGuard AI — Secure Login</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{min-height:100vh;display:flex;align-items:center;justify-content:center;overflow:hidden;
font-family:'Segoe UI',system-ui,Arial,sans-serif;background:#05080f;color:#e7eefc}
.aur{position:fixed;border-radius:50%;filter:blur(90px);opacity:.5;animation:drift 14s ease-in-out infinite alternate}
.a1{width:420px;height:420px;background:#0e5a3c;top:-120px;left:-100px}
.a2{width:380px;height:380px;background:#123a7a;bottom:-120px;right:-80px;animation-delay:-7s}
@keyframes drift{from{transform:translate(0,0) scale(1)}to{transform:translate(60px,40px) scale(1.15)}}
.box{position:relative;z-index:2;width:min(410px,92vw);background:rgba(15,23,42,.72);backdrop-filter:blur(18px);
border:1px solid rgba(148,163,184,.14);border-radius:24px;padding:38px 32px;box-shadow:0 30px 80px #000c;text-align:center}
.logo{filter:drop-shadow(0 6px 18px #10b98155);animation:bob 3.2s ease-in-out infinite}
@keyframes bob{0%,100%{transform:translateY(0)}50%{transform:translateY(-6px)}}
h1{font-size:26px;margin-top:14px}
h1 b{background:linear-gradient(90deg,#34d399,#22d3ee);-webkit-background-clip:text;background-clip:text;color:transparent}
.sub{color:#7d93b8;font-size:11.5px;margin:8px 0 28px;letter-spacing:.22em}
.f{width:100%;text-align:left;margin-bottom:15px}
.f label{display:block;font-size:10.5px;letter-spacing:.16em;color:#7d93b8;margin-bottom:7px}
.f input{width:100%;padding:13px 15px;border-radius:13px;border:1px solid rgba(148,163,184,.16);
background:rgba(2,6,23,.6);color:#e7eefc;font-size:15px;outline:none;transition:.2s}
.f input:focus{border-color:#34d399;box-shadow:0 0 0 4px rgba(52,211,153,.12)}
button{width:100%;padding:14px;border:0;border-radius:13px;font-size:15.5px;font-weight:800;letter-spacing:.06em;cursor:pointer;
background:linear-gradient(135deg,#34d399,#059669);color:#03150c;box-shadow:0 10px 30px rgba(5,150,105,.35);transition:.12s}
button:active{transform:scale(.97)}
#err{color:#f87171;font-size:13px;min-height:20px;margin-top:13px}
.shake{animation:sh .35s}
@keyframes sh{0%,100%{transform:translateX(0)}25%{transform:translateX(-8px)}75%{transform:translateX(8px)}}
.foot{margin-top:24px;font-size:10.5px;color:#4c5f85;letter-spacing:.08em}
</style></head><body>
<div class="aur a1"></div><div class="aur a2"></div>
<div class="box" id="box">
  <div class="logo">''' + LOGO_SVG + '''</div>
  <h1>Grass<b>Guard</b> AI</h1>
  <div class="sub">PI-BRAIN v1.3 • ZIGZAG-MOW EDITION</div>
  <div class="f"><label>USERNAME</label><input id="u" autocomplete="username" placeholder="username"></div>
  <div class="f"><label>PASSWORD</label><input id="p" type="password" autocomplete="current-password" placeholder="••••••"></div>
  <button onclick="go()">ACCESS CONTROL CENTER →</button>
  <div id="err"></div>
  <div class="foot">SECURE • PI 4 BRAIN • NARZO GPS LINK 🛰️</div>
</div>
<script>
function go(){
var u=document.getElementById('u').value,p=document.getElementById('p').value;
fetch('/login',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({u:u,p:p})})
.then(function(r){if(r.ok){location.reload()}else{throw 0}})
.catch(function(){var e=document.getElementById('err');e.textContent='galat username ya password';
var b=document.getElementById('box');b.classList.remove('shake');void b.offsetWidth;b.classList.add('shake')})}
document.addEventListener('keydown',function(e){if(e.key=='Enter')go()});
</script></body></html>'''

DASH_PAGE = '''<!doctype html><html><head><meta charset="utf-8">
<title>GrassGuard AI — Control Center</title>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<style>
:root{--bg:#05080f;--glass:rgba(15,23,42,.66);--line:rgba(148,163,184,.13);--txt:#e7eefc;--mut:#7d93b8;
--ok:#34d399;--warn:#fbbf24;--bad:#f87171;--cyan:#22d3ee}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:var(--bg);color:var(--txt);font-family:'Segoe UI',system-ui,Arial,sans-serif;
-webkit-user-select:none;user-select:none;overflow-x:hidden}
.aur{position:fixed;border-radius:50%;filter:blur(110px);opacity:.4;animation:drift 16s ease-in-out infinite alternate;z-index:0;pointer-events:none}
.a1{width:460px;height:460px;background:#0e5a3c;top:-140px;left:-120px}
.a2{width:420px;height:420px;background:#123a7a;bottom:-140px;right:-100px;animation-delay:-8s}
@keyframes drift{from{transform:translate(0,0) scale(1)}to{transform:translate(70px,50px) scale(1.18)}}
.topbar{position:sticky;top:0;z-index:20;display:flex;align-items:center;gap:9px;padding:10px 14px;
background:rgba(5,8,15,.75);backdrop-filter:blur(14px);border-bottom:1px solid var(--line)}
.brand{font-weight:800;font-size:15.5px;white-space:nowrap}
.brand small{display:block;color:var(--mut);font-weight:600;font-size:9.5px;letter-spacing:.18em;margin-top:1px}
.chip{font-size:10px;padding:4px 9px;border-radius:99px;border:1px solid var(--line);background:rgba(2,6,23,.5);color:var(--mut);display:flex;align-items:center;gap:5px;white-space:nowrap}
.dot{width:7px;height:7px;border-radius:50%;background:#334155}
.dot.ok{background:var(--ok);box-shadow:0 0 8px var(--ok)}
.dot.bad{background:var(--bad);box-shadow:0 0 8px var(--bad);animation:blink 1s infinite}
@keyframes blink{50%{opacity:.3}}
.spacer{flex:1}
.up{font-size:10.5px;color:var(--mut);font-variant-numeric:tabular-nums}
.user{font-size:11px;padding:5px 11px;border-radius:99px;background:rgba(52,211,153,.12);border:1px solid rgba(52,211,153,.3);color:var(--ok);font-weight:700}
.lo{font-size:11px;padding:5px 11px;border-radius:99px;border:1px solid var(--line);background:transparent;color:var(--mut);cursor:pointer}
.wrap{position:relative;z-index:1;display:grid;grid-template-columns:minmax(0,1fr) 370px;gap:13px;padding:13px;max-width:1300px;margin:0 auto}
@media(max-width:980px){.wrap{grid-template-columns:1fr}.up{display:none}}
.card{background:var(--glass);backdrop-filter:blur(12px);border:1px solid var(--line);border-radius:18px;padding:14px;box-shadow:0 10px 34px rgba(0,0,0,.45)}
.card h3{margin:0 0 11px;font-size:10px;letter-spacing:.2em;color:var(--mut);display:flex;align-items:center;gap:7px;font-weight:700}
.hdot{width:6px;height:6px;border-radius:50%;background:var(--ok);box-shadow:0 0 7px var(--ok);animation:blink 2.2s infinite}
.col{display:flex;flex-direction:column;gap:13px;min-width:0}
.seg{display:flex;position:relative;background:rgba(2,6,23,.65);border:1px solid var(--line);border-radius:14px;padding:4px;gap:4px}
.segb{flex:1;padding:12px 4px;border-radius:11px;border:0;background:transparent;color:var(--mut);font-size:13.5px;font-weight:700;cursor:pointer;transition:.2s}
.segb.on{color:#03150c;background:linear-gradient(135deg,#34d399,#059669);box-shadow:0 4px 18px rgba(5,150,105,.4)}
.segb.man.on{background:linear-gradient(135deg,#38bdf8,#0369a1);color:#031524;box-shadow:0 4px 18px rgba(3,105,161,.4)}
.segb.stp.on{background:linear-gradient(135deg,#f87171,#dc2626);color:#fff;box-shadow:0 4px 18px rgba(220,38,38,.4)}
.segb.mowb.on{background:linear-gradient(135deg,#a3e635,#65a30d);color:#0d1a02;box-shadow:0 4px 18px rgba(101,163,13,.4)}
#state{margin-top:11px;text-align:center;font-size:14px;font-weight:700;color:var(--ok);min-height:20px}
.vwrap{position:relative;border-radius:13px;overflow:hidden;background:#000}
.vwrap img{width:100%;display:block;aspect-ratio:16/9;object-fit:contain}
.scan{position:absolute;inset:0;pointer-events:none;background:repeating-linear-gradient(0deg,transparent 0 3px,rgba(0,0,0,.14) 3px 4px)}
.live{position:absolute;top:9px;right:9px;font-size:10px;font-weight:800;letter-spacing:.1em;color:#fff;background:rgba(220,38,38,.85);padding:4px 10px;border-radius:99px;animation:blink 1.6s infinite}
.det{position:absolute;top:9px;left:9px;font-size:12px;font-weight:700;background:rgba(2,6,23,.72);border:1px solid var(--line);padding:5px 11px;border-radius:99px;color:var(--ok)}
.vstrip{display:flex;gap:7px;margin-top:9px;flex-wrap:wrap}
.pill{font-size:10.5px;padding:5px 10px;border-radius:99px;border:1px solid var(--line);background:rgba(2,6,23,.5);color:var(--mut)}
.pill b{color:var(--txt);font-variant-numeric:tabular-nums}
.bladecard{text-align:center}
#saw{display:inline-block;font-size:30px;margin-bottom:2px}
#blade{display:block;font-size:23px;font-weight:900;letter-spacing:.08em;padding:9px 30px;border-radius:14px;margin-top:4px}
#blade.cut{background:rgba(127,29,29,.25);color:var(--bad);border:1px solid rgba(127,29,29,.6)}
#blade.on{background:rgba(6,78,59,.35);color:var(--ok);border:1px solid rgba(6,95,70,.7);box-shadow:0 0 24px rgba(52,211,153,.35)}
.spin{animation:rot 1s linear infinite;display:inline-block}
@keyframes rot{to{transform:rotate(360deg)}}
.gbox{text-align:center}
.gbox span{display:block;font-size:10px;letter-spacing:.14em;color:var(--mut);margin-top:-4px}
.bl{display:flex;justify-content:space-between;font-size:12px;color:var(--mut);margin-top:8px}
.bl b{color:var(--txt);font-variant-numeric:tabular-nums}
.bar{background:rgba(2,6,23,.6);border:1px solid var(--line);border-radius:9px;height:16px;margin:4px 0 9px;overflow:hidden}
.fill{height:100%;border-radius:8px;transition:width .4s;background:linear-gradient(90deg,#059669,#34d399)}
.fill.y{background:linear-gradient(90deg,#b45309,#fbbf24)}
.fill.r{background:linear-gradient(90deg,#b91c1c,#f87171)}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:9px}
.gpsbig{font-family:ui-monospace,Consolas,monospace;font-size:15.5px;color:var(--ok);text-align:center;padding:4px 0 8px;word-break:break-all}
#log{background:rgba(2,6,23,.8);border:1px solid var(--line);text-align:left;font-size:11px;padding:10px;border-radius:11px;min-height:120px;max-height:180px;overflow:auto;color:#86efac;font-family:ui-monospace,Consolas,monospace;white-space:pre-line;line-height:1.6}
.rmt{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;max-width:280px;margin:0 auto}
.pad{font-size:19px;padding:15px 0;border-radius:13px;border:1px solid var(--line);background:rgba(2,6,23,.55);color:var(--txt);cursor:pointer;touch-action:none}
.pad:active,.pad.on{background:#fbbf24;color:#231000;box-shadow:0 0 18px rgba(251,191,36,.5);transform:scale(.96)}
.pad.danger{border-color:rgba(127,29,29,.7);color:var(--bad)}
.pad.danger:active{background:var(--bad);color:#fff}
.rbtns{display:flex;gap:7px;justify-content:center;flex-wrap:wrap;margin-top:11px}
.sbtn{font-size:12.5px;padding:9px 14px;border-radius:11px;border:1px solid var(--line);background:rgba(2,6,23,.55);color:var(--txt);cursor:pointer}
.sbtn:active{transform:scale(.95)}
.sbtn.grn{border-color:rgba(6,95,70,.8);color:var(--ok)}
.sbtn.red{border-color:rgba(127,29,29,.8);color:var(--bad)}
.sbtn.done{background:rgba(52,211,153,.15);border-color:rgba(52,211,153,.5);color:var(--ok)}
.tgl{display:flex;gap:8px;justify-content:center;margin-bottom:11px}
.tb{font-size:12px;padding:7px 15px;border-radius:99px;border:1.5px solid var(--cyan);background:transparent;color:var(--cyan);cursor:pointer;font-weight:700}
.tb.on{background:var(--cyan);color:#031524}
.mowrow{display:flex;gap:7px;justify-content:center;flex-wrap:wrap;margin-bottom:9px}
#sp{width:76px;padding:8px 10px;border-radius:10px;border:1px solid var(--line);background:rgba(2,6,23,.6);color:var(--txt);font-size:13px;text-align:center}
.hint{color:var(--mut);font-size:10.5px;text-align:center;min-height:16px;margin-top:8px}
#mst{color:var(--warn);font-weight:700}
canvas{display:block;margin:0 auto;border-radius:10px}
#estop{position:fixed;right:16px;bottom:16px;z-index:50;width:76px;height:76px;border-radius:50%;border:4px solid #7f1d1d;cursor:pointer;
background:radial-gradient(circle at 35% 30%,#f87171,#b91c1c 70%);color:#fff;font-size:11px;font-weight:900;letter-spacing:.06em;
box-shadow:0 0 0 6px rgba(185,28,28,.25),0 10px 30px rgba(0,0,0,.6);animation:epulse 2s infinite}
#estop:active{transform:scale(.93)}
@keyframes epulse{0%,100%{box-shadow:0 0 0 6px rgba(185,28,28,.25),0 10px 30px rgba(0,0,0,.6)}50%{box-shadow:0 0 0 12px rgba(185,28,28,.12),0 10px 30px rgba(0,0,0,.6)}}
#toast{position:fixed;left:50%;bottom:28px;transform:translateX(-50%);background:rgba(15,23,42,.92);border:1px solid var(--line);color:var(--txt);
padding:11px 20px;border-radius:12px;font-size:13px;box-shadow:0 10px 30px rgba(0,0,0,.7);opacity:0;pointer-events:none;transition:.25s;max-width:86vw;text-align:center;z-index:99;backdrop-filter:blur(8px)}
</style></head><body>
<div class="aur a1"></div><div class="aur a2"></div>
<div class="topbar">
''' + LOGO_SVG + '''
  <div class="brand">GrassGuard AI<small>PI-BRAIN v1.3 • ZIGZAG-MOW</small></div>
  <div class="chip"><span class="dot" id="d_esp"></span>ESP32</div>
  <div class="chip"><span class="dot" id="d_ard"></span>ARD</div>
  <div class="chip"><span class="dot" id="d_gps"></span>GPS</div>
  <div class="spacer"></div>
  <div class="up" id="up">⏱ 0s</div>
  <div class="user">shubham27</div>
  <button class="lo" onclick="location='/logout'">Logout</button>
</div>
<div class="wrap">
 <div class="col">
  <div class="card">
    <h3><span class="hdot"></span>SYSTEM CONTROL</h3>
    <div class="seg">
      <button class="segb" id="m_auto" onclick="setMode('auto')">▶ AUTO</button>
      <button class="segb mowb" id="m_mow" onclick="setMode('mow')">🌾 MOW</button>
      <button class="segb man" id="m_manual" onclick="setMode('manual')">✋ MANUAL</button>
      <button class="segb stp on" id="m_stop" onclick="setMode('stop')">⛔ STOP</button>
    </div>
    <div id="state">READY</div>
  </div>
  <div class="card">
    <h3><span class="hdot"></span>ZIGZAG MOW — GPS FIELD</h3>
    <div class="mowrow">
      <button class="sbtn" id="c1" onclick="setC(1)">C1 SET</button>
      <button class="sbtn" id="c2" onclick="setC(2)">C2 SET</button>
      <button class="sbtn" id="c3" onclick="setC(3)">C3 SET</button>
      <button class="sbtn" id="c4" onclick="setC(4)">C4 SET</button>
    </div>
    <div class="mowrow">
      <span class="pill">line-gap <input id="sp" type="number" value="1.0" step="0.1" min="0.4" max="3"> m</span>
      <button class="sbtn grn" onclick="mowStart()">🌾 MOW START</button>
      <button class="sbtn red" onclick="mowStop()">MOW STOP</button>
    </div>
    <canvas id="fmap" width="330" height="210"></canvas>
    <div class="hint" id="mst">Narzo haath me → kone pe khade ho → C SET → agla kona...</div>
  </div>
  <div class="card">
    <h3><span class="hdot"></span>LIVE VISION — YOLO (80-CHEEZEIN)</h3>
    <div class="vwrap"><img src="/video" id="vid" onerror="this.style.opacity=.12"><div class="scan"></div><div class="live">● LIVE</div><div class="det" id="det">kuch nahi dikha</div></div>
    <div class="vstrip">
      <span class="pill">cam <b id="src">-</b></span>
      <span class="pill"><b id="vf">0</b> fps</span>
      <span class="pill">total <b id="tot">0</b></span>
      <span class="pill">motor <b id="spd">-</b></span>
    </div>
  </div>
  <div class="card">
    <h3><span class="hdot"></span>MANUAL REMOTE</h3>
    <div class="tgl">
      <button class="tb on" id="bh" onclick="setR('hold')">✋ HOLD</button>
      <button class="tb" id="bt" onclick="setR('tap')">👆 TAP</button>
    </div>
    <div class="rmt">
      <span></span><button class="pad" ontouchstart="dn('F',event)" ontouchend="upp(event)" onmousedown="dn('F',null)" onmouseup="upp(null)">▲</button><span></span>
      <button class="pad" ontouchstart="dn('L',event)" ontouchend="upp(event)" onmousedown="dn('L',null)" onmouseup="upp(null)">◀</button>
      <button class="pad danger" ontouchstart="dn('S',event)" ontouchend="upp(event)" onmousedown="dn('S',null)" onmouseup="upp(null)">■</button>
      <button class="pad" ontouchstart="dn('R',event)" ontouchend="upp(event)" onmousedown="dn('R',null)" onmouseup="upp(null)">▶</button>
      <span></span><button class="pad" ontouchstart="dn('B',event)" ontouchend="upp(event)" onmousedown="dn('B',null)" onmouseup="upp(null)">▼</button><span></span>
    </div>
    <div class="rbtns">
      <button class="sbtn" onclick="send('2')">🐢 DHEERE</button>
      <button class="sbtn" onclick="send('1')">🐢 SLOW LOCKED</button>
      <button class="sbtn grn" onclick="send('K')">BLADE CUT</button>
      <button class="sbtn red" onclick="send('M')">BLADE ON</button>
    </div>
    <div class="hint" id="rep">(remote sirf MANUAL me • 5s chup = sab OFF)</div>
  </div>
 </div>
 <div class="col">
  <div class="card bladecard">
    <h3><span class="hdot"></span>BLADE SYSTEM</h3>
    <span id="saw">⚙️</span>
    <span id="blade" class="cut">CUT</span>
    <div class="gbox" style="margin-top:6px"><canvas id="gauge" width="210" height="118"></canvas><span>ACS CURRENT (A)</span></div>
  </div>
  <div class="card">
    <h3><span class="hdot"></span>TELEMETRY</h3>
    <div class="bl"><span>F — aage</span><b><span id="F">-</span> cm</b></div>
    <div class="bar"><div class="fill" id="bF" style="width:0%"></div></div>
    <div class="bl"><span>L — bayan</span><b><span id="L">-</span> cm</b></div>
    <div class="bar"><div class="fill" id="bL" style="width:0%"></div></div>
    <div class="bl"><span>R — dahina</span><b><span id="R">-</span> cm</b></div>
    <div class="bar"><div class="fill" id="bR" style="width:0%"></div></div>
    <div class="bl"><span>B — peechhe (min 5cm)</span><b><span id="B">-</span> cm</b></div>
    <div class="bar"><div class="fill" id="bB" style="width:0%"></div></div>
    <div class="bl"><span>C — camera (fixed)</span><b><span id="C">-</span> cm</b></div>
    <div class="bar"><div class="fill" id="bC" style="width:0%"></div></div>
    <div class="grid2">
      <span class="pill">RAIN <b id="RAIN">-</b></span>
      <span class="pill">MPU <b id="MPU">-</b></span>
      <span class="pill">PIT <b id="PIT">-</b>°</span>
      <span class="pill">ROL <b id="ROL">-</b>°</span>
    </div>
  </div>
  <div class="card">
    <h3><span class="hdot"></span>GPS — NARZO 50A</h3>
    <div class="gpsbig" id="gps">...</div>
    <div class="grid2">
      <span class="pill">accuracy <b id="GACC">-</b> m</span>
      <span class="pill">age <b id="GAGE">-</b> s</span>
    </div>
  </div>
  <div class="card">
    <h3><span class="hdot"></span>BRAIN LOG</h3>
    <div id="log">System ready. Waiting for logs...</div>
  </div>
 </div>
</div>
<button id="estop" onclick="estop()">E-STOP</button>
<div id="toast"></div>
<script>
var J={F:'-',L:'-',R:'-',B:'-'};
function t(msg){var e=document.getElementById('toast');e.textContent=msg;e.style.opacity=1;
clearTimeout(window._tt);window._tt=setTimeout(function(){e.style.opacity=0},2400)}
function send(x){fetch('/cmd?c='+x).then(function(r){return r.text()}).then(function(t2){
var e=document.getElementById('rep');e.textContent=t2;t(t2)}).catch(function(){})}
function setMode(m){fetch('/mode?m='+m).then(function(r){return r.text()}).then(function(){t('MODE -> '+m.toUpperCase())})}
function setC(i){fetch('/corner?i='+i).then(function(r){return r.text()}).then(function(t2){
t('C'+i+': '+t2);if(t2.indexOf('OK')>=0)document.getElementById('c'+i).classList.add('done')})}
function mowStart(){var sp=encodeURIComponent(document.getElementById('sp').value||'1.0');
fetch('/mow?a=start&sp='+sp).then(function(r){return r.text()}).then(function(t2){t('🌾 '+t2)})}
function mowStop(){fetch('/mow?a=stop').then(function(r){return r.text()}).then(function(t2){t('⛔ '+t2)})}
function estop(){fetch('/mow?a=stop').then(function(){return fetch('/cmd?c=K')}).then(function(){
t('🛑 EMERGENCY — SAB BAND, BLADE CUT!')})}
var RMODE='hold',held='',hb=null;
function setR(m){RMODE=m;upp(null);
document.getElementById('bh').className='tb'+(m=='hold'?' on':'');
document.getElementById('bt').className='tb'+(m=='tap'?' on':'')}
function tapGo(x){send(x);
if('FBLR'.indexOf(x)>=0){if(hb)clearInterval(hb);hb=setInterval(function(){send(x)},2000)}
else if(x=='S'&&hb){clearInterval(hb);hb=null}}
function dn(x,ev){if(ev)ev.preventDefault();
if(RMODE=='tap'){tapGo(x);return}
document.querySelectorAll('.pad.on').forEach(function(el){el.classList.remove('on')});
if(hb)clearInterval(hb);
held=x;if(ev&&ev.currentTarget)ev.currentTarget.classList.add('on');
send(x);hb=setInterval(function(){send(x)},300)}
function upp(ev){if(ev)ev.preventDefault();
if(hb){clearInterval(hb);hb=null}
document.querySelectorAll('.pad.on').forEach(function(el){el.classList.remove('on')});
if(held){held='';send('S')}}
document.addEventListener('mouseup',function(){upp(null)});
window.addEventListener('blur',function(){upp(null)});
document.addEventListener('contextmenu',function(e){e.preventDefault()});
document.addEventListener('selectstart',function(e){e.preventDefault()});
document.addEventListener('dragstart',function(e){e.preventDefault()});
function barF(id,v,hard){var n=parseFloat(v);var e=document.getElementById(id);
if(isNaN(n)||n>900){e.style.width='0%';e.className='fill';return}
e.style.width=Math.max(2,Math.min(100,n/150*100))+'%';
e.className='fill'+(n<hard?' r':'')}
function num(id,v){document.getElementById(id).textContent=v}
function upStr(s){s=parseInt(s)||0;var h=(s/3600)|0,m=((s%3600)/60)|0,x=s%60;
return (h?h+'h ':'')+m+'m '+x+'s'}
/* FIELD MAP (zigzag) */
var FC=document.getElementById('fmap'),FX=FC.getContext('2d');
function fmap(){
var w=FC.width,h=FC.height;FX.clearRect(0,0,w,h);
var C=J.MCC||[],W=J.MWPTS||[],R=J.MROT||[];
FX.fillStyle='rgba(52,211,153,.05)';FX.fillRect(0,0,w,h);
if(C.length<4){FX.fillStyle='#7d93b8';FX.font='11px Segoe UI';FX.textAlign='center';
FX.fillText('4 corner set hone ka intezar...',w/2,h/2);return}
var pts=C.concat(W);if(R.length==2)pts=pts.concat([R]);
var las=pts.map(function(p){return p[0]}),los=pts.map(function(p){return p[1]});
var mla=Math.min.apply(null,las),Mla=Math.max.apply(null,las);
var mlo=Math.min.apply(null,los),Mlo=Math.max.apply(null,los);
var pad=26,dx=(Mlo-mlo)||1e-6,dy=(Mla-mla)||1e-6;
var s=Math.min((w-2*pad)/dx,(h-2*pad)/dy);
var ox=(w-s*dx)/2,oy=(h-s*dy)/2;
function X(p){return ox+(p[1]-mlo)*s}
function Y(p){return oy+(Mla-p[0])*s}
FX.strokeStyle='rgba(163,230,53,.75)';FX.lineWidth=2;FX.setLineDash([5,4]);
FX.beginPath();W.forEach(function(p,i){i?FX.lineTo(X(p),Y(p)):FX.moveTo(X(p),Y(p))});FX.stroke();
FX.setLineDash([]);
C.forEach(function(p,i){FX.fillStyle='#38bdf8';FX.fillRect(X(p)-5,Y(p)-5,10,10);
FX.fillStyle='#e7eefc';FX.font='bold 10px Segoe UI';FX.textAlign='center';
FX.fillText('C'+(i+1),X(p),Y(p)-9)});
if(J.MWI<J.MWPTS.length&&W[J.MWI]){FX.strokeStyle='#fbbf24';FX.lineWidth=2;
FX.beginPath();FX.arc(X(W[J.MWI]),Y(W[J.MWI]),8,0,6.283);FX.stroke()}
if(R.length==2){FX.fillStyle='#34d399';FX.beginPath();FX.arc(X(R),Y(R),6,0,6.283);FX.fill();
FX.strokeStyle='rgba(52,211,153,.4)';FX.beginPath();FX.arc(X(R),Y(R),11,0,6.283);FX.stroke()}}
/* GAUGE */
var GC=document.getElementById('gauge'),GX=GC.getContext('2d');
function gauge(){var w=GC.width,h=GC.height,cx=w/2,cy=h-12,R=80;
GX.clearRect(0,0,w,h);
var a0=Math.PI,a1=2*Math.PI;
GX.lineWidth=13;GX.lineCap='round';
GX.strokeStyle='rgba(148,163,184,.15)';
GX.beginPath();GX.arc(cx,cy,R,a0,a1);GX.stroke();
var amps=parseFloat(J.ACS);if(isNaN(amps))amps=0;
if(amps<0)amps=0;if(amps>15)amps=15;
GX.strokeStyle=amps>12?'#f87171':(amps>8?'#fbbf24':'#34d399');
GX.beginPath();GX.arc(cx,cy,R,a0,a0+(amps/15)*Math.PI);GX.stroke();
GX.fillStyle='#e7eefc';GX.font='bold 21px Segoe UI';GX.textAlign='center';
GX.fillText(amps.toFixed(1),cx,cy-8);
GX.fillStyle='#7d93b8';GX.font='10px Segoe UI';
GX.fillText(amps>12?'OVERLOAD!':(amps>8?'bhaari':'theek'),cx,cy+8)}
setInterval(gauge,200);
/* MAIN */
function f(){
fetch('/data').then(function(r){if(r.status==401){location='/';throw 0}return r.json()}).then(function(j){
J=j;
var det=(j.det_name=='-')?'kuch nahi dikha':(j.det_name.toUpperCase()+' '+Math.round(j.det_conf*100)+'% ['+j.det_zone+']');
num('det',det);num('src',j.cam_src);num('vf',j.vfps);num('tot',j.det_total);num('spd',j.spd);
num('F',j.F);num('L',j.L);num('R',j.R);num('B',j.B);
barF('bF',j.F,30);barF('bL',j.L,30);barF('bR',j.R,30);barF('bB',j.B,5);barF('bC',j.C,40);
num('RAIN',j.RAIN);num('MPU',j.MPU);num('PIT',j.PIT);num('ROL',j.ROL);num('C',j.C);
num('gps',j.LAT=='-'?'NARZO ka intezar...':(j.LAT+' , '+j.LON));
num('GACC',j.GACC);num('GAGE',j.GAGE);
num('up','⏱ '+upStr(j.UP));
document.getElementById('d_esp').className='dot '+(j.esp_age<=2?'ok':'bad');
document.getElementById('d_ard').className='dot '+((j.ard+'').indexOf('JUD')>=0?'ok':'bad');
document.getElementById('d_gps').className='dot '+(j.GAGE<=15?'ok':'bad');
['auto','mow','manual','stop'].forEach(function(m){
document.getElementById('m_'+m).className='segb'+(m=='manual'?' man':(m=='stop'?' stp':(m=='mow'?' mowb':'')))+(j.mode==m?' on':'')});
num('state',j.state);
if(j.MC>=4)for(var i=1;i<=4;i++)document.getElementById('c'+i).classList.add('done');
var mst=document.getElementById('mst');
if(j.MWN>0){mst.textContent='🌾 line '+Math.min(j.MWI+1,j.MWN)+'/'+j.MWN+' • gap '+j.MSP+'m'+(j.mode=='mow'?' • CHALU!':' • (START dabana hai)')}
else if(j.MC<4){mst.textContent='corners: '+j.MC+'/4 set — Narzo leke kone pe khao'}
else{mst.textContent='4 corners set ✓ — MOW START dabao!'}
fmap();
var bl=document.getElementById('blade'),sw=document.getElementById('saw');
if(j.blade=='ON'){bl.className='on'; bl.textContent='ON'; sw.innerHTML='<span class="spin">⚙️</span>'} else {bl.className='cut'; bl.textContent='CUT'; sw.textContent='⚙️'}

fetch('/log').then(function(r){return r.text()}).then(function(t2){
if(document.getElementById('log').textContent!==t2)document.getElementById('log').textContent=t2});
}).catch(function(){})}
setInterval(f,1000);f();
</script></body></html>'''

@app.route('/')
def home():
    if not authed():
        return LOGIN_PAGE
    return DASH_PAGE

# ---------------- MAIN ----------------
if __name__ == '__main__':
    ard, ard_path = find_arduino()
    if ard:
        S['ard'] = 'JUD GAYA (' + str(ard_path) + ')'
        ard_send('K'); ard_send('S')
        ard_send('C')
    else:
        S['ard'] = 'NAHI MILA — retry chalu'
    threading.Thread(target=arduino_keeper, daemon=True).start()
    esp, esp_path = find_esp32(ard_path)
    if esp:
        esp_send('C')
    threading.Thread(target=esp_loop, args=(esp,), daemon=True).start()
    threading.Thread(target=vision_loop, daemon=True).start()
    threading.Thread(target=gps_loop, daemon=True).start()
    threading.Thread(target=ard_status_loop, daemon=True).start()
    threading.Thread(target=brain_loop, daemon=True).start()
    threading.Thread(target=manual_watchdog, daemon=True).start()
    log('GrassGuard d5pi 1.3.2 chalu — CAM-US boss + ZIGZAG-MOW + MAX-speed')
    ip = get_local_ip() or '10.150.248.187'
    print()
    print('============================================')
    print('  GRASSGUARD d5pi 1.3.2 — PI BRAIN ON! (CAM-US fixed-radar)')
    print('  Arduino: %s | ESP32: %s' % (ard_path, esp_path))
    print('  Camera : %s' % S['cam_src'])
    print('  GPS    : %s' % NARZO_URL)
    print('  BROWSER: http://%s:5000' % ip)
    print('  LOGIN  : shubham27 / 270308')
    print('  ZIGZAG : dashboard -> 4 CORNER -> MOW START')
    print('  CAM-US : %s cm (fixed-radar, Arduino v5)' % S['C'])
    print('============================================')
    app.run(host='0.0.0.0', port=5000)
