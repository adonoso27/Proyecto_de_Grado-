#!/usr/bin/env python3
# Detecta cubos (verde/rojo/azul) y reporta color + dot únicamente cuando CAMBIA.
# - Mantiene las ventanas/overlays originales.
# - Envía JSON al bridge TCP (host/puerto abajo).
# - Escribe a Modbus TCP HR[128] con code = 10*color_id + (1 si dot else 0).

import json, os, time, socket
from collections import deque, Counter
import cv2
import numpy as np
import depthai as dai

# ===================== Parámetros de forma / estabilidad =====================
MIN_DIE_AREA   = 1500
WARP_SIZE      = 260
CENTER_TOL_FR  = 0.18
DOT_AREA_MIN   = 70
DOT_AREA_MAX   = 4000
DOT_CIRC_MIN   = 0.5
DOT_INT_MAX    = 120
VOTE_WINDOW    = 7   # solo para overlay suave, no para decidir

# ===================== Bridge TCP (JSON) =====================
RECEPTOR_HOST = "127.0.0.1"
RECEPTOR_PORT = 5555

def send_event_json(color_name:str, dot:bool, host=RECEPTOR_HOST, port=RECEPTOR_PORT):
    payload = {"color": color_name, "dot": bool(dot)}
    try:
        with socket.create_connection((host, port), timeout=0.5) as s:
            s.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        print(f"➡️  JSON → {host}:{port}: {payload}")
    except Exception as e:
        print(f"⚠️  JSON no enviado ({e})")

# ===================== Modbus TCP (cliente) =====================
MODBUS_HOST = "127.0.0.1"
MODBUS_PORT = 502
MODBUS_ADDR = 128

# Compatibilidad pymodbus 2.x / 3.x
try:
    from pymodbus.client.sync import ModbusTcpClient  # 2.x
except Exception:
    from pymodbus.client import ModbusTcpClient       # 3.x

_mb_client = None

def modbus_connect(host=MODBUS_HOST, port=MODBUS_PORT):
    global _mb_client
    if _mb_client is None:
        _mb_client = ModbusTcpClient(host=host, port=port)
    if not _mb_client.connect():
        print(f"⚠️  No conecta Modbus {host}:{port}")
        return False
    return True

def modbus_write_code(code:int, addr=MODBUS_ADDR):
    try:
        if not modbus_connect():
            return False
        rr = _mb_client.write_register(addr, int(code), unit=1)  # unit=1 por convención
        if hasattr(rr, "isError") and rr.isError():
            print(f"⚠️  Error write HR[{addr}]={code}: {rr}")
            return False
        print(f"🔌 Modbus HR[{addr}] = {code}")
        return True
    except Exception as e:
        print(f"⚠️  Modbus no escrito: {e}")
        return False

# ===================== HSV defaults =====================
DEF_PREFS = {
    "green": {"lower": [41, 66, 82],  "upper": [85, 148, 255]},
    "red":   {"lower": [16,103, 94],  "upper": [14, 214, 255]},  # wrap
    "blue":  {"lower": [90, 80, 60],  "upper": [130, 255, 255]},
}
PREFS_PATH = "hsv_range.json"

COLOR_NAME_TO_ID = {"none":0, "green":1, "red":2, "blue":3}
COLOR_ID_TO_NAME = {v:k for k,v in COLOR_NAME_TO_ID.items()}

def load_hsv(path=PREFS_PATH):
    if os.path.exists(path):
        try:
            with open(path,"r") as f:
                data = json.load(f)
            for k in ("green","red","blue"):
                if k not in data: data[k] = DEF_PREFS[k]
                lo = data[k].get("lower", DEF_PREFS[k]["lower"])
                hi = data[k].get("upper", DEF_PREFS[k]["upper"])
                data[k]["lower"] = [int(lo[0]), int(lo[1]), int(lo[2])]
                data[k]["upper"] = [int(hi[0]), int(hi[1]), int(hi[2])]
            return data
        except Exception:
            pass
    return DEF_PREFS.copy()

def hsv_mask(hsv_img, lo, hi, wrap=False):
    lo = [int(lo[0]), int(lo[1]), int(lo[2])]
    hi = [int(hi[0]), int(hi[1]), int(hi[2])]
    lo[1] = max(0, min(lo[1], hi[1]))
    lo[2] = max(0, min(lo[2], hi[2]))
    if wrap and lo[0] > hi[0]:
        lower1 = np.array([lo[0], lo[1], lo[2]], dtype=np.uint8)
        upper1 = np.array([179,   hi[1], hi[2]], dtype=np.uint8)
        lower2 = np.array([0,     lo[1], lo[2]], dtype=np.uint8)
        upper2 = np.array([hi[0], hi[1], hi[2]], dtype=np.uint8)
        m1 = cv2.inRange(hsv_img, lower1, upper1)
        m2 = cv2.inRange(hsv_img, lower2, upper2)
        mask = cv2.bitwise_or(m1, m2)
    else:
        mask = cv2.inRange(hsv_img, np.array(lo, dtype=np.uint8), np.array(hi, dtype=np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
    return mask

# ===================== Geometría / warping =====================
def most_centered_contour(cnts, w, h, min_area=MIN_DIE_AREA):
    if not cnts: return None, -1e9
    cx0, cy0 = w//2, h//2
    best, best_score = None, -1e9
    for c in cnts:
        a = cv2.contourArea(c)
        if a < min_area: continue
        x,y,ww,hh = cv2.boundingRect(c)
        cx, cy = x + ww//2, y + hh//2
        score = a - 0.6*np.hypot(cx-cx0, cy-cy0)
        if score > best_score:
            best, best_score = c, score
    return best, best_score

def order_box_pts(pts):
    pts = np.array(pts, dtype=np.float32)
    s = pts.sum(axis=1); d = np.diff(pts, axis=1).ravel()
    tl = pts[np.argmin(s)]; br = pts[np.argmax(s)]
    tr = pts[np.argmin(d)]; bl = pts[np.argmax(d)]
    return np.stack([tl,tr,br,bl]).astype(np.float32)

def warp_die(bgr, contour, size=WARP_SIZE):
    rect = cv2.minAreaRect(contour)
    box  = cv2.boxPoints(rect)
    M = cv2.getPerspectiveTransform(
        order_box_pts(box),
        np.array([[0,0],[size-1,0],[size-1,size-1],[0,size-1]], np.float32)
    )
    warp = cv2.warpPerspective(bgr, M, (size, size))
    return warp, box.astype(int)

# ===================== Punto central =====================
def detect_center_dot(warp_gray):
    vis = cv2.cvtColor(warp_gray, cv2.COLOR_GRAY2BGR)
    clahe= cv2.createCLAHE(2.0, (8,8))
    blur = cv2.GaussianBlur(clahe.apply(warp_gray), (5,5), 0)
    bw   = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                 cv2.THRESH_BINARY_INV, 31, 7)
    cnts,_ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    H, W = warp_gray.shape[:2]
    cx0, cy0 = W//2, H//2
    max_dist = CENTER_TOL_FR * W

    best = None
    best_score = -1e9
    for c in cnts:
        area = cv2.contourArea(c)
        if not (DOT_AREA_MIN <= area <= DOT_AREA_MAX): continue
        per = cv2.arcLength(c, True)
        if per == 0: continue
        circ = 4*np.pi*area/(per*per)
        if circ < DOT_CIRC_MIN: continue

        x,y,w,h = cv2.boundingRect(c)
        roi = warp_gray[y:y+h, x:x+w]
        if roi.size == 0: continue
        if int(roi.mean()) > DOT_INT_MAX: continue

        cx, cy = x + w//2, y + h//2
        dist = np.hypot(cx-cx0, cy-cy0)
        if dist > max_dist: continue

        score = (circ * 2.0) + (area / 300.0) - (dist * 1.2)
        if score > best_score:
            best = (cx, cy, area, circ)
            best_score = score

    has_dot = best is not None
    if has_dot:
        cx, cy, area, circ = best
        cv2.circle(vis, (cx,cy), 12, (0,255,0), 2)
        cv2.circle(vis, (W//2,H//2), int(max_dist), (255,255,0), 1)
        cv2.putText(vis, f"dot OK  A={int(area)} C={circ:.2f}", (8,22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    else:
        cv2.circle(vis, (W//2,H//2), int(max_dist), (0,0,255), 1)
        cv2.putText(vis, "sin punto", (8,22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
    return has_dot, vis

# ===================== Pipeline RGB =====================
def make_pipeline():
    p = dai.Pipeline()
    cam = p.createColorCamera()
    cam.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setPreviewSize(640, 360)
    cam.setInterleaved(False)
    cam.setFps(10)
    xout = p.createXLinkOut()
    xout.setStreamName("rgb")
    cam.preview.link(xout.input)
    return p

# ===================== Detección de color + dot por frame =====================
def detect_color_and_dot_from_frame(frame, prefs):
    """Devuelve (color_id, color_name, has_dot, show, mask_vis, vis_face, die_found:bool)"""
    H,W,_ = frame.shape
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    masks = {}
    cnt_choice = {}
    score_choice = {}

    masks["green"] = hsv_mask(hsv, prefs["green"]["lower"], prefs["green"]["upper"], wrap=False)
    cnt_g, score_g = cv2.findContours(masks["green"], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], None
    cnt_g, score_g = most_centered_contour(cnt_g, W, H)

    masks["red"] = hsv_mask(hsv, prefs["red"]["lower"], prefs["red"]["upper"], wrap=True)
    cnt_r, score_r = cv2.findContours(masks["red"], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], None
    cnt_r, score_r = most_centered_contour(cnt_r, W, H)

    masks["blue"] = hsv_mask(hsv, prefs["blue"]["lower"], prefs["blue"]["upper"], wrap=False)
    cnt_b, score_b = cv2.findContours(masks["blue"], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], None
    cnt_b, score_b = most_centered_contour(cnt_b, W, H)

    choices = [("green", cnt_g, score_g), ("red", cnt_r, score_r), ("blue", cnt_b, score_b)]
    best_color, best_cnt, best_score = None, None, -1e9
    for c, cnt, sc in choices:
        if cnt is not None and sc > best_score:
            best_color, best_cnt, best_score = c, cnt, sc

    show = frame.copy()
    vis_face = None
    mask_vis = None
    has_dot = False
    die_found = best_cnt is not None

    if best_cnt is not None:
        x,y,ww,hh = cv2.boundingRect(best_cnt)
        cv2.rectangle(show, (x,y), (x+ww,y+hh), (0,255,0), 2)
        warp_bgr, box = warp_die(frame, best_cnt, WARP_SIZE)
        cv2.drawContours(show, [box], 0, (255,255,255), 2)
        warp_gray = cv2.cvtColor(warp_bgr, cv2.COLOR_BGR2GRAY)
        has_dot, vis_face = detect_center_dot(warp_gray)
        mask_vis = cv2.cvtColor(masks[best_color], cv2.COLOR_GRAY2BGR)
        color_name = best_color
        color_id   = COLOR_NAME_TO_ID.get(best_color, 0)
        return color_id, color_name, has_dot, show, mask_vis, vis_face, die_found
    else:
        cv2.putText(show, "Buscando cubo…", (12,28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
        return 0, "none", False, show, None, None, die_found

# ===================== Main =====================
if __name__ == "__main__":
    prefs = load_hsv()

    pipeline = make_pipeline()
    cfg = dai.Device.Config()
    cfg.board.usb.maxSpeed = dai.UsbSpeed.HIGH
    cfg.nonExclusiveMode = True

    # Estado previo para emitir solo en cambios
    prev_color_id = None
    prev_dot = None

    # Suavizado visual (no afecta decisión)
    color_votes = deque(maxlen=VOTE_WINDOW)

    with dai.Device(cfg) as dev:
        print("🧊 Detector de cubos (cambio-only) + JSON + Modbus (q para salir)")
        dev.startPipeline(pipeline)
        q = dev.getOutputQueue("rgb", maxSize=4, blocking=False)

        while True:
            msg = q.tryGet()
            if msg is None:
                if cv2.waitKey(1) & 0xFF == ord('q'): break
                continue

            frame = msg.getCvFrame()
            color_id, color_name, has_dot, show, mask_vis, vis_face, die_found = \
                detect_color_and_dot_from_frame(frame, prefs)

            # Overlay continuo
            if color_name != "none":
                color_votes.append(color_name)
                stable_color = max(set(color_votes), key=color_votes.count)
            else:
                stable_color = "none"
            status_txt = f"Color={stable_color} | dot={'SI' if has_dot else 'NO'}"
            cv2.putText(show, status_txt, (12,28), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0,255,255) if has_dot else (255,255,255), 2)

            if mask_vis is not None:
                stacked = np.hstack([show, mask_vis])
            else:
                stacked = show
            cv2.imshow("Original + Mascara", stacked)
            if vis_face is not None:
                cv2.imshow("Cara rectificada", vis_face)

            # ===== Emitir solo si CAMBIA (color_id, dot) =====
            # (Imprime como "c1,d1" p.ej. "0,0" → none/sin punto; "1,0" → verde sin punto, etc.)
            if (color_id != prev_color_id) or (has_dot != prev_dot):
                # Imprimir cambio
                print(f"{color_id},{1 if has_dot else 0}")

                # Enviar JSON al bridge
                send_event_json(color_name, has_dot)

                # Escribir Modbus: code = 10*color_id + (1 si dot else 0)
                code = int(10*color_id + (1 if has_dot else 0))
                modbus_write_code(code)

                prev_color_id = color_id
                prev_dot = has_dot

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
        # Cerrar cliente Modbus si quedó abierto
        try:
            if _mb_client: _mb_client.close()
        except Exception:
            pass
