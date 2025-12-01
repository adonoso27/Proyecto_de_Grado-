#!/usr/bin/env python3
# modbus_bridge_color_dot_min.py — Bridge Modbus TCP minimal
# Recibe {"color":"green|red|blue|none","dot":true|false} por TCP (líneas JSON)
# Publica SOLO:
#   HR[128] = color_id (0 none, 1 green, 2 red, 3 blue)
#   HR[129] = dot (0/1)
# (Opcional) Espejo en IR para depurar.
#
# Imprime cambios (color_id/dot) y repite publicación con periodo fijo.

import socket
import json
import threading
import time
from typing import Dict, Any

# --- Pymodbus 2.x ---
from pymodbus.server.sync import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext

# ===================== TCP RECEPCIÓN =====================
RX_HOST = "127.0.0.1"
RX_PORT = 5555

# ===================== MODBUS =====================
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 502
ADDR_COLOR  = 128
ADDR_DOT    = 129
POLL_DELAY  = 0.20  # s (periodo de publicación)

# ===================== MAPEO DE COLORES =====================
COLOR_NAME_TO_ID: Dict[str, int] = {
    "none": 0, "": 0, None: 0,
    "green": 1, "verde": 1,
    "red": 2, "rojo": 2,
    "blue": 3, "azul": 3,
}

# ===================== MODBUS CONTEXT =====================
store = ModbusSlaveContext(
    di=ModbusSequentialDataBlock(0, [0]*256),
    co=ModbusSequentialDataBlock(0, [0]*256),
    hr=ModbusSequentialDataBlock(0, [0]*256),
    ir=ModbusSequentialDataBlock(0, [0]*256),
)
context = ModbusServerContext(slaves=store, single=True)

identity = ModbusDeviceIdentification()
identity.VendorName = 'UR30_Modbus_Bridge'
identity.ProductCode = 'UR30'
identity.ProductName = 'Color/Dot Minimal Bridge'
identity.ModelName = 'UR30 Bridge v-min'
identity.MajorMinorRevision = '1.0'

# ===================== ESTADO COMPARTIDO =====================
state_lock = threading.Lock()
color_id = 0
dot_val  = 0
last_printed = {"color_id": None, "dot": None}

# ===================== UTILIDADES MODBUS =====================
def write_hr(addr: int, val: int):
    context[0].setValues(3, addr, [int(val)])

def write_ir(addr: int, val: int):
    context[0].setValues(4, addr, [int(val)])

def publish_both():
    """Publica HR e IR de color y dot."""
    with state_lock:
        write_hr(ADDR_COLOR, color_id)
        write_hr(ADDR_DOT,   dot_val)
        # espejos en IR para pruebas
        write_ir(ADDR_COLOR, color_id)
        write_ir(ADDR_DOT,   dot_val)

# ===================== PARSEO DE JSON =====================
def parse_payload(payload: Any):
    """
    Acepta:
      {"color": "green|red|blue|none", "dot": true|false}
      {"color_id": 0..3, "dot": 0|1|true|false}
      {"color": "red"}  (dot se asume 0)
    Devuelve (color_id:int 0..3, dot:int 0/1) o None si inválido.
    """
    try:
        if not isinstance(payload, dict):
            return None
        # color
        if "color_id" in payload:
            cid = int(payload.get("color_id", 0))
        else:
            cname = payload.get("color", "none")
            cid = COLOR_NAME_TO_ID.get(str(cname).strip().lower(), 0)
        # dot
        d_raw = payload.get("dot", 0)
        if isinstance(d_raw, bool):
            d = 1 if d_raw else 0
        else:
            d = 1 if int(d_raw) != 0 else 0

        if cid not in (0,1,2,3): cid = 0
        if d not in (0,1): d = 0
        return cid, d
    except Exception:
        return None

# ===================== TCP RECEPTOR (JSON líneas) =====================
def tcp_receiver():
    global color_id, dot_val
    print(f"📡 Receptor escuchando JSON en {RX_HOST}:{RX_PORT}\n")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((RX_HOST, RX_PORT))
        s.listen(1)
        while True:
            conn, addr = s.accept()
            print(f"🔗 Conexión desde {addr}")
            with conn:
                buf = b""
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        print("🔚 Cliente desconectado\n")
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        if not line:
                            continue
                        try:
                            payload = json.loads(line.decode("utf-8").strip())
                            parsed = parse_payload(payload)
                            if parsed is None:
                                print(f"⚠️  JSON inválido: {payload}")
                                continue
                            cid, d = parsed
                            changed = False
                            with state_lock:
                                if cid != color_id or d != dot_val:
                                    color_id = cid
                                    dot_val  = d
                                    changed = True
                            if changed:
                                print(f"✅ Evento: color_id={cid}, dot={d}")
                            else:
                                print(f"ℹ️  Repetido (sin cambio): color_id={cid}, dot={d}")
                        except Exception as e:
                            print(f"❌ JSON inválido: {e}")

# ===================== CONTROLADOR DE PUBLICACIÓN =====================
def modbus_controller():
    global last_printed
    print(f"🔌 Servidor ModbusTCP en {SERVER_HOST}:{SERVER_PORT}")
    print(f"📝 Publicando HR[{ADDR_COLOR}]=color_id y HR[{ADDR_DOT}]=dot\n")

    # Inicializa a ceros
    with state_lock:
        cid, dv = 0, 0
    write_hr(ADDR_COLOR, cid)
    write_hr(ADDR_DOT,   dv)
    write_ir(ADDR_COLOR, cid)
    write_ir(ADDR_DOT,   dv)
    last_printed = {"color_id": -1, "dot": -1}
    print("Modbus → color_id=0  dot=0")

    while True:
        try:
            publish_both()
            with state_lock:
                if color_id != last_printed["color_id"] or dot_val != last_printed["dot"]:
                    print(f"Modbus → color_id={color_id}  dot={dot_val}")
                    last_printed = {"color_id": color_id, "dot": dot_val}
            time.sleep(POLL_DELAY)
        except Exception as e:
            print(f"❌ Error en publicación: {e}")
            time.sleep(1)

# ===================== MAIN =====================
if __name__ == "__main__":
    threading.Thread(target=tcp_receiver,     daemon=True).start()
    threading.Thread(target=modbus_controller,daemon=True).start()
    StartTcpServer(context, identity=identity, address=(SERVER_HOST, SERVER_PORT))
