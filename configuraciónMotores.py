#!/usr/bin/env python3
import time
import threading
from collections import deque
from typing import Optional

from flask import Flask, request, jsonify, Response
import serial

# =========================
# RS485
# =========================
PORT = "/dev/ttyUSB0"
BAUDRATE = 38400
ADDR = 0x01

# =========================
# Constantes de control
# =========================
# F6 (velocidad)
SPD_ACC = 2  # 0..255

# F5 (posición) requiere speed y acc
POS_SPEED_RPM = 300  # 0..3000
POS_ACC = 2          # 0..255

# Encoder axis: 16384 counts/vuelta
COUNTS_PER_TURN = 16384

# =========================
# Estado / logs
# =========================
app = Flask(__name__)

state_lock = threading.Lock()
mode = "speed"   # "speed" o "position"
setpoint = 0     # rpm (signed) o grados

pid_lock = threading.Lock()
pid = {"kp": 220, "ki": 100, "kd": 270, "kv": 320}

log_lock = threading.Lock()
log_buf = deque(maxlen=400)

ser_lock = threading.Lock()
ser: Optional[serial.Serial] = None

# =========================
# Utilidades
# =========================
def log(msg: str):
    with log_lock:
        ts = time.strftime("%H:%M:%S")
        log_buf.append(f"[{ts}] {msg}")

def checksum8(payload: bytes) -> int:
    # CHECKSUM 8-bit = sum(bytes) & 0xFF
    return sum(payload) & 0xFF

def ensure_serial_open():
    global ser
    with ser_lock:
        if ser and ser.is_open:
            return
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
            write_timeout=0.2
        )
    log(f"INFO: abierto {PORT} @ {BAUDRATE}")
    # habilitar bus control al arrancar
    send_hex_with_optional_crc("FA 01 F3 01", add_crc=True, rx_len=16)

def close_serial():
    global ser
    with ser_lock:
        if ser:
            try:
                ser.close()
            except Exception:
                pass
            ser = None
    log("INFO: puerto cerrado")

def hex_to_bytes(hex_str: str) -> bytes:
    parts = hex_str.strip().split()
    return bytes(int(p, 16) for p in parts)

def bytes_to_hex(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def send_bytes(tx: bytes, rx_len: int = 16) -> bytes:
    ensure_serial_open()
    with ser_lock:
        assert ser is not None
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        ser.write(tx)
        ser.flush()

        rx = ser.read(rx_len) if rx_len else b""

    log(f"TX: {bytes_to_hex(tx)}")
    log(f"RX: {bytes_to_hex(rx) if rx else '(vacío)'}")
    return rx

def send_hex_with_optional_crc(payload_hex: str, add_crc: bool, rx_len: int = 16) -> bytes:
    """
    payload_hex: bytes SIN CRC (p.ej. "FA 01 F6 00 32 02")
    add_crc=True: calcula checksum8 y lo añade
    """
    payload = hex_to_bytes(payload_hex)
    if add_crc:
        crc = checksum8(payload)
        tx = payload + bytes([crc])
    else:
        tx = payload
    return send_bytes(tx, rx_len=rx_len)

# =========================
# Builders de comandos
# =========================
def send_f6_speed(rpm_signed: int):
    rpm_signed = int(rpm_signed)
    if rpm_signed < 0:
        direction = 1
        speed = -rpm_signed
    else:
        direction = 0
        speed = rpm_signed

    speed = max(0, min(3000, int(speed)))
    acc = max(0, min(255, int(SPD_ACC)))

    byte4 = ((direction & 0x01) << 7) | ((speed >> 8) & 0x0F)
    byte5 = speed & 0xFF

    # payload sin CRC:
    payload_hex = f"FA {ADDR:02X} F6 {byte4:02X} {byte5:02X} {acc:02X}"
    send_hex_with_optional_crc(payload_hex, add_crc=True, rx_len=16)

def degrees_to_axis(deg: int) -> int:
    return int(round(int(deg) * COUNTS_PER_TURN / 360.0))

def send_f5_position(deg: int):
    deg = int(deg)
    speed = max(0, min(3000, int(POS_SPEED_RPM)))
    acc = max(0, min(255, int(POS_ACC)))
    axis = degrees_to_axis(deg)

    # payload: FA addr F5 speedHi speedLo acc axis(4B, signed, BE) + CRC
    speed_hi = (speed >> 8) & 0xFF
    speed_lo = speed & 0xFF
    axis_bytes = axis.to_bytes(4, byteorder="big", signed=True)

    payload = bytes([0xFA, ADDR, 0xF5, speed_hi, speed_lo, acc]) + axis_bytes
    tx = payload + bytes([checksum8(payload)])
    send_bytes(tx, rx_len=16)
    log(f"INFO: F5 angle={deg}° -> absAxis={axis} (16384/turn), speed={speed}, acc={acc}")

def send_pid_vfoc(kp: int, ki: int, kd: int, kv: int):
    # 96h: Kp Ki Kd Kv como uint16 big-endian
    for name, val in (("kp", kp), ("ki", ki), ("kd", kd), ("kv", kv)):
        if not (0 <= val <= 1024):
            raise ValueError(f"{name} fuera de rango (0..1024)")

    data = (
        int(kp).to_bytes(2, "big", signed=False) +
        int(ki).to_bytes(2, "big", signed=False) +
        int(kd).to_bytes(2, "big", signed=False) +
        int(kv).to_bytes(2, "big", signed=False)
    )
    payload = bytes([0xFA, ADDR, 0x96]) + data
    tx = payload + bytes([checksum8(payload)])
    send_bytes(tx, rx_len=16)

# =========================
# API
# =========================
@app.get("/api/status")
def api_status():
    with state_lock, pid_lock:
        return jsonify({
            "port": PORT,
            "baudrate": BAUDRATE,
            "addr": ADDR,
            "mode": mode,
            "setpoint": setpoint,
            "pid": dict(pid),
            "spd_acc": SPD_ACC,
            "pos_speed_rpm": POS_SPEED_RPM,
            "pos_acc": POS_ACC,
        })

@app.get("/api/log")
def api_log():
    with log_lock:
        return jsonify({"lines": list(log_buf)})

@app.post("/api/mode")
def api_mode():
    global mode, setpoint
    data = request.get_json(force=True, silent=True) or {}
    m = data.get("mode")
    if m not in ("speed", "position"):
        return jsonify({"ok": False, "error": "mode must be 'speed' or 'position'"}), 400
    with state_lock:
        mode = m
        setpoint = 0
    log(f"INFO: modo -> {mode}")
    # al cambiar modo, envía consigna 0 (segura)
    try:
        if mode == "speed":
            send_f6_speed(0)
        else:
            send_f5_position(0)
    except Exception as e:
        log(f"ERROR: al cambiar modo: {e}")
    return jsonify({"ok": True})

@app.post("/api/setpoint")
def api_setpoint():
    global setpoint
    data = request.get_json(force=True, silent=True) or {}
    v = int(data.get("value", 0))

    with state_lock:
        m = mode
        if m == "speed":
            if v < -100 or v > 100:
                return jsonify({"ok": False, "error": "speed out of range (-100..100)"}), 400
            setpoint = v
        else:
            if v < -360 or v > 360:
                return jsonify({"ok": False, "error": "position out of range (-360..360)"}), 400
            setpoint = v

    try:
        if m == "speed":
            send_f6_speed(setpoint)
        else:
            send_f5_position(setpoint)
    except Exception as e:
        log(f"ERROR: setpoint send failed: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500

    return jsonify({"ok": True})

@app.post("/api/pid")
def api_pid():
    data = request.get_json(force=True, silent=True) or {}
    try:
        kp = int(data.get("kp"))
        ki = int(data.get("ki"))
        kd = int(data.get("kd"))
        kv = int(data.get("kv"))
    except Exception:
        return jsonify({"ok": False, "error": "invalid pid payload"}), 400

    try:
        send_pid_vfoc(kp, ki, kd, kv)
    except Exception as e:
        log(f"ERROR: pid send failed: {e}")
        return jsonify({"ok": False, "error": str(e)}), 400

    with pid_lock:
        pid.update({"kp": kp, "ki": ki, "kd": kd, "kv": kv})
    log(f"INFO: PID aplicado: Kp={kp}, Ki={ki}, Kd={kd}, Kv={kv}")
    return jsonify({"ok": True})

# =========================
# UI WEB
# =========================
HTML = r"""
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>SERVO42D Web</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 16px; }
    .row { display: flex; gap: 16px; flex-wrap: wrap; }
    .card { border: 1px solid #ccc; border-radius: 10px; padding: 14px; flex: 1; min-width: 320px; }
    .bigbtn {
      width: 100%;
      font-size: 22px;
      padding: 18px;
      border-radius: 12px;
      border: 1px solid #333;
      cursor: pointer;
      margin-top: 10px;
    }
    .bigbtn.active { background: #111; color: #fff; }
    .slider { width: 100%; }
    .label { display: flex; justify-content: space-between; margin-top: 10px; }
    #console {
      width: 100%;
      height: 240px;
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace;
      white-space: pre;
      overflow: auto;
      border: 1px solid #ccc;
      border-radius: 10px;
      padding: 10px;
      background: #fafafa;
    }
    .muted { color: #666; font-size: 12px; }
  </style>
</head>
<body>
  <h2>SERVO42D – Control Web (mínimo)</h2>

  <div class="row">
    <div class="card">
      <div><b>Modo</b></div>
      <button id="btnSpeed" class="bigbtn">Modo Velocidad</button>
      <button id="btnPos" class="bigbtn">Modo Posición</button>
      <div class="muted">Selecciona modo y usa el slider único.</div>
    </div>

    <div class="card">
      <div><b>Selector (según modo)</b></div>
      <div class="label"><span id="spName">—</span><span id="spVal">—</span></div>
      <input id="spSlider" class="slider" type="range" min="-100" max="100" step="1" value="0"/>
      <div class="muted">
        Velocidad: -100..+100 RPM (F6). Posición: -360..+360° (F5).
      </div>
    </div>
  </div>

  <div class="row" style="margin-top: 16px;">
    <div class="card">
      <div><b>PID vFOC (96h)</b></div>

      <div class="label"><span>Kp</span><span id="kpVal">0</span></div>
      <input id="kp" class="slider" type="range" min="0" max="1024" step="1" value="220"/>

      <div class="label"><span>Ki</span><span id="kiVal">0</span></div>
      <input id="ki" class="slider" type="range" min="0" max="1024" step="1" value="100"/>

      <div class="label"><span>Kd</span><span id="kdVal">0</span></div>
      <input id="kd" class="slider" type="range" min="0" max="1024" step="1" value="270"/>

      <div class="label"><span>Kv</span><span id="kvVal">0</span></div>
      <input id="kv" class="slider" type="range" min="0" max="1024" step="1" value="320"/>

      <button id="applyPid" class="bigbtn" style="font-size:18px; padding: 12px;">Aplicar PID</button>
    </div>

    <div class="card">
      <div><b>Consola (TX/RX)</b></div>
      <div id="console"></div>
      <div class="muted">Si RX sale vacío pero el motor ejecuta, la respuesta puede estar desactivada en el driver.</div>
    </div>
  </div>

<script>
function $(id){ return document.getElementById(id); }

let currentMode = "speed";
let spTimer = null;

async function api(path, method="GET", body=null) {
  const opt = { method, headers: {} };
  if (body !== null) {
    opt.headers["Content-Type"] = "application/json";
    opt.body = JSON.stringify(body);
  }
  const r = await fetch(path, opt);
  const j = await r.json();
  if (!r.ok) throw new Error(j.error || "API error");
  return j;
}

function setButtons() {
  $("btnSpeed").classList.toggle("active", currentMode === "speed");
  $("btnPos").classList.toggle("active", currentMode === "position");
}

function configureSetpointUI() {
  if (currentMode === "speed") {
    $("spName").textContent = "Velocidad objetivo (RPM)";
    $("spSlider").min = -100;
    $("spSlider").max = 100;
    $("spSlider").value = 0;
    $("spVal").textContent = "0";
  } else {
    $("spName").textContent = "Posición objetivo (°)";
    $("spSlider").min = -360;
    $("spSlider").max = 360;
    $("spSlider").value = 0;
    $("spVal").textContent = "0";
  }
}

async function refreshLog() {
  try {
    const j = await api("/api/log");
    $("console").textContent = j.lines.join("\n");
    $("console").scrollTop = $("console").scrollHeight;
  } catch(e) {}
}

function bindPidLabels() {
  const keys = ["kp","ki","kd","kv"];
  for (const k of keys) {
    $(k+"Val").textContent = $(k).value;
    $(k).addEventListener("input", () => $(k+"Val").textContent = $(k).value);
  }
}

function scheduleSetpointSend() {
  if (spTimer) clearTimeout(spTimer);
  spTimer = setTimeout(async () => {
    const v = parseInt($("spSlider").value, 10);
    try {
      await api("/api/setpoint", "POST", { value: v });
      await refreshLog();
    } catch(e) { console.error(e); }
  }, 120);
}

window.addEventListener("load", async () => {
  bindPidLabels();

  const st = await api("/api/status");
  currentMode = st.mode || "speed";
  setButtons();
  configureSetpointUI();

  $("btnSpeed").addEventListener("click", async () => {
    currentMode = "speed";
    setButtons();
    configureSetpointUI();
    await api("/api/mode", "POST", { mode: "speed" });
    await refreshLog();
  });

  $("btnPos").addEventListener("click", async () => {
    currentMode = "position";
    setButtons();
    configureSetpointUI();
    await api("/api/mode", "POST", { mode: "position" });
    await refreshLog();
  });

  $("spSlider").addEventListener("input", () => {
    $("spVal").textContent = $("spSlider").value;
    scheduleSetpointSend();
  });

  $("applyPid").addEventListener("click", async () => {
    const body = {
      kp: parseInt($("kp").value, 10),
      ki: parseInt($("ki").value, 10),
      kd: parseInt($("kd").value, 10),
      kv: parseInt($("kv").value, 10),
    };
    await api("/api/pid", "POST", body);
    await refreshLog();
  });

  setInterval(refreshLog, 700);
  await refreshLog();
});
</script>
</body>
</html>
"""

@app.get("/")
def index():
    return Response(HTML, mimetype="text/html")

# =========================
# Main
# =========================
if __name__ == "__main__":
    try:
        ensure_serial_open()
    except Exception as e:
        log(f"ERROR: no se pudo abrir {PORT}: {e}")
    log("INFO: servidor web en http://0.0.0.0:5000")
    app.run(host="0.0.0.0", port=5000, debug=False)
