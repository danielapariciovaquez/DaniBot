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
# Constantes control
# =========================
# F6 (velocidad)
SPD_ACC = 2  # 0..255

# F5 (posición) requiere speed y acc
POS_SPEED_RPM = 300  # 0..3000
POS_ACC = 2          # 0..255

# Encoder axis
COUNTS_PER_TURN = 16384

# =========================
# App / estado
# =========================
app = Flask(__name__)

state_lock = threading.Lock()
mode = "speed"   # "speed" o "position"
setpoint = 0     # rpm signed o grados

pid_lock = threading.Lock()
pid = {"kp": 220, "ki": 100, "kd": 270, "kv": 320}

log_lock = threading.Lock()
log_buf = deque(maxlen=500)

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

def bytes_to_hex(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

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
    # Enable al arrancar
    send_enable(True)

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

def frame_down(code: int, data: bytes = b"") -> bytes:
    base = bytes([0xFA, ADDR & 0xFF, code & 0xFF]) + data
    return base + bytes([checksum8(base)])

def u16_be(v: int) -> bytes:
    return int(v).to_bytes(2, "big", signed=False)

def i32_be(v: int) -> bytes:
    return int(v).to_bytes(4, "big", signed=True)

def degrees_to_axis(deg: int) -> int:
    return int(round(int(deg) * COUNTS_PER_TURN / 360.0))


# =========================
# Comandos
# =========================
def send_enable(en: bool):
    tx = frame_down(0xF3, bytes([0x01 if en else 0x00]))
    send_bytes(tx, rx_len=16)

def send_speed_f6(rpm_signed: int):
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

    tx = frame_down(0xF6, bytes([byte4, byte5, acc]))
    send_bytes(tx, rx_len=16)

def send_stop():
    # STOP = F6 speed=0 acc=SPD_ACC -> CRC correcto calculado por frame_down
    send_speed_f6(0)

def send_position_f5(deg: int):
    deg = int(deg)
    deg = max(-360, min(360, deg))

    speed = max(0, min(3000, int(POS_SPEED_RPM)))
    acc = max(0, min(255, int(POS_ACC)))
    axis = degrees_to_axis(deg)

    data = u16_be(speed) + bytes([acc]) + i32_be(axis)
    tx = frame_down(0xF5, data)
    send_bytes(tx, rx_len=16)
    log(f"INFO: F5 angle={deg}° -> absAxis={axis} (16384/turn), speed={speed}, acc={acc}")

def send_pid_vfoc(kp: int, ki: int, kd: int, kv: int):
    for name, val in (("kp", kp), ("ki", ki), ("kd", kd), ("kv", kv)):
        if not (0 <= val <= 1024):
            raise ValueError(f"{name} fuera de rango (0..1024)")

    data = u16_be(kp) + u16_be(ki) + u16_be(kd) + u16_be(kv)
    tx = frame_down(0x96, data)
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

    try:
        # Al cambiar de modo, manda consigna 0 segura
        if mode == "speed":
            send_speed_f6(0)
        else:
            send_position_f5(0)
    except Exception as e:
        log(f"ERROR: cambio de modo: {e}")

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
            send_speed_f6(setpoint)
        else:
            send_position_f5(setpoint)
    except Exception as e:
        log(f"ERROR: setpoint send failed: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500

    return jsonify({"ok": True})

@app.post("/api/stop")
def api_stop():
    try:
        send_stop()
    except Exception as e:
        log(f"ERROR: stop failed: {e}")
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
    .bigbtn.stop { background: #b00020; color: #fff; border-color: #600; }
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
  <h2>SERVO42D – Control Web (completo)</h2>

  <div class="row">
    <div class="card">
      <div><b>Modo</b></div>
      <button id="btnSpeed" class="bigbtn">Modo Velocidad</button>
      <button id="btnPos" class="bigbtn">Modo Posición</button>
      <button id="btnStop" class="bigbtn stop">STOP</button>
      <div class="muted">STOP envía F6=0 (checksum correcto).</div>
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
      <div class="muted">RX puede salir vacío si el driver no responde, aunque ejecute.</div>
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

  $("btnStop").addEventListener("click", async () => {
    await api("/api/stop", "POST");
    // además resetea el slider a 0 por claridad
    $("spSlider").value = 0;
    $("spVal").textContent = "0";
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


if __name__ == "__main__":
    log("INFO: servidor web en http://0.0.0.0:5000")
    try:
        ensure_serial_open()
    except Exception as e:
        log(f"ERROR: no se pudo abrir {PORT}: {e}")
    app.run(host="0.0.0.0", port=5000, debug=False)
