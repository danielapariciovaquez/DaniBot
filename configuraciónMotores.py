#!/usr/bin/env python3
import time
import threading
from collections import deque
from typing import Optional

from flask import Flask, request, jsonify, Response
import serial

# (Opcional) Algunos adaptadores RS485 requieren control RTS para DE/RE.
# Descomenta si sospechas que tu adaptador no conmuta automáticamente.
# from serial.rs485 import RS485Settings

# =========================
# CONFIG RS485
# =========================
PORT = "/dev/ttyUSB0"
BAUDRATE = 38400
ADDR = 0x01

# =========================
# CONTROL DEFAULTS
# =========================
# F5 (posición) requiere speed y acc en la trama.
POS_SPEED_RPM = 300   # 0..3000
POS_ACC = 2           # 0..255

# F6 (velocidad) usa ACC; lo fijamos.
SPD_ACC = 2           # 0..255

# Encoder multi-turn: 16384 counts / vuelta
COUNTS_PER_TURN = 16384

# =========================
# Estado global
# =========================
state_lock = threading.Lock()
mode = "speed"   # "speed" o "position"
setpoint = 0     # rpm signed si speed, grados si position
pid = {"kp": 220, "ki": 100, "kd": 270, "kv": 320}

log_lock = threading.Lock()
log_buf = deque(maxlen=400)

ser_lock = threading.Lock()
ser: Optional[serial.Serial] = None

app = Flask(__name__)

# =========================
# Protocolo MKS: CHECKSUM8
# =========================
def checksum8(payload: bytes) -> int:
    """
    CHECKSUM 8-bit = sum(all bytes) & 0xFF
    (No CRC16, no XOR, no complementos)
    """
    return sum(payload) & 0xFF

def frame_down(addr: int, code: int, data: bytes = b"") -> bytes:
    """
    Downlink: FA addr code data... checksum8
    """
    base = bytes([0xFA, addr & 0xFF, code & 0xFF]) + data
    return base + bytes([checksum8(base)])

def u16_be(v: int) -> bytes:
    return int(v).to_bytes(2, "big", signed=False)

def i32_be(v: int) -> bytes:
    return int(v).to_bytes(4, "big", signed=True)

def degrees_to_axis(deg: int) -> int:
    return int(round(int(deg) * COUNTS_PER_TURN / 360.0))

def hexs(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def log(msg: str):
    with log_lock:
        ts = time.strftime("%H:%M:%S")
        log_buf.append(f"[{ts}] {msg}")

# =========================
# Serie
# =========================
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
            timeout=0.25,
            write_timeout=0.25,
        )

        # Si tu adaptador necesita RTS/DE:
        # ser.rs485_mode = RS485Settings(
        #     rts_level_for_tx=True,
        #     rts_level_for_rx=False,
        #     delay_before_tx=0.001,
        #     delay_before_rx=0.001
        # )

    log(f"INFO: Puerto abierto {PORT} @ {BAUDRATE}")
    # Enable bus control al arrancar
    send_enable(True)

def serial_txrx(tx: bytes, rx_len: int = 5) -> bytes:
    """
    Envía una trama y lee respuesta (si existe).
    Muchas respuestas de 'set' son 5 bytes: FB addr code status checksum.
    """
    ensure_serial_open()
    with ser_lock:
        assert ser is not None
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        ser.write(tx)   # IMPORTANTE: trama completa, sin trocear
        ser.flush()

        rx = ser.read(rx_len) if rx_len else b""

    log(f"TX: {hexs(tx)}")
    log(f"RX: {hexs(rx) if rx else '(sin respuesta)'}")
    return rx

# =========================
# Comandos SERVO42D
# =========================
def send_enable(en: bool):
    tx = frame_down(ADDR, 0xF3, bytes([0x01 if en else 0x00]))
    serial_txrx(tx, rx_len=5)

def send_pid_vfoc(kp: int, ki: int, kd: int, kv: int):
    # 96h: PID vFOC: 4x uint16 big-endian
    data = u16_be(kp) + u16_be(ki) + u16_be(kd) + u16_be(kv)
    tx = frame_down(ADDR, 0x96, data)
    serial_txrx(tx, rx_len=5)

def send_speed_f6(rpm_signed: int):
    # F6: speed mode. Signo -> DIR bit7
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

    tx = frame_down(ADDR, 0xF6, bytes([byte4, byte5, acc]))
    serial_txrx(tx, rx_len=5)

def send_position_f5(deg: int):
    # F5: position mode4 absAxis
    deg = int(deg)
    deg = max(-360, min(360, deg))

    speed = max(0, min(3000, int(POS_SPEED_RPM)))
    acc = max(0, min(255, int(POS_ACC)))

    axis = degrees_to_axis(deg)
    data = u16_be(speed) + bytes([acc]) + i32_be(axis)

    tx = frame_down(ADDR, 0xF5, data)
    serial_txrx(tx, rx_len=5)
    log(f"INFO: F5 angle={deg}° -> absAxis={axis} (16384/turn), speed={speed}, acc={acc}")

# =========================
# API
# =========================
@app.get("/api/status")
def api_status():
    with state_lock:
        st = {
            "port": PORT,
            "baudrate": BAUDRATE,
            "addr": ADDR,
            "mode": mode,
            "setpoint": setpoint,
            "pid": dict(pid),
            "pos_speed_rpm": POS_SPEED_RPM,
            "pos_acc": POS_ACC,
            "spd_acc": SPD_ACC,
        }
    return jsonify(st)

@app.get("/api/log")
def api_log():
    with log_lock:
        return jsonify({"lines": list(log_buf)})

@app.post("/api/mode")
def api_mode():
    global mode
    data = request.get_json(force=True, silent=True) or {}
    m = data.get("mode")
    if m not in ("speed", "position"):
        return jsonify({"ok": False, "error": "mode must be 'speed' or 'position'"}), 400
    with state_lock:
        mode = m
    log(f"INFO: Modo -> {mode}")
    return jsonify({"ok": True})

@app.post("/api/setpoint")
def api_setpoint():
    global setpoint
    data = request.get_json(force=True, silent=True) or {}
    value = data.get("value", 0)

    with state_lock:
        m = mode
        if m == "speed":
            v = int(value)
            if v < -100 or v > 100:
                return jsonify({"ok": False, "error": "speed out of range (-100..100)"}), 400
            setpoint = v
        else:
            v = int(value)
            if v < -360 or v > 360:
                return jsonify({"ok": False, "error": "position out of range (-360..360)"}), 400
            setpoint = v

    # Enviar inmediatamente
    try:
        ensure_serial_open()
        if m == "speed":
            send_speed_f6(setpoint)
        else:
            send_position_f5(setpoint)
    except Exception as e:
        log(f"ERROR: setpoint send failed: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500

    return jsonify({"ok": True})

@app.post("/api/pid")
def api_pid():
    global pid
    data = request.get_json(force=True, silent=True) or {}
    try:
        kp = int(data.get("kp", pid["kp"]))
        ki = int(data.get("ki", pid["ki"]))
        kd = int(data.get("kd", pid["kd"]))
        kv = int(data.get("kv", pid["kv"]))
    except Exception:
        return jsonify({"ok": False, "error": "invalid pid values"}), 400

    for name, val in (("kp", kp), ("ki", ki), ("kd", kd), ("kv", kv)):
        if val < 0 or val > 1024:
            return jsonify({"ok": False, "error": f"{name} out of range (0..1024)"}), 400

    with state_lock:
        pid = {"kp": kp, "ki": ki, "kd": kd, "kv": kv}

    try:
        ensure_serial_open()
        send_pid_vfoc(kp, ki, kd, kv)
    except Exception as e:
        log(f"ERROR: pid send failed: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500

    return jsonify({"ok": True})

# =========================
# Web UI
# =========================
HTML = r"""
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>SERVO42D Web Control</title>
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
      height: 220px;
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
  <h2>SERVO42D – Control Web (FOC)</h2>

  <div class="row">
    <div class="card">
      <div><b>Modo</b></div>
      <button id="btnSpeed" class="bigbtn">Modo Velocidad</button>
      <button id="btnPos" class="bigbtn">Modo Posición</button>
      <div class="muted">Dos botones grandes para seleccionar el modo.</div>
    </div>

    <div class="card">
      <div><b>Selector (según modo)</b></div>
      <div id="spLabel" class="label"><span>—</span><span id="spVal">—</span></div>
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
      <div class="muted">Se envía 96h con Kp/Ki/Kd/Kv.</div>
    </div>

    <div class="card">
      <div><b>Consola (TX/RX)</b></div>
      <div id="console"></div>
      <div class="muted">Muestra tramas enviadas y respuesta (si el motor responde).</div>
    </div>
  </div>

<script>
let currentMode = "speed";

function $(id){ return document.getElementById(id); }

function setActiveButtons() {
  $("btnSpeed").classList.toggle("active", currentMode === "speed");
  $("btnPos").classList.toggle("active", currentMode === "position");
}

function configureSetpointUI() {
  if (currentMode === "speed") {
    $("spLabel").children[0].textContent = "Velocidad objetivo (RPM)";
    $("spSlider").min = -100;
    $("spSlider").max = 100;
    $("spSlider").value = 0;
    $("spVal").textContent = "0";
  } else {
    $("spLabel").children[0].textContent = "Posición objetivo (°)";
    $("spSlider").min = -360;
    $("spSlider").max = 360;
    $("spSlider").value = 0;
    $("spVal").textContent = "0";
  }
}

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

async function refreshLog() {
  try {
    const j = await api("/api/log");
    $("console").textContent = j.lines.join("\n");
    $("console").scrollTop = $("console").scrollHeight;
  } catch(e) {}
}

function bindPidLabels() {
  const ids = ["kp","ki","kd","kv"];
  for (const k of ids) {
    $(k+"Val").textContent = $(k).value;
    $(k).addEventListener("input", () => {
      $(k+"Val").textContent = $(k).value;
    });
  }
}

let spTimer = null;
function scheduleSetpointSend() {
  if (spTimer) clearTimeout(spTimer);
  spTimer = setTimeout(async () => {
    const v = parseInt($("spSlider").value, 10);
    try {
      await api("/api/setpoint", "POST", { value: v });
      await refreshLog();
    } catch(e) {
      console.error(e);
    }
  }, 120);
}

window.addEventListener("load", async () => {
  bindPidLabels();
  setActiveButtons();
  configureSetpointUI();

  $("btnSpeed").addEventListener("click", async () => {
    currentMode = "speed";
    setActiveButtons();
    configureSetpointUI();
    await api("/api/mode", "POST", { mode: "speed" });
    await refreshLog();
  });

  $("btnPos").addEventListener("click", async () => {
    currentMode = "position";
    setActiveButtons();
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

if __name__ == "__main__":
    try:
        ensure_serial_open()
    except Exception as e:
        log(f"ERROR: no se pudo abrir {PORT}: {e}")

    log("INFO: Servidor web iniciado en http://0.0.0.0:5000")
    app.run(host="0.0.0.0", port=5000, debug=False)
