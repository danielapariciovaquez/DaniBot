#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
from collections import deque
from typing import Optional, List, Dict, Any

import serial
from flask import Flask, request, jsonify, Response

# =============================
# 2. Configuración fija
# =============================
PORT = "/dev/ttyUSB0"
BAUDRATE = 38400
ADDR = 0x01

# Velocidad (F6)
SPD_ACC = 2  # 0..255 fijo

# Posición (F5)
POS_SPEED_RPM = 300  # 0..3000 fijo
POS_ACC = 2          # 0..255 fijo
COUNTS_PER_REV = 16384  # counts / 360°

# Recepción
RX_LEN = 16

# Logs
LOG_MAX_LINES = 400

# =============================
# 4. Estado interno
# =============================
state_lock = threading.Lock()
serial_lock = threading.Lock()
log_lock = threading.Lock()

mode = "speed"      # "speed" o "position"
setpoint = 0        # int: RPM o grados según mode
pid = {"kp": 0, "ki": 0, "kd": 0, "kv": 0}  # 0..1024

log_buf = deque(maxlen=LOG_MAX_LINES)

ser: Optional[serial.Serial] = None

# =============================
# Utilidades de logging
# =============================
def _log(line: str) -> None:
    ts = time.strftime("%H:%M:%S")
    with log_lock:
        log_buf.append(f"[{ts}] {line}")

def log_info(msg: str) -> None:
    _log(f"INFO: {msg}")

def log_error(msg: str) -> None:
    _log(f"ERROR: {msg}")

def bytes_to_hex(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

# =============================
# 5. Protocolo: checksum8
# =============================
def checksum8(frame_wo_checksum: List[int]) -> int:
    return sum(frame_wo_checksum) & 0xFF

def build_frame(cmd: int, data: List[int]) -> bytes:
    # [0xFA] [ADDR] [CMD] [DATA...] [CHK]
    frame = [0xFA, ADDR & 0xFF, cmd & 0xFF] + [x & 0xFF for x in data]
    chk = checksum8(frame)
    frame.append(chk)
    return bytes(frame)

# =============================
# 3.1 UART RS485: open-on-demand
# =============================
def _ensure_serial_open() -> serial.Serial:
    global ser
    # Nota: este helper se invoca SIEMPRE bajo serial_lock.
    if ser is not None and ser.is_open:
        return ser

    try:
        ser = serial.Serial(
            PORT,
            BAUDRATE,
            timeout=0.2,        # lectura RX con timeout (no bloqueante indefinido)
            write_timeout=0.2
        )
        log_info(f"Puerto serie abierto: {PORT} @ {BAUDRATE} bps, ADDR=0x{ADDR:02X}")
        # Enable automático al arrancar / reabrir
        send_enable_f3_locked(True)
        return ser
    except Exception as e:
        ser = None
        log_error(f"Fallo abriendo puerto serie {PORT}: {e}")
        raise

def _serial_write_read_locked(tx: bytes) -> bytes:
    """
    Envía TX y lee hasta RX_LEN bytes, sin asumir longitud exacta.
    Debe llamarse SIEMPRE bajo serial_lock.
    """
    s = _ensure_serial_open()

    # TX
    try:
        log_info(f"TX: {bytes_to_hex(tx)}")
        s.reset_input_buffer()
        s.write(tx)
        s.flush()
    except Exception as e:
        log_error(f"Fallo en escritura serie: {e}")
        raise

    # RX (puede ser vacío)
    try:
        rx = s.read(RX_LEN)
        if rx:
            log_info(f"RX: {bytes_to_hex(rx)}")
        else:
            log_info("RX: (vacío)")
        return rx
    except Exception as e:
        log_error(f"Fallo en lectura serie: {e}")
        raise

# =============================
# 6. Comandos
# =============================
def send_enable_f3_locked(enable: bool) -> None:
    # CMD=0xF3, DATA=[0x01] enable / [0x00] disable
    data = [0x01 if enable else 0x00]
    tx = build_frame(0xF3, data)
    _serial_write_read_locked(tx)

def send_speed_f6(rpm_signed: int) -> None:
    # Protege UART
    with serial_lock:
        # direction = 1 si RPM < 0, si no 0
        direction = 1 if rpm_signed < 0 else 0
        speed = abs(int(rpm_signed))

        # robustez 0..3000
        if speed < 0:
            speed = 0
        if speed > 3000:
            speed = 3000

        byte4 = ((direction & 0x01) << 7) | ((speed >> 8) & 0x0F)
        byte5 = speed & 0xFF
        acc = SPD_ACC & 0xFF

        tx = build_frame(0xF6, [byte4, byte5, acc])
        _serial_write_read_locked(tx)

def _int32_to_be_bytes(x: int) -> List[int]:
    # signed 32-bit big-endian (two's complement)
    x &= 0xFFFFFFFF
    return [(x >> 24) & 0xFF, (x >> 16) & 0xFF, (x >> 8) & 0xFF, x & 0xFF]

def send_position_f5(deg: int) -> None:
    # absAxis = round(deg * 16384 / 360)
    # deg int en [-360..360]
    with serial_lock:
        deg_i = int(deg)
        abs_axis = int(round(deg_i * COUNTS_PER_REV / 360.0))

        # speed u16 BE, acc u8, axis int32 BE
        speed = int(POS_SPEED_RPM)
        if speed < 0:
            speed = 0
        if speed > 3000:
            speed = 3000

        speed_hi = (speed >> 8) & 0xFF
        speed_lo = speed & 0xFF
        acc = POS_ACC & 0xFF

        axis_bytes = _int32_to_be_bytes(abs_axis)

        log_info(f"F5 conversion: deg={deg_i} -> absAxis={abs_axis} (counts, int32)")
        tx = build_frame(0xF5, [speed_hi, speed_lo, acc] + axis_bytes)
        _serial_write_read_locked(tx)

def send_pid_96(kp_v: int, ki_v: int, kd_v: int, kv_v: int) -> None:
    with serial_lock:
        def u16be(v: int) -> List[int]:
            v = int(v) & 0xFFFF
            return [(v >> 8) & 0xFF, v & 0xFF]

        tx = build_frame(0x96, u16be(kp_v) + u16be(ki_v) + u16be(kd_v) + u16be(kv_v))
        _serial_write_read_locked(tx)

# =============================
# Flask
# =============================
app = Flask(__name__)

HTML_PAGE = r"""
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>MKS SERVO42D RS485 Control</title>
  <style>
    :root {
      --bg: #0f172a;
      --card: #111c36;
      --border: #1e2a4a;
      --text: #e5e7eb;
      --muted: #a7b0c0;
      --btn: #223055;
      --btnActive: #2f4a9f;
      --danger: #9f2f2f;
      --ok: #2f9f6a;
    }
    body {
      margin: 0;
      font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Ubuntu, Cantarell, Noto Sans, Arial;
      background: var(--bg);
      color: var(--text);
    }
    .wrap {
      max-width: 980px;
      margin: 18px auto;
      padding: 0 14px 24px;
    }
    h1 {
      margin: 10px 0 18px;
      font-size: 18px;
      font-weight: 650;
      color: var(--text);
    }
    .grid {
      display: grid;
      grid-template-columns: 1fr;
      gap: 12px;
    }
    @media (min-width: 920px) {
      .grid { grid-template-columns: 1fr 1fr; }
      .span2 { grid-column: span 2; }
    }
    .card {
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 14px;
      box-shadow: 0 6px 22px rgba(0,0,0,0.25);
    }
    .card h2 {
      margin: 0 0 10px;
      font-size: 14px;
      font-weight: 650;
      color: var(--text);
    }
    .row {
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      align-items: center;
    }
    button {
      border: 1px solid var(--border);
      background: var(--btn);
      color: var(--text);
      padding: 12px 14px;
      border-radius: 10px;
      cursor: pointer;
      font-weight: 650;
      font-size: 14px;
      min-width: 160px;
    }
    button.active {
      background: var(--btnActive);
      border-color: #3b63ff;
    }
    button.stop {
      background: #381a1a;
      border-color: #6b1f1f;
      min-width: 160px;
    }
    button.stop:hover {
      filter: brightness(1.10);
    }
    button.apply {
      min-width: 160px;
    }
    .kv {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 8px 10px;
      align-items: center;
      margin-top: 8px;
    }
    .kv label {
      color: var(--muted);
      font-size: 13px;
    }
    .value {
      font-variant-numeric: tabular-nums;
      color: var(--text);
      font-weight: 650;
    }
    input[type="range"] {
      width: 100%;
      margin-top: 8px;
    }
    .mono {
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
    }
    pre {
      margin: 0;
      background: #0b1224;
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 12px;
      height: 300px;
      overflow: auto;
      color: #d1d5db;
      font-size: 12px;
      line-height: 1.35;
    }
    .hint {
      color: var(--muted);
      font-size: 12px;
      margin-top: 6px;
    }
  </style>
</head>

<body>
<div class="wrap">
  <h1>MKS SERVO42D — Control RS485 (Flask)</h1>

  <div class="grid">

    <div class="card">
      <h2>Modo de control</h2>
      <div class="row">
        <button id="btnSpeed" onclick="setMode('speed')">Modo Velocidad</button>
        <button id="btnPos" onclick="setMode('position')">Modo Posición</button>
        <button class="stop" id="btnStop" onclick="stopMotor()">STOP</button>
      </div>
      <div class="hint">STOP envía F6 a 0 rpm. Cambio de modo resetea setpoint a 0 y envía consigna segura.</div>
    </div>

    <div class="card">
      <h2>Setpoint</h2>
      <div class="kv">
        <label id="spLabel">Velocidad objetivo (RPM)</label>
        <div class="value mono" id="spValue">0</div>
      </div>
      <input id="spSlider" type="range" min="-100" max="100" step="1" value="0" oninput="onSliderInput()" />
      <div class="hint" id="spHint">Rango: [-100, +100] RPM</div>
    </div>

    <div class="card">
      <h2>PID vFOC (CMD 0x96)</h2>

      <div class="kv">
        <label>Kp</label><div class="value mono" id="kpVal">0</div>
        <input id="kp" type="range" min="0" max="1024" step="1" value="0" oninput="updPIDLabel('kp')"/>

        <label>Ki</label><div class="value mono" id="kiVal">0</div>
        <input id="ki" type="range" min="0" max="1024" step="1" value="0" oninput="updPIDLabel('ki')"/>

        <label>Kd</label><div class="value mono" id="kdVal">0</div>
        <input id="kd" type="range" min="0" max="1024" step="1" value="0" oninput="updPIDLabel('kd')"/>

        <label>Kv</label><div class="value mono" id="kvVal">0</div>
        <input id="kv" type="range" min="0" max="1024" step="1" value="0" oninput="updPIDLabel('kv')"/>
      </div>

      <div class="row" style="margin-top: 10px;">
        <button class="apply" onclick="applyPID()">Aplicar PID</button>
      </div>
      <div class="hint">Los sliders PID no envían nada hasta pulsar “Aplicar PID”.</div>
    </div>

    <div class="card span2">
      <h2>Consola TX/RX</h2>
      <pre id="console" class="mono"></pre>
      <div class="hint">Refresco ~700 ms. Auto-scroll al final.</div>
    </div>

  </div>
</div>

<script>
  let currentMode = "speed";
  let debounceTimer = null;

  async function apiGet(url) {
    const r = await fetch(url, {cache:"no-store"});
    const j = await r.json();
    if (!r.ok || j.ok === false) throw new Error(j.error || ("HTTP " + r.status));
    return j;
  }

  async function apiPost(url, bodyObj) {
    const r = await fetch(url, {
      method: "POST",
      headers: {"Content-Type":"application/json"},
      body: bodyObj ? JSON.stringify(bodyObj) : null
    });
    const j = await r.json();
    if (!r.ok || j.ok === false) throw new Error(j.error || ("HTTP " + r.status));
    return j;
  }

  function setActiveButtons() {
    document.getElementById("btnSpeed").classList.toggle("active", currentMode === "speed");
    document.getElementById("btnPos").classList.toggle("active", currentMode === "position");
  }

  function configureSetpointUI() {
    const sp = document.getElementById("spSlider");
    const lbl = document.getElementById("spLabel");
    const hint = document.getElementById("spHint");

    if (currentMode === "speed") {
      sp.min = -100; sp.max = 100; sp.step = 1;
      lbl.textContent = "Velocidad objetivo (RPM)";
      hint.textContent = "Rango: [-100, +100] RPM";
    } else {
      sp.min = -360; sp.max = 360; sp.step = 1;
      lbl.textContent = "Posición objetivo (°)";
      hint.textContent = "Rango: [-360, +360] °";
    }
    setActiveButtons();
  }

  function setSliderValue(v) {
    const sp = document.getElementById("spSlider");
    sp.value = v;
    document.getElementById("spValue").textContent = String(v);
  }

  function onSliderInput() {
    const v = parseInt(document.getElementById("spSlider").value);
    document.getElementById("spValue").textContent = String(v);

    // Debounce ~120ms
    if (debounceTimer) clearTimeout(debounceTimer);
    debounceTimer = setTimeout(async () => {
      try {
        await apiPost("/api/setpoint", {value: v});
      } catch (e) {
        // Si falla, se verá en log; no bloqueamos UI.
        console.warn(e);
      }
    }, 120);
  }

  async function setMode(m) {
    try {
      await apiPost("/api/mode", {mode: m});
      currentMode = m;
      configureSetpointUI();
      setSliderValue(0);
    } catch (e) {
      console.warn(e);
    }
  }

  async function stopMotor() {
    try {
      await apiPost("/api/stop", null);
      setSliderValue(0);
    } catch (e) {
      console.warn(e);
    }
  }

  function updPIDLabel(k) {
    const v = document.getElementById(k).value;
    document.getElementById(k + "Val").textContent = String(v);
  }

  async function applyPID() {
    try {
      const kp = parseInt(document.getElementById("kp").value);
      const ki = parseInt(document.getElementById("ki").value);
      const kd = parseInt(document.getElementById("kd").value);
      const kv = parseInt(document.getElementById("kv").value);
      await apiPost("/api/pid", {kp, ki, kd, kv});
    } catch (e) {
      console.warn(e);
    }
  }

  async function refreshLog() {
    try {
      const j = await apiGet("/api/log");
      const c = document.getElementById("console");
      c.textContent = j.lines.join("\n");
      c.scrollTop = c.scrollHeight;
    } catch (e) {
      // no-op
    }
  }

  async function init() {
    try {
      const st = await apiGet("/api/status");
      currentMode = st.mode;
      configureSetpointUI();
      setSliderValue(st.setpoint);

      // PID UI
      for (const k of ["kp","ki","kd","kv"]) {
        document.getElementById(k).value = st.pid[k];
        updPIDLabel(k);
      }
    } catch (e) {
      console.warn(e);
      configureSetpointUI();
      setSliderValue(0);
    }

    // polling log ~700ms
    await refreshLog();
    setInterval(refreshLog, 700);
  }

  init();
</script>
</body>
</html>
"""

# =============================
# 7. Endpoints
# =============================
@app.get("/")
def index() -> Response:
    return Response(HTML_PAGE, mimetype="text/html; charset=utf-8")

@app.get("/api/status")
def api_status():
    with state_lock:
        st = {
            "ok": True,
            "serial": {"port": PORT, "baudrate": BAUDRATE, "addr": ADDR},
            "mode": mode,
            "setpoint": setpoint,
            "pid": dict(pid),
            "constants": {
                "spd_acc": SPD_ACC,
                "pos_speed_rpm": POS_SPEED_RPM,
                "pos_acc": POS_ACC,
                "counts_per_rev": COUNTS_PER_REV,
                "rx_len": RX_LEN,
            }
        }
    return jsonify(st)

@app.get("/api/log")
def api_log():
    with log_lock:
        lines = list(log_buf)
    return jsonify({"ok": True, "lines": lines})

@app.post("/api/mode")
def api_mode():
    global mode, setpoint
    try:
        data = request.get_json(force=True, silent=False)
        m = data.get("mode", "")
        if m not in ("speed", "position"):
            return jsonify({"ok": False, "error": "mode inválido (speed|position)"}), 400

        with state_lock:
            mode = m
            setpoint = 0

        log_info(f"Modo cambiado a: {m}. Setpoint reseteado a 0.")

        # consigna segura
        if m == "speed":
            send_speed_f6(0)
        else:
            send_position_f5(0)

        return jsonify({"ok": True})
    except Exception as e:
        log_error(f"/api/mode error: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500

@app.post("/api/setpoint")
def api_setpoint():
    global setpoint
    try:
        data = request.get_json(force=True, silent=False)
        v = int(data.get("value"))

        with state_lock:
            m = mode

        if m == "speed":
            if v < -100 or v > 100:
                return jsonify({"ok": False, "error": "setpoint speed fuera de rango [-100..100]"}), 400
            with state_lock:
                setpoint = v
            send_speed_f6(v)
            return jsonify({"ok": True})

        if m == "position":
            if v < -360 or v > 360:
                return jsonify({"ok": False, "error": "setpoint position fuera de rango [-360..360]"}), 400
            with state_lock:
                setpoint = v
            send_position_f5(v)
            return jsonify({"ok": True})

        return jsonify({"ok": False, "error": "modo interno inválido"}), 500
    except Exception as e:
        log_error(f"/api/setpoint error: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500

@app.post("/api/stop")
def api_stop():
    global setpoint
    try:
        with state_lock:
            setpoint = 0
        log_info("STOP solicitado: enviando F6 a 0 rpm.")
        send_speed_f6(0)
        return jsonify({"ok": True})
    except Exception as e:
        log_error(f"/api/stop error: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500

@app.post("/api/pid")
def api_pid():
    global pid
    try:
        data = request.get_json(force=True, silent=False)
        kp_v = int(data.get("kp"))
        ki_v = int(data.get("ki"))
        kd_v = int(data.get("kd"))
        kv_v = int(data.get("kv"))

        for name, v in [("kp", kp_v), ("ki", ki_v), ("kd", kd_v), ("kv", kv_v)]:
            if v < 0 or v > 1024:
                return jsonify({"ok": False, "error": f"{name} fuera de rango [0..1024]"}), 400

        send_pid_96(kp_v, ki_v, kd_v, kv_v)

        with state_lock:
            pid = {"kp": kp_v, "ki": ki_v, "kd": kd_v, "kv": kv_v}

        log_info(f"PID aplicado: kp={kp_v}, ki={ki_v}, kd={kd_v}, kv={kv_v}")
        return jsonify({"ok": True})
    except Exception as e:
        log_error(f"/api/pid error: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500

# =============================
# Main
# =============================
def _startup_open_serial():
    # Apertura inicial + enable. Si falla, el servidor sigue y reintentará on-demand.
    try:
        with serial_lock:
            _ensure_serial_open()
    except Exception:
        pass

if __name__ == "__main__":
    log_info("Arrancando servidor Flask...")
    _startup_open_serial()
    # host 0.0.0.0, puerto 5000
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
