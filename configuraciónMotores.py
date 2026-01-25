#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Servidor de prueba mínimo:
- Flask (una sola página) con DIAL de velocidad (F6)
- Hilo único propietario del RS485 (evita colisiones)
- En cada apertura/reapertura del puerto: envía F3 ENABLE (0xF3 0x01)
- Endpoints: / (UI), /api/speed, /api/stop, /api/status
"""

import time
import threading
from typing import Optional, List

import serial
from flask import Flask, request, jsonify, Response

# =============================
# Configuración fija
# =============================
PORT = "/dev/ttyUSB0"
BAUDRATE = 38400
ADDR = 0x01

# Velocidad (F6)
SPD_ACC = 2
MAX_SPEED_RPM = 300      # dial ±300
ABS_MAX_RPM = 3000       # clamp robustez

# Serie
TIMEOUT_S = 0.25
INTER_FRAME_DELAY = 0.015
RX_LEN = 32

# =============================
# Estado compartido
# =============================
cmd_lock = threading.Lock()
pending_rpm: Optional[int] = None  # última consigna pendiente (se sobrescribe)

status_lock = threading.Lock()
last_tx_hex: str = ""
last_rx_hex: str = ""
last_info: str = "init"

stop_event = threading.Event()

# =============================
# Utilidades protocolo
# =============================
def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF

def build_frame(cmd: int, data: List[int]) -> bytes:
    wo = bytes([0xFA, ADDR & 0xFF, cmd & 0xFF] + [x & 0xFF for x in data])
    return wo + bytes([checksum8(wo)])

def hx(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def frame_f3_enable() -> bytes:
    # F3 enable: FA addr F3 01 chk
    return build_frame(0xF3, [0x01])

def frame_f6_speed(rpm_signed: int) -> bytes:
    rpm = int(rpm_signed)
    direction = 1 if rpm < 0 else 0
    speed = abs(rpm)

    if speed > ABS_MAX_RPM:
        speed = ABS_MAX_RPM

    byte4 = ((direction & 0x01) << 7) | ((speed >> 8) & 0x0F)
    byte5 = speed & 0xFF
    acc = SPD_ACC & 0xFF
    return build_frame(0xF6, [byte4, byte5, acc])

# =============================
# Hilo único propietario del RS485
# =============================
def motor_thread():
    global pending_rpm, last_tx_hex, last_rx_hex, last_info

    ser: Optional[serial.Serial] = None
    last_sent: Optional[int] = None

    def open_port_and_enable():
        nonlocal ser
        # Cierra si existía
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

        ser = serial.Serial(
            PORT, BAUDRATE,
            timeout=TIMEOUT_S,
            write_timeout=TIMEOUT_S,
            rtscts=False,
            dsrdtr=False,
        )
        # Fuerza señales si existen
        try:
            ser.rts = False
            ser.dtr = False
        except Exception:
            pass

        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Enviar ENABLE F3
        tx = frame_f3_enable()
        ser.write(tx)
        ser.flush()
        time.sleep(INTER_FRAME_DELAY)
        rx = ser.read(RX_LEN)

        with status_lock:
            last_tx_hex = hx(tx)
            last_rx_hex = hx(rx) if rx else ""
            last_info = "F3 enable sent (port opened)"

        # Pequeño margen
        time.sleep(0.03)

    # Apertura inicial
    open_port_and_enable()

    while not stop_event.is_set():
        time.sleep(0.01)

        with cmd_lock:
            rpm = pending_rpm
            pending_rpm = None

        if rpm is None:
            continue

        # Clamp UI
        if rpm < -MAX_SPEED_RPM:
            rpm = -MAX_SPEED_RPM
        if rpm > MAX_SPEED_RPM:
            rpm = MAX_SPEED_RPM

        # No reenviar si no cambia
        if last_sent is not None and rpm == last_sent:
            continue

        try:
            assert ser is not None
            tx = frame_f6_speed(rpm)

            ser.reset_input_buffer()
            ser.write(tx)
            ser.flush()
            time.sleep(INTER_FRAME_DELAY)
            rx = ser.read(RX_LEN)

            with status_lock:
                last_tx_hex = hx(tx)
                last_rx_hex = hx(rx) if rx else ""
                last_info = f"F6 sent rpm={rpm}"

            last_sent = rpm

        except Exception as e:
            with status_lock:
                last_info = f"EXC: {e} (reopen+enable)"
            # Recuperación determinista
            try:
                open_port_and_enable()
                last_sent = None
            except Exception as e2:
                with status_lock:
                    last_info = f"EXC reopen failed: {e2}"
                time.sleep(0.2)

# =============================
# Flask + HTML (solo dial)
# =============================
app = Flask(__name__)

HTML = r"""
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Dial Velocidad (prueba)</title>
  <style>
    :root { --bg:#0f172a; --card:#111c36; --border:#1e2a4a; --text:#e5e7eb; --muted:#a7b0c0; }
    body { margin:0; font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Arial; background:var(--bg); color:var(--text); }
    .wrap { max-width:820px; margin:18px auto; padding:0 14px 24px; }
    .card { background:var(--card); border:1px solid var(--border); border-radius:14px; padding:16px; box-shadow:0 10px 26px rgba(0,0,0,0.25); }
    h1 { font-size:16px; margin:0 0 12px; font-weight:650; }
    .row { display:flex; gap:12px; flex-wrap:wrap; align-items:center; }
    button { border:1px solid var(--border); background:#223055; color:var(--text); padding:12px 14px; border-radius:12px; cursor:pointer; font-weight:650; min-width:140px; }
    button.stop { background:#381a1a; border-color:#6b1f1f; }
    .muted { color:var(--muted); font-size:12px; margin-top:10px; }
    .mono { font-family: ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,"Liberation Mono","Courier New",monospace; }

    .dialWrap { display:flex; gap:18px; align-items:center; flex-wrap:wrap; }
    .dial {
      width: 260px; height: 260px; border-radius: 50%;
      background: radial-gradient(circle at 30% 30%, #1a2b55, #0b1224 70%);
      border: 1px solid var(--border);
      position: relative;
      user-select: none;
      touch-action: none;
    }
    .dial::after {
      content:"";
      position:absolute; inset: 38%;
      border-radius:50%;
      background: #0b1224;
      border: 1px solid var(--border);
    }
    .needle {
      position:absolute; left:50%; top:50%;
      width: 4px; height: 104px;
      background: #e5e7eb;
      transform-origin: 50% 104%;
      transform: translate(-50%,-104%) rotate(0deg);
      border-radius: 4px;
    }
    .rpm { font-variant-numeric: tabular-nums; font-size: 38px; font-weight: 780; }
    .unit { color: var(--muted); font-size: 13px; margin-top: 2px; }

    pre {
      margin: 12px 0 0;
      background:#0b1224; border:1px solid var(--border);
      border-radius:12px; padding:12px; height: 160px; overflow:auto;
      font-size:12px; line-height:1.35; color:#d1d5db;
    }
  </style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h1>Dial de velocidad (F6) — prueba</h1>

    <div class="dialWrap">
      <div class="dial" id="dial">
        <div class="needle" id="needle"></div>
      </div>

      <div>
        <div class="rpm" id="rpmText">0</div>
        <div class="unit">RPM (±{{MAX}})</div>

        <div class="row" style="margin-top:14px;">
          <button class="stop" onclick="stopMotor()">STOP</button>
          <button onclick="setZero()">0 RPM</button>
        </div>

        <div class="muted">
          Envío con debounce (~120 ms). Backend envía F3 enable al abrir/reabrir el puerto.
        </div>
      </div>
    </div>

    <pre class="mono" id="dbg"></pre>
  </div>
</div>

<script>
  const MAX = {{MAX}};
  const dial = document.getElementById("dial");
  const needle = document.getElementById("needle");
  const rpmText = document.getElementById("rpmText");
  const dbg = document.getElementById("dbg");

  // Arco útil: 270° (-135..+135)
  const ANG_MIN = -135;
  const ANG_MAX =  135;

  let currentRpm = 0;
  let debounceTimer = null;
  let dragging = false;

  function clamp(v,a,b){ return Math.max(a, Math.min(b,v)); }

  function rpmToAngle(rpm){
    const x = clamp(rpm, -MAX, MAX);
    const t = (x + MAX) / (2*MAX);
    return ANG_MIN + t*(ANG_MAX-ANG_MIN);
  }

  function angleToRpm(angle){
    const a = clamp(angle, ANG_MIN, ANG_MAX);
    const t = (a - ANG_MIN) / (ANG_MAX-ANG_MIN);
    return Math.round(-MAX + t*(2*MAX));
  }

  function setRpmUI(rpm){
    currentRpm = clamp(rpm, -MAX, MAX);
    rpmText.textContent = String(currentRpm);
    const ang = rpmToAngle(currentRpm);
    needle.style.transform = `translate(-50%,-104%) rotate(${ang}deg)`;
  }

  async function apiPost(url, body){
    const r = await fetch(url, {
      method:"POST",
      headers: {"Content-Type":"application/json"},
      body: body ? JSON.stringify(body) : null
    });
    const j = await r.json();
    if (!r.ok || j.ok === false) throw new Error(j.error || ("HTTP " + r.status));
    return j;
  }

  async function apiGet(url){
    const r = await fetch(url, {cache:"no-store"});
    const j = await r.json();
    if (!r.ok || j.ok === false) throw new Error(j.error || ("HTTP " + r.status));
    return j;
  }

  function scheduleSend(){
    if (debounceTimer) clearTimeout(debounceTimer);
    debounceTimer = setTimeout(async () => {
      try {
        await apiPost("/api/speed", {rpm: currentRpm});
      } catch(e) {
        console.warn(e);
      }
    }, 120);
  }

  function pointerToAngle(ev){
    const rect = dial.getBoundingClientRect();
    const cx = rect.left + rect.width/2;
    const cy = rect.top  + rect.height/2;
    const x = ev.clientX - cx;
    const y = ev.clientY - cy;
    const rad = Math.atan2(y, x);
    let deg = rad * (180/Math.PI);
    deg = deg + 90;
    if (deg > 180) deg -= 360;
    if (deg < -180) deg += 360;
    return deg;
  }

  dial.addEventListener("pointerdown", (ev) => {
    dragging = true;
    dial.setPointerCapture(ev.pointerId);
    setRpmUI(angleToRpm(pointerToAngle(ev)));
    scheduleSend();
  });

  dial.addEventListener("pointermove", (ev) => {
    if (!dragging) return;
    setRpmUI(angleToRpm(pointerToAngle(ev)));
    scheduleSend();
  });

  dial.addEventListener("pointerup", (ev) => {
    dragging = false;
    try { dial.releasePointerCapture(ev.pointerId); } catch(e){}
  });

  async function stopMotor(){
    try {
      setRpmUI(0);
      await apiPost("/api/stop", null);
    } catch(e){ console.warn(e); }
  }

  async function setZero(){
    try {
      setRpmUI(0);
      await apiPost("/api/speed", {rpm: 0});
    } catch(e){ console.warn(e); }
  }

  async function refreshDbg(){
    try {
      const st = await apiGet("/api/status");
      dbg.textContent =
        `info: ${st.info}\n` +
        `pending_rpm: ${st.pending_rpm}\n` +
        `last_tx: ${st.last_tx_hex}\n` +
        `last_rx: ${st.last_rx_hex}\n`;
      dbg.scrollTop = dbg.scrollHeight;
    } catch(e) {
      // no-op
    }
  }

  setRpmUI(0);
  refreshDbg();
  setInterval(refreshDbg, 700);
</script>
</body>
</html>
"""

@app.get("/")
def index() -> Response:
    return Response(HTML.replace("{{MAX}}", str(MAX_SPEED_RPM)),
                    mimetype="text/html; charset=utf-8")

@app.post("/api/speed")
def api_speed():
    global pending_rpm
    try:
        data = request.get_json(force=True, silent=False)
        rpm = int(data.get("rpm"))

        if rpm < -MAX_SPEED_RPM or rpm > MAX_SPEED_RPM:
            return jsonify({"ok": False, "error": f"rpm fuera de rango [-{MAX_SPEED_RPM}..{MAX_SPEED_RPM}]"}), 400

        with cmd_lock:
            pending_rpm = rpm

        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

@app.post("/api/stop")
def api_stop():
    global pending_rpm
    try:
        with cmd_lock:
            pending_rpm = 0
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

@app.get("/api/status")
def api_status():
    with cmd_lock:
        pr = pending_rpm
    with status_lock:
        st = {
            "ok": True,
            "pending_rpm": pr,
            "last_tx_hex": last_tx_hex,
            "last_rx_hex": last_rx_hex,
            "info": last_info,
        }
    return jsonify(st)

# =============================
# Main
# =============================
def main():
    threading.Thread(target=motor_thread, daemon=True).start()
    print(f"[INFO] http://0.0.0.0:5000  RS485 {PORT}@{BAUDRATE} ADDR=0x{ADDR:02X}")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)

if __name__ == "__main__":
    main()
