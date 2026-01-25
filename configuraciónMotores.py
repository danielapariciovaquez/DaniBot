#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

# F6 velocidad
SPD_ACC = 2          # 0..255 fijo
MAX_SPEED_RPM = 300  # rango del dial: [-MAX_SPEED_RPM..+MAX_SPEED_RPM]

# Recepción
RX_LEN = 32
TIMEOUT_S = 0.25

# =============================
# Estado + locks
# =============================
serial_lock = threading.Lock()
ser: Optional[serial.Serial] = None

# =============================
# Protocolo checksum8
# =============================
def checksum8(frame_wo_checksum: List[int]) -> int:
    return sum(frame_wo_checksum) & 0xFF

def build_frame(cmd: int, data: List[int]) -> bytes:
    frame = [0xFA, ADDR & 0xFF, cmd & 0xFF] + [x & 0xFF for x in data]
    frame.append(checksum8(frame))
    return bytes(frame)

def bytes_to_hex(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

# =============================
# Serie: open-on-demand
# =============================
def _ensure_serial_open_locked() -> serial.Serial:
    global ser
    if ser is not None and ser.is_open:
        return ser
    ser = serial.Serial(
        PORT,
        BAUDRATE,
        timeout=TIMEOUT_S,
        write_timeout=TIMEOUT_S
    )
    # Limpieza buffers para evitar basura previa
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print(f"[INFO] Puerto abierto: {PORT} @ {BAUDRATE}, ADDR=0x{ADDR:02X}")
    return ser

def _txrx_locked(tx: bytes) -> bytes:
    s = _ensure_serial_open_locked()
    print("[TX]", bytes_to_hex(tx))
    s.reset_input_buffer()
    s.write(tx)
    s.flush()
    rx = s.read(RX_LEN)
    if rx:
        print("[RX]", bytes_to_hex(rx))
    else:
        print("[RX] (vacío)")
    return rx

# =============================
# Comandos
# =============================
def send_speed_f6(rpm_signed: int) -> None:
    """
    F6: FA ADDR F6 byte4 byte5 acc chk
    byte4 = (direction<<7) | ((speed>>8)&0x0F)
    byte5 = speed & 0xFF
    """
    rpm = int(rpm_signed)
    direction = 1 if rpm < 0 else 0
    speed = abs(rpm)

    # robustez
    if speed > 3000:
        speed = 3000

    byte4 = ((direction & 0x01) << 7) | ((speed >> 8) & 0x0F)
    byte5 = speed & 0xFF
    acc = SPD_ACC & 0xFF

    tx = build_frame(0xF6, [byte4, byte5, acc])

    with serial_lock:
        _txrx_locked(tx)

def stop_motor() -> None:
    send_speed_f6(0)

# =============================
# Flask
# =============================
app = Flask(__name__)

HTML = r"""
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Dial Velocidad — MKS SERVO42D</title>
  <style>
    :root { --bg:#0f172a; --card:#111c36; --border:#1e2a4a; --text:#e5e7eb; --muted:#a7b0c0; }
    body { margin:0; font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Arial; background:var(--bg); color:var(--text); }
    .wrap { max-width:720px; margin:20px auto; padding:0 14px; }
    .card { background:var(--card); border:1px solid var(--border); border-radius:14px; padding:16px; box-shadow:0 10px 26px rgba(0,0,0,0.25); }
    h1 { font-size:16px; margin:0 0 12px; font-weight:650; }
    .row { display:flex; gap:12px; flex-wrap:wrap; align-items:center; }
    button { border:1px solid var(--border); background:#223055; color:var(--text); padding:12px 14px; border-radius:12px; cursor:pointer; font-weight:650; min-width:140px; }
    button.stop { background:#381a1a; border-color:#6b1f1f; }
    .muted { color:var(--muted); font-size:12px; margin-top:8px; }

    /* Dial */
    .dialWrap { display:flex; gap:18px; align-items:center; flex-wrap:wrap; }
    .dial {
      width: 240px; height: 240px; border-radius: 50%;
      background: radial-gradient(circle at 30% 30%, #1a2b55, #0b1224 70%);
      border: 1px solid var(--border);
      position: relative;
      box-shadow: inset 0 0 0 10px rgba(255,255,255,0.02);
      user-select: none;
      touch-action: none;
    }
    .dial::after { /* centro */
      content:"";
      position:absolute; inset: 38%;
      border-radius:50%;
      background: #0b1224;
      border: 1px solid var(--border);
      box-shadow: 0 0 0 2px rgba(255,255,255,0.03);
    }
    .needle {
      position:absolute; left:50%; top:50%;
      width: 4px; height: 92px;
      background: #e5e7eb;
      transform-origin: 50% 92%;
      transform: translate(-50%,-92%) rotate(0deg);
      border-radius: 4px;
      box-shadow: 0 0 12px rgba(255,255,255,0.15);
    }
    .scale {
      position:absolute; inset: 8%;
      border-radius: 50%;
      pointer-events:none;
    }
    .readout {
      min-width: 200px;
    }
    .rpm {
      font-variant-numeric: tabular-nums;
      font-size: 34px;
      font-weight: 750;
    }
    .unit {
      color: var(--muted);
      font-size: 13px;
      margin-top: 2px;
    }
  </style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h1>Velocidad (F6) — Dial</h1>

    <div class="dialWrap">
      <div class="dial" id="dial">
        <div class="needle" id="needle"></div>
        <canvas class="scale" id="scale"></canvas>
      </div>

      <div class="readout">
        <div class="rpm" id="rpmText">0</div>
        <div class="unit">RPM (rango: ±{{MAX}})</div>
        <div class="row" style="margin-top:14px;">
          <button class="stop" onclick="doStop()">STOP</button>
        </div>
        <div class="muted">
          Arrastra el dial. Envío con debounce (~120 ms) para no saturar RS485.
        </div>
      </div>
    </div>
  </div>
</div>

<script>
  const MAX = {{MAX}};
  const dial = document.getElementById("dial");
  const needle = document.getElementById("needle");
  const rpmText = document.getElementById("rpmText");

  // Mapeo ángulo: usamos un arco útil de 270° (-135°..+135°)
  const ANG_MIN = -135;
  const ANG_MAX =  135;

  let currentRpm = 0;
  let debounceTimer = null;

  function clamp(v, a, b){ return Math.max(a, Math.min(b, v)); }

  function rpmToAngle(rpm){
    const x = clamp(rpm, -MAX, MAX);
    const t = (x + MAX) / (2*MAX);       // 0..1
    return ANG_MIN + t*(ANG_MAX-ANG_MIN);
  }

  function angleToRpm(angle){
    const a = clamp(angle, ANG_MIN, ANG_MAX);
    const t = (a - ANG_MIN) / (ANG_MAX-ANG_MIN); // 0..1
    const rpm = Math.round(-MAX + t*(2*MAX));
    return rpm;
  }

  function setRpmUI(rpm){
    currentRpm = clamp(rpm, -MAX, MAX);
    rpmText.textContent = String(currentRpm);
    const ang = rpmToAngle(currentRpm);
    needle.style.transform = `translate(-50%,-92%) rotate(${ang}deg)`;
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
    // atan2: 0° hacia +x. Convertimos a grados con 0° arriba.
    const rad = Math.atan2(y, x);
    let deg = rad * (180/Math.PI);
    deg = deg + 90; // ahora 0° arriba
    // normalizamos a [-180..180]
    if (deg > 180) deg -= 360;
    if (deg < -180) deg += 360;
    return deg;
  }

  let dragging = false;

  dial.addEventListener("pointerdown", (ev) => {
    dragging = true;
    dial.setPointerCapture(ev.pointerId);
    const ang = pointerToAngle(ev);
    const rpm = angleToRpm(ang);
    setRpmUI(rpm);
    scheduleSend();
  });

  dial.addEventListener("pointermove", (ev) => {
    if (!dragging) return;
    const ang = pointerToAngle(ev);
    const rpm = angleToRpm(ang);
    setRpmUI(rpm);
    scheduleSend();
  });

  dial.addEventListener("pointerup", (ev) => {
    dragging = false;
    dial.releasePointerCapture(ev.pointerId);
  });

  async function doStop(){
    try {
      setRpmUI(0);
      await apiPost("/api/stop", null);
    } catch(e) {
      console.warn(e);
    }
  }

  // Dibuja escala simple
  function drawScale(){
    const canvas = document.getElementById("scale");
    const d = canvas.getBoundingClientRect();
    canvas.width = Math.floor(d.width * devicePixelRatio);
    canvas.height = Math.floor(d.height * devicePixelRatio);
    const ctx = canvas.getContext("2d");
    ctx.scale(devicePixelRatio, devicePixelRatio);

    const w = d.width, h = d.height;
    const cx = w/2, cy = h/2;
    const r1 = Math.min(w,h)*0.46;
    const r2 = Math.min(w,h)*0.40;

    ctx.clearRect(0,0,w,h);
    ctx.lineWidth = 2;

    function tick(angleDeg, len, alpha){
      const a = (angleDeg-90) * Math.PI/180; // 0° arriba
      const x1 = cx + Math.cos(a)*r1;
      const y1 = cy + Math.sin(a)*r1;
      const x2 = cx + Math.cos(a)*(r1-len);
      const y2 = cy + Math.sin(a)*(r1-len);
      ctx.strokeStyle = `rgba(229,231,235,${alpha})`;
      ctx.beginPath(); ctx.moveTo(x1,y1); ctx.lineTo(x2,y2); ctx.stroke();
    }

    for (let i=0;i<=18;i++){
      const a = ANG_MIN + i*(ANG_MAX-ANG_MIN)/18;
      tick(a, (i%3===0)?18:10, (i%3===0)?0.45:0.18);
    }
  }

  window.addEventListener("resize", drawScale);
  drawScale();
  setRpmUI(0);
</script>
</body>
</html>
"""

@app.get("/")
def index() -> Response:
    page = HTML.replace("{{MAX}}", str(MAX_SPEED_RPM))
    return Response(page, mimetype="text/html; charset=utf-8")

@app.post("/api/speed")
def api_speed():
    try:
        data = request.get_json(force=True, silent=False)
        rpm = int(data.get("rpm"))
        if rpm < -MAX_SPEED_RPM or rpm > MAX_SPEED_RPM:
            return jsonify({"ok": False, "error": f"rpm fuera de rango [-{MAX_SPEED_RPM}..{MAX_SPEED_RPM}]"}), 400
        send_speed_f6(rpm)
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

@app.post("/api/stop")
def api_stop():
    try:
        stop_motor()
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

if __name__ == "__main__":
    print("[INFO] Servidor web en http://0.0.0.0:5000")
    # No forzamos apertura: open-on-demand al primer comando.
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
