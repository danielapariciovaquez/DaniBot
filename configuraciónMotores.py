#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Flask + UI dial (knob) para control de velocidad (F6) en MKS SERVO42D por RS485.
Arquitectura robusta:
- Flask NO escribe al puerto serie.
- Un único hilo "motor_loop" es el único propietario de la UART y envía F6 de forma secuencial.
- El hilo motor aplica un INTER_FRAME_DELAY y puede (opcionalmente) leer 0x31 para comprobar enlace.

Requisitos:
  pip install flask pyserial
Ejecución:
  python3 speed_dial_server.py
Abrir en navegador:
  http://IP_DE_LA_RASPBERRY:5000
"""

import time
import threading
from typing import Optional, List, Tuple

import serial
from flask import Flask, request, jsonify, Response

# =============================
# Configuración fija
# =============================
PORT = "/dev/ttyUSB0"
BAUDRATE = 38400
ADDR = 0x01

# Control velocidad (F6)
SPD_ACC = 2              # 0..255 fijo
MAX_SPEED_RPM = 300      # rango del dial: [-MAX_SPEED_RPM..+MAX_SPEED_RPM]
ABS_MAX_RPM = 3000       # clamp robustez

# Lectura opcional de posición (0x31) para "health check"
ENABLE_READBACK_31 = True
COUNTS_PER_REV = 16384   # 0x31 -> counts / vuelta

# Temporización RS485
TIMEOUT_S = 0.25
RX_LEN = 64
INTER_FRAME_DELAY = 0.015   # gap real entre tramas (15 ms)
LOOP_PERIOD = 0.05          # 20 Hz (estable). Solo envía cuando hay cambio, no streaming continuo.

# =============================
# Estado compartido (Flask <-> motor_loop)
# =============================
speed_lock = threading.Lock()
desired_speed_rpm: int = 0  # consigna actual del usuario

# Estado para status
status_lock = threading.Lock()
last_tx: Optional[bytes] = None
last_rx: Optional[bytes] = None
last_pos_counts: Optional[int] = None
link_ok: bool = False
last_error: Optional[str] = None
last_activity_ts: float = 0.0

# Señal de parada (por limpieza)
stop_event = threading.Event()

# =============================
# Protocolo checksum8
# =============================
def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF

def build_frame(cmd: int, data: List[int]) -> bytes:
    wo = bytes([0xFA, ADDR & 0xFF, cmd & 0xFF] + [x & 0xFF for x in data])
    return wo + bytes([checksum8(wo)])

def bytes_to_hex(b: Optional[bytes]) -> str:
    if not b:
        return ""
    return " ".join(f"{x:02X}" for x in b)

def int48_be_signed(b6: bytes) -> int:
    # 6 bytes big-endian two's complement
    u = int.from_bytes(b6, "big", signed=False)
    if u & (1 << 47):
        u -= (1 << 48)
    return u

# =============================
# UART owner: Link
# =============================
class Link:
    def __init__(self):
        self.ser: Optional[serial.Serial] = None

    def open(self) -> None:
        self.close()
        self.ser = serial.Serial(
            PORT,
            BAUDRATE,
            timeout=TIMEOUT_S,
            write_timeout=TIMEOUT_S,
            rtscts=False,
            dsrdtr=False,
        )
        # Forzar señales si están disponibles (algunos adaptadores se benefician)
        try:
            self.ser.rts = False
            self.ser.dtr = False
        except Exception:
            pass

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def txrx(self, tx: bytes, read_len: int = RX_LEN) -> bytes:
        if self.ser is None or not self.ser.is_open:
            self.open()

        assert self.ser is not None
        s = self.ser

        # Limpieza RX previa para evitar residuos
        s.reset_input_buffer()

        s.write(tx)
        s.flush()

        time.sleep(INTER_FRAME_DELAY)

        rx = s.read(read_len)
        return rx

# =============================
# Comandos
# =============================
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

def read_31_pos_counts(link: Link) -> Optional[int]:
    tx = build_frame(0x31, [])
    rx = link.txrx(tx, read_len=10)  # 0x31: FB addr 31 + int48(6) + chk = 10 bytes

    if len(rx) != 10:
        return None
    if rx[0] != 0xFB or rx[1] != (ADDR & 0xFF) or rx[2] != 0x31:
        return None
    if checksum8(rx[:-1]) != rx[-1]:
        return None

    return int48_be_signed(rx[3:9])

# =============================
# Motor loop (único dueño de la UART)
# =============================
def motor_loop():
    global last_tx, last_rx, last_pos_counts, link_ok, last_error, last_activity_ts

    link = Link()
    last_sent_speed: Optional[int] = None

    while not stop_event.is_set():
        t_start = time.time()

        # 1) Leer consigna atómica
        with speed_lock:
            rpm = int(desired_speed_rpm)

        # 2) Clamp UI range (robustez)
        if rpm < -MAX_SPEED_RPM:
            rpm = -MAX_SPEED_RPM
        if rpm > MAX_SPEED_RPM:
            rpm = MAX_SPEED_RPM

        try:
            # Reabrimos si hace falta (si link.ser murió, txrx hará open)
            if rpm != last_sent_speed:
                tx = frame_f6_speed(rpm)
                rx = link.txrx(tx)

                with status_lock:
                    last_tx = tx
                    last_rx = rx if rx else b""
                    link_ok = True
                    last_error = None
                    last_activity_ts = time.time()

                last_sent_speed = rpm

            # 3) Health check opcional: leer 0x31 (sin “spamear”)
            if ENABLE_READBACK_31:
                # Solo cada ~200ms para no cargar bus
                # (en un loop 20Hz, lo hacemos 1 de cada 4 iteraciones aprox)
                if int(t_start * 10) % 2 == 0:  # patrón simple ~5 Hz
                    pos = read_31_pos_counts(link)
                    with status_lock:
                        last_pos_counts = pos
                        # Si pos es None no marcamos link como KO automáticamente;
                        # solo lo usamos como telemetría. Si quisieras, aquí podrías reabrir.
                        last_activity_ts = time.time()

        except Exception as e:
            # Si hay error, marcamos y reabrimos para recuperar
            with status_lock:
                link_ok = False
                last_error = str(e)
            try:
                link.open()
            except Exception as e2:
                with status_lock:
                    last_error = f"{e} | reopen failed: {e2}"

        # 4) Control de periodo del loop
        elapsed = time.time() - t_start
        sleep_s = LOOP_PERIOD - elapsed
        if sleep_s > 0:
            time.sleep(sleep_s)

# =============================
# Flask + HTML embebido
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
    .wrap { max-width:860px; margin:20px auto; padding:0 14px 24px; }
    .grid { display:grid; grid-template-columns:1fr; gap:12px; }
    @media (min-width: 900px) { .grid { grid-template-columns: 1fr 1fr; } .span2 { grid-column: span 2; } }
    .card { background:var(--card); border:1px solid var(--border); border-radius:14px; padding:16px; box-shadow:0 10px 26px rgba(0,0,0,0.25); }
    h1 { font-size:16px; margin:0 0 12px; font-weight:650; }
    h2 { font-size:13px; margin:0 0 10px; font-weight:650; color:var(--text); }
    .row { display:flex; gap:12px; flex-wrap:wrap; align-items:center; }
    button { border:1px solid var(--border); background:#223055; color:var(--text); padding:12px 14px; border-radius:12px; cursor:pointer; font-weight:650; min-width:140px; }
    button.stop { background:#381a1a; border-color:#6b1f1f; }
    .muted { color:var(--muted); font-size:12px; margin-top:8px; }

    /* Dial */
    .dialWrap { display:flex; gap:18px; align-items:center; flex-wrap:wrap; }
    .dial {
      width: 260px; height: 260px; border-radius: 50%;
      background: radial-gradient(circle at 30% 30%, #1a2b55, #0b1224 70%);
      border: 1px solid var(--border);
      position: relative;
      box-shadow: inset 0 0 0 10px rgba(255,255,255,0.02);
      user-select: none;
      touch-action: none;
    }
    .dial::after {
      content:"";
      position:absolute; inset: 38%;
      border-radius:50%;
      background: #0b1224;
      border: 1px solid var(--border);
      box-shadow: 0 0 0 2px rgba(255,255,255,0.03);
    }
    .needle {
      position:absolute; left:50%; top:50%;
      width: 4px; height: 104px;
      background: #e5e7eb;
      transform-origin: 50% 104%;
      transform: translate(-50%,-104%) rotate(0deg);
      border-radius: 4px;
      box-shadow: 0 0 12px rgba(255,255,255,0.15);
    }
    .scale {
      position:absolute; inset: 8%;
      border-radius: 50%;
      pointer-events:none;
    }
    .readout { min-width: 240px; }
    .rpm { font-variant-numeric: tabular-nums; font-size: 38px; font-weight: 780; }
    .unit { color: var(--muted); font-size: 13px; margin-top: 2px; }
    .mono { font-family: ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,"Liberation Mono","Courier New",monospace; }
    pre {
      margin:0; background:#0b1224; border:1px solid var(--border);
      border-radius:12px; padding:12px; height: 220px; overflow:auto; font-size:12px; line-height:1.35;
      color:#d1d5db;
    }
    .kv { display:grid; grid-template-columns: 1fr auto; gap: 6px 10px; align-items:center; }
    .kv .k { color: var(--muted); font-size: 12px; }
    .kv .v { font-variant-numeric: tabular-nums; font-weight:650; }
  </style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h1>MKS SERVO42D — Dial de velocidad (F6) por RS485</h1>
    <div class="muted">Flask solo actualiza estado. Un hilo único gestiona el puerto serie (sin colisiones).</div>
  </div>

  <div class="grid">
    <div class="card">
      <h2>Control</h2>
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
            <button onclick="setZero()">0 RPM</button>
          </div>

          <div class="muted">
            Arrastra el dial. Envío al backend con debounce (~120 ms).
          </div>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>Estado enlace</h2>
      <div class="kv">
        <div class="k">Link</div><div class="v mono" id="linkOk">—</div>
        <div class="k">Última actividad</div><div class="v mono" id="lastTs">—</div>
        <div class="k">Pos 0x31 (counts)</div><div class="v mono" id="posCounts">—</div>
        <div class="k">Vueltas</div><div class="v mono" id="turns">—</div>
        <div class="k">Error</div><div class="v mono" id="err">—</div>
      </div>
      <div class="muted" style="margin-top:10px;">Actualización cada ~700 ms.</div>
    </div>

    <div class="card span2">
      <h2>Última TX/RX</h2>
      <pre id="console" class="mono"></pre>
    </div>
  </div>
</div>

<script>
  const MAX = {{MAX}};
  const COUNTS_PER_REV = {{CPR}};

  const dial = document.getElementById("dial");
  const needle = document.getElementById("needle");
  const rpmText = document.getElementById("rpmText");

  // Arco útil: 270° (-135..+135)
  const ANG_MIN = -135;
  const ANG_MAX =  135;

  let currentRpm = 0;
  let debounceTimer = null;
  let dragging = false;

  function clamp(v, a, b){ return Math.max(a, Math.min(b, v)); }

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
    deg = deg + 90; // 0° arriba
    if (deg > 180) deg -= 360;
    if (deg < -180) deg += 360;
    return deg;
  }

  dial.addEventListener("pointerdown", (ev) => {
    dragging = true;
    dial.setPointerCapture(ev.pointerId);
    const rpm = angleToRpm(pointerToAngle(ev));
    setRpmUI(rpm);
    scheduleSend();
  });

  dial.addEventListener("pointermove", (ev) => {
    if (!dragging) return;
    const rpm = angleToRpm(pointerToAngle(ev));
    setRpmUI(rpm);
    scheduleSend();
  });

  dial.addEventListener("pointerup", (ev) => {
    dragging = false;
    try { dial.releasePointerCapture(ev.pointerId); } catch(e){}
  });

  async function doStop(){
    try {
      setRpmUI(0);
      await apiPost("/api/stop", null);
    } catch(e) {
      console.warn(e);
    }
  }

  async function setZero(){
    try {
      setRpmUI(0);
      await apiPost("/api/speed", {rpm: 0});
    } catch(e) { console.warn(e); }
  }

  function drawScale(){
    const canvas = document.getElementById("scale");
    const d = canvas.getBoundingClientRect();
    canvas.width = Math.floor(d.width * devicePixelRatio);
    canvas.height = Math.floor(d.height * devicePixelRatio);
    const ctx = canvas.getContext("2d");
    ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);

    const w = d.width, h = d.height;
    const cx = w/2, cy = h/2;
    const r1 = Math.min(w,h)*0.46;

    ctx.clearRect(0,0,w,h);
    ctx.lineWidth = 2;

    function tick(angleDeg, len, alpha){
      const a = (angleDeg-90) * Math.PI/180;
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

  async function refreshStatus(){
    try {
      const st = await apiGet("/api/status");
      document.getElementById("linkOk").textContent = st.link_ok ? "OK" : "NO";
      document.getElementById("lastTs").textContent = st.last_activity_ts ? st.last_activity_ts : "—";
      document.getElementById("err").textContent = st.last_error ? st.last_error : "—";

      const pc = st.last_pos_counts;
      document.getElementById("posCounts").textContent = (pc === null || pc === undefined) ? "—" : String(pc);
      if (pc === null || pc === undefined) {
        document.getElementById("turns").textContent = "—";
      } else {
        document.getElementById("turns").textContent = (pc / COUNTS_PER_REV).toFixed(6);
      }

      const con = document.getElementById("console");
      con.textContent =
        `desired_rpm: ${st.desired_rpm}\n` +
        `last_tx: ${st.last_tx_hex || ""}\n` +
        `last_rx: ${st.last_rx_hex || ""}\n`;
      con.scrollTop = con.scrollHeight;

    } catch(e) {
      // no-op
    }
  }

  window.addEventListener("resize", drawScale);
  drawScale();
  setRpmUI(0);
  refreshStatus();
  setInterval(refreshStatus, 700);
</script>
</body>
</html>
"""

@app.get("/")
def index() -> Response:
    page = HTML.replace("{{MAX}}", str(MAX_SPEED_RPM)).replace("{{CPR}}", str(COUNTS_PER_REV))
    return Response(page, mimetype="text/html; charset=utf-8")

@app.post("/api/speed")
def api_speed():
    global desired_speed_rpm
    try:
        data = request.get_json(force=True, silent=False)
        rpm = int(data.get("rpm"))

        if rpm < -MAX_SPEED_RPM or rpm > MAX_SPEED_RPM:
            return jsonify({"ok": False, "error": f"rpm fuera de rango [-{MAX_SPEED_RPM}..{MAX_SPEED_RPM}]"}), 400

        # IMPORTANTE: aquí NO se escribe a RS485
        with speed_lock:
            desired_speed_rpm = rpm

        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

@app.post("/api/stop")
def api_stop():
    global desired_speed_rpm
    try:
        with speed_lock:
            desired_speed_rpm = 0
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

@app.get("/api/status")
def api_status():
    with speed_lock:
        rpm = int(desired_speed_rpm)

    with status_lock:
        st = {
            "ok": True,
            "desired_rpm": rpm,
            "link_ok": bool(link_ok),
            "last_error": last_error,
            "last_tx_hex": bytes_to_hex(last_tx),
            "last_rx_hex": bytes_to_hex(last_rx),
            "last_pos_counts": last_pos_counts,
            "last_activity_ts": time.strftime("%H:%M:%S", time.localtime(last_activity_ts)) if last_activity_ts else None,
            "serial": {"port": PORT, "baudrate": BAUDRATE, "addr": ADDR},
            "constants": {
                "spd_acc": SPD_ACC,
                "max_speed_rpm": MAX_SPEED_RPM,
                "abs_max_rpm": ABS_MAX_RPM,
                "inter_frame_delay": INTER_FRAME_DELAY,
                "loop_period": LOOP_PERIOD,
                "readback_31": ENABLE_READBACK_31,
            }
        }
    return jsonify(st)

# =============================
# Main
# =============================
def main():
    # Arranca motor loop (único dueño de UART)
    th = threading.Thread(target=motor_loop, daemon=True)
    th.start()

    print(f"[INFO] Servidor web: http://0.0.0.0:5000  (PORT={PORT}, BAUD={BAUDRATE}, ADDR=0x{ADDR:02X})")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)

if __name__ == "__main__":
    main()
