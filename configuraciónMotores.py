#!/usr/bin/env python3
import time
import threading
from collections import deque
from typing import Optional

from flask import Flask, request, jsonify, Response
import serial

PORT = "/dev/ttyUSB0"
BAUDRATE = 38400

app = Flask(__name__)

log_lock = threading.Lock()
log_buf = deque(maxlen=400)

ser_lock = threading.Lock()
ser: Optional[serial.Serial] = None

def log(msg: str):
    with log_lock:
        ts = time.strftime("%H:%M:%S")
        log_buf.append(f"[{ts}] {msg}")

def hex_to_bytes(s: str) -> bytes:
    return bytes(int(x, 16) for x in s.split())

def bytes_to_hex(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def ensure_open():
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

def send_fixed(frame_hex: str, rx_len: int = 16) -> bytes:
    """
    Envía una trama ya completa (incluye CRC).
    """
    ensure_open()
    tx = hex_to_bytes(frame_hex)
    with ser_lock:
        assert ser is not None
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        ser.write(tx)
        ser.flush()
        rx = ser.read(rx_len) if rx_len else b""
    log(f"TX: {frame_hex}")
    log(f"RX: {bytes_to_hex(rx) if rx else '(vacío)'}")
    return rx

@app.get("/api/log")
def api_log():
    with log_lock:
        return jsonify({"lines": list(log_buf)})

@app.post("/api/test50")
def api_test50():
    """
    Envía EXACTAMENTE lo que ya te funciona:
      ENABLE: FA 01 F3 01 EF
      SPEED 50: FA 01 F6 00 32 02 25
    """
    try:
        send_fixed("FA 01 F3 01 EF", rx_len=16)          # enable
        time.sleep(0.05)
        send_fixed("FA 01 F6 00 32 02 25", rx_len=16)    # +50 rpm
    except Exception as e:
        log(f"ERROR: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500
    return jsonify({"ok": True})

@app.post("/api/stop")
def api_stop():
    """
    STOP ya verificado: FA 01 F6 00 00 02 F9
    """
    try:
        send_fixed("FA 01 F6 00 00 02 F9", rx_len=16)
    except Exception as e:
        log(f"ERROR: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500
    return jsonify({"ok": True})

HTML = r"""
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>SERVO42D Test Fixed</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 16px; }
    .row { display: flex; gap: 16px; flex-wrap: wrap; }
    .card { border: 1px solid #ccc; border-radius: 10px; padding: 14px; flex: 1; min-width: 320px; }
    .bigbtn {
      width: 100%;
      font-size: 24px;
      padding: 18px;
      border-radius: 12px;
      border: 1px solid #333;
      cursor: pointer;
      margin-top: 10px;
    }
    #console {
      width: 100%;
      height: 260px;
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace;
      white-space: pre;
      overflow: auto;
      border: 1px solid #ccc;
      border-radius: 10px;
      padding: 10px;
      background: #fafafa;
    }
  </style>
</head>
<body>
  <h2>SERVO42D – Diagnóstico (tramas fijas)</h2>
  <div class="row">
    <div class="card">
      <button id="b50" class="bigbtn">TEST 50 RPM</button>
      <button id="bstop" class="bigbtn">STOP</button>
      <p>Estos botones envían tramas completas ya verificadas (con CRC incluido).</p>
    </div>
    <div class="card">
      <div><b>Consola (TX/RX)</b></div>
      <div id="console"></div>
    </div>
  </div>

<script>
async function api(path, method="GET") {
  const r = await fetch(path, {method});
  const j = await r.json();
  if (!r.ok) throw new Error(j.error || "API error");
  return j;
}
async function refreshLog() {
  const j = await api("/api/log");
  const c = document.getElementById("console");
  c.textContent = j.lines.join("\n");
  c.scrollTop = c.scrollHeight;
}
window.addEventListener("load", async () => {
  document.getElementById("b50").onclick = async () => { await api("/api/test50","POST"); await refreshLog(); };
  document.getElementById("bstop").onclick = async () => { await api("/api/stop","POST"); await refreshLog(); };
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
    log("INFO: servidor en http://0.0.0.0:5000")
    app.run(host="0.0.0.0", port=5000, debug=False)
