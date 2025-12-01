#!/usr/bin/env python3
#
# ENME441 Interim Test Code:
# - Two-DOF turret control via Web interface
# - Read turret location from positions.json
#
# Requires:
#   - turret_motors.py (your working motor driver)
#   - Laser on GPIO pin 12 (feel free to change)
#

import http.server
import socketserver
import urllib.parse
import json
import requests
import RPi.GPIO as GPIO

from turret_motors import TurretMotors

# ===========================
# CONFIGURATION
# ===========================
PORT = 8080
LASER_PIN = 12
JSON_URL = "http://192.168.1.254:8000/positions.json"

GPIO.setmode(GPIO.BCM)
GPIO.setup(LASER_PIN, GPIO.OUT)
GPIO.output(LASER_PIN, GPIO.LOW)

# Create motor object
motors = TurretMotors(
    data_pin=16,
    latch_pin=20,
    clock_pin=21,
    pan_gear_ratio=8.0,
    tilt_gear_ratio=6.0,
)

# ===========================
# WEB PAGE HTML
# ===========================
PAGE = """
<!DOCTYPE html>
<html>
<head>
<title>ENME441 Turret Control</title>
<style>
button { width: 150px; height: 40px; margin: 5px; font-size: 16px; }
</style>
</head>
<body>
<h1>ENME441 Turret Web Control</h1>

<h2>Manual Turret Control</h2>
<form method="GET">
  <button name="cmd" value="pan_left">Pan Left</button>
  <button name="cmd" value="pan_right">Pan Right</button><br>
  <button name="cmd" value="tilt_up">Tilt Up</button>
  <button name="cmd" value="tilt_down">Tilt Down</button><br>
  <button name="cmd" value="zero">Zero Motors</button>
</form>

<h2>Laser Control</h2>
<form method="GET">
  <button name="cmd" value="laser_on">Laser ON</button>
  <button name="cmd" value="laser_off">Laser OFF</button>
</form>

<h2>Read turret location from JSON</h2>
<form method="GET">
  <button name="cmd" value="read_json">Read JSON</button>
</form>

<p><b>{MESSAGE}</b></p>

</body>
</html>
"""

class Handler(http.server.SimpleHTTPRequestHandler):

    # Track logical angles for manual stepping
    current_pan = 0.0
    current_tilt = 0.0

    STEP = 5.0  # degrees per button press

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        params = urllib.parse.parse_qs(parsed.query)
        cmd = params.get("cmd", [""])[0]

        message = ""

        try:
            # ============================
            # PAN / TILT CONTROLS
            # ============================
            if cmd == "pan_left":
                Handler.current_pan -= Handler.STEP
                motors.goto(pan_deg=Handler.current_pan,
                            tilt_deg=Handler.current_tilt)
                message = f"Panned left to {Handler.current_pan:.1f}°"

            elif cmd == "pan_right":
                Handler.current_pan += Handler.STEP
                motors.goto(pan_deg=Handler.current_pan,
                            tilt_deg=Handler.current_tilt)
                message = f"Panned right to {Handler.current_pan:.1f}°"

            elif cmd == "tilt_up":
                Handler.current_tilt += Handler.STEP
                motors.goto(pan_deg=Handler.current_pan,
                            tilt_deg=Handler.current_tilt)
                message = f"Tilted up to {Handler.current_tilt:.1f}°"

            elif cmd == "tilt_down":
                Handler.current_tilt -= Handler.STEP
                motors.goto(pan_deg=Handler.current_pan,
                            tilt_deg=Handler.current_tilt)
                message = f"Tilted down to {Handler.current_tilt:.1f}°"

            # ============================
            # ZERO
            # ============================
            elif cmd == "zero":
                motors.zero()
                Handler.current_pan = 0.0
                Handler.current_tilt = 0.0
                GPIO.output(LASER_PIN, GPIO.LOW)
                message = "Motors zeroed"

            # ============================
            # LASER CONTROL
            # ============================
            elif cmd == "laser_on":
                GPIO.output(LASER_PIN, GPIO.HIGH)
                message = "Laser turned ON"

            elif cmd == "laser_off":
                GPIO.output(LASER_PIN, GPIO.LOW)
                message = "Laser turned OFF"

            # ============================
            # READ JSON
            # ============================
            elif cmd == "read_json":
                data = requests.get(JSON_URL, timeout=2).json()
                message = json.dumps(data, indent=2)

            else:
                message = "Ready."

        except Exception as e:
            message = f"Error: {e}"

        # Return webpage
        page = PAGE.replace("{MESSAGE}", message.replace("\n", "<br>"))
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(page.encode("utf-8"))


# ===========================
# MAIN SERVER LOOP
# ===========================
def main():
    print(f"Serving ENME441 turret control on port {PORT}...")
    print(f"Open in browser: http://<Pi-IP>:{PORT}")

    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        httpd.serve_forever()


if __name__ == "__main__":
    main()
