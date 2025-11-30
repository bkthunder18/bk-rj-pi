# turret_interim.py
#
# Interim test script for ENME441 laser turret:
#  (1) Move laser (turret) with 2 degrees of freedom via Web interface
#  (2) Read turret location from the JSON file
#
# Web interface:
#   - Pan Left / Right buttons
#   - Tilt Up / Down buttons
#   - Zero button to reset angles
#   - Read JSON button to read and display your turret location
#
# This script uses:
#   - TurretMotors from turret_motors.py (your working motor module)
#   - A simple socket-based HTTP server + POST parsing as in course examples.

import socket
import threading
import time
import json
import requests  # used to GET positions.json from the router
from turret_motors import TurretMotors  # your motor wrapper

# ==== USER SETTINGS ====================================================

# Set this to your team ID key in the JSON file (string, e.g. "1", "7", etc.)
TEAM_ID = "2"

# Movement increments in turret degrees for each button press:
PAN_STEP_DEG = 5.0
TILT_STEP_DEG = 5.0

# Gear ratios are configured in TurretMotors; adjust in code or later via web.
PAN_GEAR_RATIO = 1.0   # stepper_deg / turret_deg
TILT_GEAR_RATIO = 1.0  # stepper_deg / turret_deg

# JSON URL from project description
JSON_URL = "http://192.168.1.254:8000/positions.json"

# Port to serve the web interface on (80 requires sudo, 8080 is easier for testing)
HTTP_PORT = 8080

# ==== GLOBAL STATE =====================================================

# Logical turret angles (in turret coordinates, not stepper shaft)
pan_angle = 0.0
tilt_angle = 0.0

# Last turret JSON data (for display on web page)
last_json_status = "No JSON read yet."
last_json_raw = ""  # optional, store raw JSON or snippet

# Turret motor controller (initialized in main)
turret = None


# ==== HELPER FUNCTIONS =================================================

def parsePOSTdata(request_bytes: bytes) -> dict:
    """
    Extract key=value pairs from a POST request body.
    Based on the parsePOSTdata() pattern in the course network slides.
    """
    try:
        data_str = request_bytes.decode('utf-8', errors='ignore')
        idx = data_str.find('\r\n\r\n') + 4
        body = data_str[idx:]
        data_pairs = body.split('&')
        data_dict = {}
        for pair in data_pairs:
            key_val = pair.split('=')
            if len(key_val) == 2:
                key, val = key_val
                data_dict[key] = val
        return data_dict
    except Exception as e:
        print(f"parsePOSTdata error: {e}")
        return {}


def fetch_positions():
    """
    Fetch the positions.json file from the router and return as a dict.
    """
    global last_json_status, last_json_raw
    try:
        r = requests.get(JSON_URL, timeout=2.0)
        r.raise_for_status()
        data = r.json()
        last_json_raw = json.dumps(data, indent=2)
        last_json_status = "Successfully read positions.json"
        return data
    except Exception as e:
        last_json_status = f"Error reading JSON: {e}"
        last_json_raw = ""
        print(last_json_status)
        return None


def get_my_turret_position(data, team_id: str):
    """
    Extract this team's turret entry from the JSON dict.
    Returns dict with keys like {'r': ..., 'theta': ...} or None.
    """
    try:
        turrets = data["turrets"]
        my_turret = turrets[team_id]
        return my_turret
    except Exception as e:
        print(f"Error extracting turret {team_id}: {e}")
        return None


def web_page() -> bytes:
    """
    Generate the HTML page showing current angles and JSON status.
    """
    global pan_angle, tilt_angle, last_json_status, last_json_raw

    html = f"""
    <html>
    <head>
        <title>ENME441 Turret Interim Test</title>
    </head>
    <body>
        <h1>ENME441 Turret Interim Test</h1>
        <h2>Manual Control (2 DOF)</h2>
        <p>Current pan (turret): {pan_angle:.1f}° &nbsp;&nbsp;
           Current tilt (turret): {tilt_angle:.1f}°</p>

        <form action="/" method="POST">
            <button name="cmd" value="PAN_LEFT">Pan Left (-{PAN_STEP_DEG}°)</button>
            <button name="cmd" value="PAN_RIGHT">Pan Right (+{PAN_STEP_DEG}°)</button>
            <br><br>
            <button name="cmd" value="TILT_DOWN">Tilt Down (-{TILT_STEP_DEG}°)</button>
            <button name="cmd" value="TILT_UP">Tilt Up (+{TILT_STEP_DEG}°)</button>
            <br><br>
            <button name="cmd" value="ZERO">Zero Pan/Tilt</button>
            <br><br>
            <h2>JSON Turret Location</h2>
            <button name="cmd" value="READ_JSON">Read JSON (Team {TEAM_ID})</button>
        </form>

        <h3>JSON Status:</h3>
        <pre>{last_json_status}</pre>

        <h3>Your Turret Entry (from last read):</h3>
        <pre>{last_json_raw}</pre>

        <hr>
        <p>Refresh or press a button to update.</p>
    </body>
    </html>
    """
    return bytes(html, 'utf-8')


def handle_command(cmd: str):
    """
    Process a command from the web form: move turret or read JSON.
    """
    global pan_angle, tilt_angle, turret, last_json_status, last_json_raw

    if turret is None:
        print("Turret not initialized yet.")
        return

    if cmd == "PAN_LEFT":
        pan_angle -= PAN_STEP_DEG
        print(f"PAN_LEFT -> new pan_angle={pan_angle:.1f}")
        turret.goto(pan_deg=pan_angle, tilt_deg=tilt_angle)

    elif cmd == "PAN_RIGHT":
        pan_angle += PAN_STEP_DEG
        print(f"PAN_RIGHT -> new pan_angle={pan_angle:.1f}")
        turret.goto(pan_deg=pan_angle, tilt_deg=tilt_angle)

    elif cmd == "TILT_UP":
        tilt_angle += TILT_STEP_DEG
        print(f"TILT_UP -> new tilt_angle={tilt_angle:.1f}")
        turret.goto(pan_deg=pan_angle, tilt_deg=tilt_angle)

    elif cmd == "TILT_DOWN":
        tilt_angle -= TILT_STEP_DEG
        print(f"TILT_DOWN -> new tilt_angle={tilt_angle:.1f}")
        turret.goto(pan_deg=pan_angle, tilt_deg=tilt_angle)

    elif cmd == "ZERO":
        print("ZERO -> reset logical angles to (0,0)")
        pan_angle = 0.0
        tilt_angle = 0.0
        turret.zero()
        # Optionally move to the zero position explicitly:
        turret.goto(pan_deg=0.0, tilt_deg=0.0)

    elif cmd == "READ_JSON":
        print("READ_JSON -> fetching positions.json")
        data = fetch_positions()
        if data is not None:
            my_turret = get_my_turret_position(data, TEAM_ID)
            if my_turret is not None:
                # Show only this turret's entry in the web page
                last_json_raw = json.dumps(
                    {TEAM_ID: my_turret},
                    indent=2
                )
                last_json_status = f"Team {TEAM_ID} turret: r={my_turret.get('r')}, theta={my_turret.get('theta')}"
            # else: last_json_status already updated by get_my_turret_position or fetch_positions
    else:
        print(f"Unknown cmd: {cmd}")


def serve_web_page():
    """
    Socket-based HTTP server that serves the web page and handles POST commands.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('', HTTP_PORT))
    s.listen(3)
    print(f"Serving HTTP on port {HTTP_PORT}...")

    try:
        while True:
            conn, (client_ip, client_port) = s.accept()
            request = conn.recv(2048)
            request_str = request.decode('utf-8', errors='ignore')
            first_line = request_str.split('\r\n', 1)[0]
            method = first_line.split(' ')[0] if first_line else "GET"

            if method == "POST":
                # Parse form data and handle command
                data_dict = parsePOSTdata(request)
                cmd = data_dict.get('cmd')
                if cmd:
                    handle_command(cmd)

            # In all cases, respond with the current web page
            try:
                conn.send(b"HTTP/1.1 200 OK\n")
                conn.send(b"Content-type: text/html\n")
                conn.send(b"Connection: close\r\n\r\n")
                conn.sendall(web_page())
            finally:
                conn.close()

    except KeyboardInterrupt:
        print("Server interrupted by user.")
    finally:
        print("Closing socket.")
        s.close()


# ==== MAIN =============================================================

def main():
    global turret

    # Initialize turret motors
    print("Initializing TurretMotors...")
    turret = TurretMotors(
        data_pin=16,
        latch_pin=20,
        clock_pin=21,
        pan_gear_ratio=PAN_GEAR_RATIO,
        tilt_gear_ratio=TILT_GEAR_RATIO,
    )

    print("Zeroing turret angles...")
    turret.zero()

    # Start web server in main thread (or you could use threading.Thread)
    serve_web_page()


if __name__ == "__main__":
    main()
