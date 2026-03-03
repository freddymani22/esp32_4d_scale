from flask import Flask, request, send_from_directory, render_template_string, session, redirect, url_for
from functools import wraps
import os
import json
import threading
import socket as sock_lib
import time
from datetime import datetime
from PIL import Image

# Separate trigger events for each ESP32 mode
trigger_event_mode0 = threading.Event()  # HX711 ESP32
trigger_event_mode1 = threading.Event()  # Camera ESP32
poll_lock = threading.Lock()
poll_tare_first = False
poll_remeasure_baseline = False
# Shared measurement ID sent to both ESP32s on a "both" trigger so uploads can be merged
pending_measurement_id_mode0 = None
pending_measurement_id_mode1 = None

# TCP (kept for future use, not actively used)
tcp_client_conn = None
tcp_client_lock = threading.Lock()

app = Flask(__name__)
app.secret_key = "4dscale-secret-key-change-me"
AUTH_USERNAME = "admin"
AUTH_PASSWORD = "scale1234"

UPLOAD_DIR = os.path.join(os.path.dirname(__file__), "uploads")
DATA_FILE = os.path.join(os.path.dirname(__file__), "uploads", "measurements.json")
CONFIG_FILE = os.path.join(os.path.dirname(__file__), "uploads", "config.json")
os.makedirs(UPLOAD_DIR, exist_ok=True)

DEFAULT_CAL_FACTOR = 29.46


def login_required(f):
    @wraps(f)
    def decorated(*args, **kwargs):
        if not session.get("logged_in"):
            return redirect(url_for("login", next=request.path))
        return f(*args, **kwargs)
    return decorated


@app.route("/login", methods=["GET", "POST"])
def login():
    error = None
    if request.method == "POST":
        if (request.form.get("username") == AUTH_USERNAME and
                request.form.get("password") == AUTH_PASSWORD):
            session["logged_in"] = True
            return redirect(request.args.get("next") or "/")
        error = "Invalid username or password."
    return render_template_string(LOGIN_TEMPLATE, error=error)


@app.route("/logout")
def logout():
    session.clear()
    return redirect(url_for("login"))


def load_config():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE) as f:
            return json.load(f)
    return {
        "cal_factor": DEFAULT_CAL_FACTOR,
        "sobel_threshold": 60,
        "dilation_iterations": 1,
        "min_bbox_area_pct": 2,
    }


def save_config(cfg):
    with open(CONFIG_FILE, "w") as f:
        json.dump(cfg, f, indent=2)


def load_measurements():
    if os.path.exists(DATA_FILE):
        with open(DATA_FILE) as f:
            return json.load(f)
    return []


def save_measurements(data):
    with open(DATA_FILE, "w") as f:
        json.dump(data, f, indent=2)


def convert_pgm_to_png(pgm_path):
    """Convert PGM file to PNG for browser viewing."""
    png_path = pgm_path.rsplit(".", 1)[0] + ".png"
    try:
        img = Image.open(pgm_path)
        img.save(png_path)
        os.remove(pgm_path)
        return os.path.basename(png_path)
    except Exception as e:
        print(f"PGM->PNG conversion failed: {e}")
        return os.path.basename(pgm_path)


def merge_into(existing, new_entry):
    """Merge new_entry fields into existing entry, preferring non-zero/non-empty values."""
    # Camera fields: take from new if existing is zero/empty
    for key in ("image", "length_mm", "width_mm", "length_px", "width_px",
                "angle", "height_cm", "pixels_per_mm"):
        new_val = new_entry.get(key)
        if new_val and (isinstance(new_val, str) and new_val != "" or
                        isinstance(new_val, float) and new_val != 0.0):
            existing[key] = new_val
    # valid: True if either is True
    if new_entry.get("valid"):
        existing["valid"] = True
    # HX711 fields: take from new if existing is zero
    for key in ("weight_g", "raw_tare", "raw_avg"):
        if new_entry.get(key, 0) != 0:
            existing[key] = new_entry[key]
    for key in ("tare_readings", "weight_readings"):
        val = new_entry.get(key, "")
        # Only overwrite if new value has actual non-zero readings (mode 1 sends 0,0,0,0,0)
        if val and any(c in '123456789' for c in val):
            existing[key] = val
    # Ultrasonic fields: take positive values
    for key in ("baseline_cm", "object_cm"):
        if new_entry.get(key, -1) > 0:
            existing[key] = new_entry[key]


# ESP32 sends images + measurements here
@app.route("/upload", methods=["POST"])
def upload():
    data = request.get_data()

    # Save image if present, otherwise weight-only upload (HX711 mode)
    if len(data) > 0:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        pgm_name = f"result_{timestamp}.pgm"
        pgm_path = os.path.join(UPLOAD_DIR, pgm_name)
        with open(pgm_path, "wb") as f:
            f.write(data)
        img_filename = convert_pgm_to_png(pgm_path)
        print(f"Saved: {img_filename} ({len(data)} bytes)")
    else:
        img_filename = ""
        print("Weight-only upload (no image)")

    # Parse measurements from query params
    measurement_id = request.args.get("measurement_id", "")
    entry = {
        "measurement_id": measurement_id,
        "image": img_filename,
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "valid": request.args.get("valid", "0") == "1",
        "length_mm": float(request.args.get("length_mm", 0)),
        "width_mm": float(request.args.get("width_mm", 0)),
        "length_px": float(request.args.get("length_px", 0)),
        "width_px": float(request.args.get("width_px", 0)),
        "angle": float(request.args.get("angle", 0)),
        "weight_g": float(request.args.get("weight_g", 0)),
        "raw_tare": int(request.args.get("raw_tare", 0)),
        "raw_avg": int(request.args.get("raw_avg", 0)),
        "tare_readings": request.args.get("tare_r", ""),
        "weight_readings": request.args.get("weight_r", ""),
        "baseline_cm": float(request.args.get("baseline_cm", -1)),
        "object_cm": float(request.args.get("object_cm", -1)),
        "height_cm": float(request.args.get("height_cm", 0)),
        "pixels_per_mm": float(request.args.get("pixels_per_mm", 0)),
    }

    measurements = load_measurements()

    # If measurement_id matches an existing pending entry, merge rather than insert
    if measurement_id:
        existing = next((m for m in measurements if m.get("measurement_id") == measurement_id), None)
        if existing:
            merge_into(existing, entry)
            save_measurements(measurements)
            print(f"Merged upload into measurement_id={measurement_id}")
            return {"status": "ok", "merged": True}, 200

    measurements.insert(0, entry)
    save_measurements(measurements)

    sep = "=" * 44
    print(f"\n{sep}")
    print(f"  MEASUREMENT  —  {entry['timestamp']}")
    print(sep)
    if entry["valid"]:
        print(f"  Length   : {entry['length_mm']:.1f} mm  ({entry['length_mm']/10:.2f} cm)")
        print(f"  Width    : {entry['width_mm']:.1f} mm  ({entry['width_mm']/10:.2f} cm)")
        print(f"  Height   : {entry['height_cm']:.2f} cm  ({entry['height_cm']*10:.1f} mm)")
        print(f"  Angle    : {entry['angle']:.1f} deg")
    else:
        print(f"  Detection: NO OBJECT DETECTED")
    print(f"  Weight   : {entry['weight_g']:.1f} g  ({entry['weight_g']/1000:.3f} kg)")
    print(f"  Baseline : {entry['baseline_cm']:.2f} cm")
    print(f"  Object   : {entry['object_cm']:.2f} cm" if entry['object_cm'] > 0 else "  Object   : N/A")
    print(f"  PPMM     : {entry['pixels_per_mm']:.4f}")
    print(f"  Image    : {img_filename}")
    print(f"{sep}\n")
    return {"status": "ok", "filename": img_filename}, 200


# ESP32 fetches calibration factor and trigger mode on boot
@app.route("/api/config")
def get_config():
    cfg = load_config()
    return {
        "cal_factor": cfg.get("cal_factor", DEFAULT_CAL_FACTOR),
        "trigger_mode": cfg.get("trigger_mode", "poll"),
        "baseline_cm": cfg.get("baseline_cm", 0),
        "raw_capture": cfg.get("raw_capture", False),
        "board_p0x": cfg.get("board_p0x", 50),  "board_p0y": cfg.get("board_p0y", 18),
        "board_p1x": cfg.get("board_p1x", 195), "board_p1y": cfg.get("board_p1y", 18),
        "board_p2x": cfg.get("board_p2x", 195), "board_p2y": cfg.get("board_p2y", 160),
        "board_p3x": cfg.get("board_p3x", 50),  "board_p3y": cfg.get("board_p3y", 160),
        "pixels_per_mm": cfg.get("pixels_per_mm", 0.1644),
        "sobel_threshold": cfg.get("sobel_threshold", 60),
        "dilation_iterations": cfg.get("dilation_iterations", 1),
        "min_bbox_area_pct": cfg.get("min_bbox_area_pct", 2),
    }


# User submits known weight to calibrate
@app.route("/api/calibrate", methods=["POST"])
@login_required
def calibrate():
    body = request.get_json()
    if not body or "known_weight_g" not in body:
        return {"status": "error", "message": "missing known_weight_g"}, 400

    known_weight_g = float(body["known_weight_g"])
    if known_weight_g <= 0:
        return {"status": "error", "message": "known_weight_g must be positive"}, 400

    measurements = load_measurements()
    if not measurements:
        return {"status": "error", "message": "no measurements yet"}, 400

    latest = measurements[0]
    raw_diff = abs(latest["raw_avg"] - latest["raw_tare"])
    if raw_diff == 0:
        return {"status": "error", "message": "raw_diff is zero, cannot calibrate"}, 400

    cal_factor = raw_diff / known_weight_g
    cfg = load_config()
    cfg["cal_factor"] = round(cal_factor, 4)
    save_config(cfg)

    print(f"Calibrated: raw_diff={raw_diff}, known={known_weight_g}g -> cal_factor={cfg['cal_factor']}")
    return {"status": "ok", "cal_factor": cfg["cal_factor"], "raw_diff": raw_diff}


# Save board crop coordinates + raw_capture flag
@app.route("/api/set_board_crop", methods=["POST"])
@login_required
def set_board_crop():
    body = request.get_json()
    if not body:
        return {"status": "error", "message": "missing body"}, 400
    cfg = load_config()
    if "raw_capture" in body:
        cfg["raw_capture"] = bool(body["raw_capture"])
    for key in ("board_p0x","board_p0y","board_p1x","board_p1y",
                "board_p2x","board_p2y","board_p3x","board_p3y"):
        if key in body:
            cfg[key] = int(body[key])
    if "pixels_per_mm" in body:
        cfg["pixels_per_mm"] = round(float(body["pixels_per_mm"]), 4)
    for key in ("sobel_threshold", "dilation_iterations", "min_bbox_area_pct"):
        if key in body:
            cfg[key] = int(body[key])
    save_config(cfg)
    pt_keys = ["board_p0x","board_p0y","board_p1x","board_p1y",
               "board_p2x","board_p2y","board_p3x","board_p3y","raw_capture","pixels_per_mm",
               "sobel_threshold","dilation_iterations","min_bbox_area_pct"]
    return {"status": "ok", "config": {k: cfg[k] for k in pt_keys if k in cfg}}


# Delete measurements within a date range (inclusive)
@app.route("/api/delete_range", methods=["POST"])
@login_required
def delete_range():
    body = request.get_json()
    if not body or "from_date" not in body or "to_date" not in body:
        return {"status": "error", "message": "missing from_date or to_date"}, 400
    from_date = body["from_date"]  # "YYYY-MM-DD"
    to_date   = body["to_date"]    # "YYYY-MM-DD"
    measurements = load_measurements()
    keep, remove = [], []
    for m in measurements:
        date = m.get("timestamp", "")[:10]  # "YYYY-MM-DD"
        if from_date <= date <= to_date:
            remove.append(m)
        else:
            keep.append(m)
    for m in remove:
        filename = m.get("image", "")
        if filename:
            img_path = os.path.join(UPLOAD_DIR, filename)
            if os.path.exists(img_path) and os.path.isfile(img_path):
                os.remove(img_path)
    save_measurements(keep)
    print(f"delete_range {from_date}→{to_date}: deleted {len(remove)}, kept {len(keep)}")
    return {"status": "ok", "deleted": len(remove), "kept": len(keep)}


# Delete a measurement and its image
@app.route("/api/delete", methods=["POST"])
@login_required
def delete_measurement():
    body = request.get_json()
    if not body or "filename" not in body:
        return {"status": "error", "message": "missing filename"}, 400
    filename = os.path.basename(body["filename"])  # prevent path traversal
    measurements = load_measurements()
    measurements = [m for m in measurements if m.get("image") != filename]
    save_measurements(measurements)
    if filename:
        img_path = os.path.join(UPLOAD_DIR, filename)
        if os.path.exists(img_path) and os.path.isfile(img_path):
            os.remove(img_path)
    return {"status": "ok"}


# Save trigger mode
@app.route("/api/set_trigger_mode", methods=["POST"])
@login_required
def set_trigger_mode():
    body = request.get_json()
    if not body or "trigger_mode" not in body:
        return {"status": "error", "message": "missing trigger_mode"}, 400
    mode = body["trigger_mode"]
    if mode not in ("tcp", "poll"):
        return {"status": "error", "message": "trigger_mode must be tcp or poll"}, 400
    cfg = load_config()
    cfg["trigger_mode"] = mode
    save_config(cfg)
    return {"status": "ok", "trigger_mode": mode}


# Save baseline distance
@app.route("/api/set_baseline", methods=["POST"])
@login_required
def set_baseline():
    body = request.get_json()
    if not body or "baseline_cm" not in body:
        return {"status": "error", "message": "missing baseline_cm"}, 400
    val = float(body["baseline_cm"])
    if val <= 0:
        return {"status": "error", "message": "baseline_cm must be positive"}, 400
    cfg = load_config()
    cfg["baseline_cm"] = round(val, 2)
    save_config(cfg)
    return {"status": "ok", "baseline_cm": cfg["baseline_cm"]}


# Frontend triggers a measurement
# target: "both" | "camera" (mode 1) | "weight" (mode 0)
# tare_first: bool — re-tare before weighing (weight target)
# remeasure_baseline: bool — re-measure empty board distance (camera target)
@app.route("/api/trigger", methods=["POST"])
@login_required
def trigger():
    global poll_tare_first, poll_remeasure_baseline
    global pending_measurement_id_mode0, pending_measurement_id_mode1
    body = request.get_json(silent=True) or {}
    target = body.get("target", "both")
    tare_first = bool(body.get("tare_first", False))
    remeasure_baseline = bool(body.get("remeasure_baseline", False))

    # Generate shared measurement ID (millisecond timestamp)
    measurement_id = str(int(time.time() * 1000))

    with poll_lock:
        poll_tare_first = tare_first
        poll_remeasure_baseline = remeasure_baseline
        if target in ("both", "weight"):
            pending_measurement_id_mode0 = measurement_id
        if target in ("both", "camera"):
            pending_measurement_id_mode1 = measurement_id

    if target in ("both", "weight"):
        trigger_event_mode0.set()
    if target in ("both", "camera"):
        trigger_event_mode1.set()

    print(f"Trigger: target={target} id={measurement_id} tare_first={tare_first} remeasure_baseline={remeasure_baseline}")
    return {"status": "ok", "target": target, "measurement_id": measurement_id}


# ESP32 long-polls this until trigger fires (or 60s timeout)
# ?mode=0 → HX711 ESP32, ?mode=1 → Camera ESP32
@app.route("/api/wait")
def wait_trigger():
    global poll_tare_first, poll_remeasure_baseline
    global pending_measurement_id_mode0, pending_measurement_id_mode1
    mode = request.args.get("mode", "1")

    if mode == "0":
        trigger_event_mode0.clear()
        fired = trigger_event_mode0.wait(timeout=60)
        tare = False
        mid = ""
        if fired:
            trigger_event_mode0.clear()
            with poll_lock:
                tare = poll_tare_first
                poll_tare_first = False
                mid = pending_measurement_id_mode0 or ""
                pending_measurement_id_mode0 = None
        return {"trigger": fired, "tare_first": tare, "measurement_id": mid}
    else:
        trigger_event_mode1.clear()
        fired = trigger_event_mode1.wait(timeout=60)
        remeasure = False
        mid = ""
        if fired:
            trigger_event_mode1.clear()
            with poll_lock:
                remeasure = poll_remeasure_baseline
                poll_remeasure_baseline = False
                mid = pending_measurement_id_mode1 or ""
                pending_measurement_id_mode1 = None
        return {"trigger": fired, "remeasure_baseline": remeasure, "measurement_id": mid}


# View results in browser
@app.route("/")
@login_required
def index():
    measurements = load_measurements()
    cfg = load_config()
    return render_template_string(PAGE_TEMPLATE, measurements=measurements, config=cfg)


@app.route("/files/<filename>")
def serve_file(filename):
    return send_from_directory(UPLOAD_DIR, filename)


LOGIN_TEMPLATE = """
<html>
<head>
    <title>ESP32 4D Scale — Login</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { font-family: 'Segoe UI', monospace; background: #0f0f0f; color: #e0e0e0;
               display: flex; justify-content: center; align-items: center; min-height: 100vh; }
        .box { background: #1a1a1a; border: 1px solid #333; border-radius: 10px; padding: 36px 40px; width: 320px; }
        h1 { color: #4fc3f7; font-size: 20px; margin-bottom: 6px; }
        .sub { color: #888; font-size: 12px; margin-bottom: 24px; }
        label { color: #aaa; font-size: 13px; display: block; margin-bottom: 6px; }
        input { width: 100%; background: #222; border: 1px solid #444; border-radius: 6px;
                color: #fff; padding: 9px 12px; font-size: 14px; margin-bottom: 16px; }
        input:focus { outline: none; border-color: #4fc3f7; }
        button { width: 100%; background: #4fc3f7; color: #000; border: none; border-radius: 6px;
                 padding: 10px; font-size: 14px; font-weight: bold; cursor: pointer; }
        button:hover { background: #29b6f6; }
        .error { color: #ef5350; font-size: 13px; margin-bottom: 14px; }
    </style>
</head>
<body>
    <div class="box">
        <h1>ESP32 4D Scale</h1>
        <p class="sub">Sign in to continue</p>
        {% if error %}<div class="error">{{ error }}</div>{% endif %}
        <form method="POST">
            <label>Username</label>
            <input type="text" name="username" autofocus autocomplete="username">
            <label>Password</label>
            <input type="password" name="password" autocomplete="current-password">
            <button type="submit">Sign In</button>
        </form>
    </div>
</body>
</html>
"""

PAGE_TEMPLATE = """
<html>
<head>
    <title>4D Scale Dashboard</title>
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg: #07070f;
            --surface: #0f0f1c;
            --surface2: #161628;
            --surface3: #1e1e35;
            --border: rgba(255,255,255,0.07);
            --border-h: rgba(255,255,255,0.14);
            --blue: #38bdf8; --blue-g: rgba(56,189,248,0.12);
            --green: #4ade80; --green-g: rgba(74,222,128,0.12);
            --amber: #fbbf24; --amber-g: rgba(251,191,36,0.12);
            --pink: #f472b6; --pink-g: rgba(244,114,182,0.12);
            --red: #f87171;
            --purple: #a78bfa;
            --orange: #fb923c;
            --t1: #f1f5f9; --t2: #94a3b8; --t3: #475569;
            --r: 12px; --rs: 8px;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { font-family: 'Inter', sans-serif; background: var(--bg); color: var(--t1); min-height: 100vh; }

        /* ── Header ── */
        .hdr {
            background: linear-gradient(135deg, #0d0d1f 0%, #13102a 100%);
            border-bottom: 1px solid var(--border);
            padding: 0 28px; height: 60px;
            display: flex; align-items: center; justify-content: space-between;
            position: sticky; top: 0; z-index: 100;
        }
        .hdr-l { display: flex; align-items: center; gap: 12px; }
        .logo {
            width: 34px; height: 34px; border-radius: 9px;
            background: linear-gradient(135deg, #38bdf8, #818cf8);
            display: flex; align-items: center; justify-content: center;
            font-size: 14px; font-weight: 800; color: #000; letter-spacing: -1px;
        }
        .hdr-title { font-size: 15px; font-weight: 700; }
        .hdr-sub { font-size: 11px; color: var(--t3); margin-top: 1px; }
        .hdr-r { display: flex; align-items: center; gap: 10px; }
        .chip {
            background: var(--surface2); border: 1px solid var(--border);
            border-radius: 20px; padding: 4px 12px;
            font-size: 11px; color: var(--t2);
        }
        .chip b { color: var(--blue); }
        .logout {
            color: var(--t2); font-size: 12px; text-decoration: none;
            border: 1px solid var(--border); border-radius: var(--rs);
            padding: 5px 13px; transition: all 0.2s; font-family: 'Inter', sans-serif;
        }
        .logout:hover { color: var(--t1); border-color: var(--border-h); }

        /* ── Layout ── */
        .wrap { padding: 24px 28px; max-width: 1440px; margin: 0 auto; }
        .sec-label {
            font-size: 10px; font-weight: 700; letter-spacing: 1.8px;
            text-transform: uppercase; color: var(--t3); margin-bottom: 12px;
        }

        /* ── Panels ── */
        .panels { display: grid; grid-template-columns: repeat(auto-fill, minmax(290px, 1fr)); gap: 14px; margin-bottom: 28px; }
        .panel {
            background: var(--surface); border: 1px solid var(--border);
            border-radius: var(--r); padding: 20px;
            position: relative; overflow: hidden;
            transition: border-color 0.2s;
        }
        .panel:hover { border-color: var(--border-h); }
        .panel::after {
            content: ''; position: absolute;
            top: 0; left: 0; right: 0; height: 1px;
        }
        .panel.blue::after  { background: linear-gradient(90deg, var(--blue) 0%, transparent 70%); }
        .panel.green::after { background: linear-gradient(90deg, var(--green) 0%, transparent 70%); }
        .panel.amber::after { background: linear-gradient(90deg, var(--amber) 0%, transparent 70%); }
        .panel.pink::after  { background: linear-gradient(90deg, var(--pink) 0%, transparent 70%); }
        .ph { display: flex; align-items: center; gap: 10px; margin-bottom: 14px; }
        .pico {
            width: 30px; height: 30px; border-radius: 7px;
            display: flex; align-items: center; justify-content: center; font-size: 14px;
        }
        .panel.blue  .pico { background: var(--blue-g);  color: var(--blue); }
        .panel.green .pico { background: var(--green-g); color: var(--green); }
        .panel.amber .pico { background: var(--amber-g); color: var(--amber); }
        .panel.pink  .pico { background: var(--pink-g);  color: var(--pink); }
        .ptitle { font-size: 13px; font-weight: 700; }
        .panel.blue  .ptitle { color: var(--blue); }
        .panel.green .ptitle { color: var(--green); }
        .panel.amber .ptitle { color: var(--amber); }
        .panel.pink  .ptitle { color: var(--pink); }
        .pdesc { font-size: 12px; color: var(--t2); line-height: 1.65; margin-bottom: 14px; }
        .cval {
            background: var(--surface2); border: 1px solid var(--border);
            border-radius: var(--rs); padding: 9px 12px;
            font-size: 12px; color: var(--t2); margin-bottom: 12px;
        }
        .cval b { color: var(--green); }
        hr { border: none; border-top: 1px solid var(--border); margin: 13px 0; }
        .row { display: flex; gap: 8px; align-items: center; margin-bottom: 9px; flex-wrap: wrap; }
        .row label { font-size: 12px; color: var(--t2); font-weight: 500; min-width: 108px; }
        input[type=number], input[type=text], input[type=date] {
            background: var(--surface2); border: 1px solid var(--border);
            border-radius: var(--rs); color: var(--t1);
            padding: 7px 10px; font-size: 12px; font-family: 'Inter', sans-serif;
            width: 105px; transition: border-color 0.2s;
        }
        input:focus { outline: none; border-color: var(--blue); }
        input[type=date] { width: auto; color-scheme: dark; }
        .hint { color: var(--t3); font-size: 11px; margin-top: 5px; }
        .result {
            font-size: 12px; margin-top: 8px; display: none;
            padding: 7px 10px; border-radius: var(--rs); background: var(--surface2);
        }
        .check-row { display: flex; align-items: center; gap: 7px; margin-bottom: 11px; cursor: pointer; }
        .check-row input[type=checkbox] { accent-color: var(--blue); width: 14px; height: 14px; }
        .check-row span { font-size: 12px; color: var(--t2); }
        .board-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 7px; margin-bottom: 11px; }
        .board-cell {
            background: var(--surface2); border: 1px solid var(--border);
            border-radius: var(--rs); padding: 9px; text-align: center;
        }
        .board-cell .dim-label { color: var(--t3); font-size: 9px; letter-spacing: 1px; text-transform: uppercase; margin-bottom: 5px; }
        .board-cell .inputs { display: flex; gap: 5px; justify-content: center; }
        .board-cell input { width: 52px !important; padding: 4px 3px !important; text-align: center; font-size: 11px !important; }

        /* ── Buttons ── */
        .btn {
            border: none; border-radius: var(--rs); padding: 8px 16px;
            font-size: 12px; font-weight: 600; cursor: pointer;
            font-family: 'Inter', sans-serif; transition: all 0.15s;
            display: inline-flex; align-items: center; gap: 5px;
        }
        .btn:disabled { opacity: 0.35; cursor: not-allowed; transform: none !important; }
        .btn-blue  { background: var(--blue);  color: #000; }
        .btn-blue:not(:disabled):hover  { background: #7dd3fc; transform: translateY(-1px); box-shadow: 0 6px 20px var(--blue-g); }
        .btn-green { background: var(--green); color: #000; }
        .btn-green:not(:disabled):hover { background: #86efac; transform: translateY(-1px); box-shadow: 0 6px 20px var(--green-g); }
        .btn-amber { background: var(--amber); color: #000; }
        .btn-amber:not(:disabled):hover { background: #fcd34d; transform: translateY(-1px); box-shadow: 0 6px 20px var(--amber-g); }
        .btn-pink  { background: var(--pink);  color: #000; }
        .btn-pink:not(:disabled):hover  { background: #f9a8d4; transform: translateY(-1px); box-shadow: 0 6px 20px var(--pink-g); }
        .btn-red   { background: transparent; color: var(--red); border: 1px solid rgba(248,113,113,0.25); }
        .btn-red:not(:disabled):hover   { background: rgba(248,113,113,0.08); border-color: var(--red); }

        /* ── Delete bar ── */
        .del-bar {
            background: var(--surface); border: 1px solid var(--border);
            border-radius: var(--r); padding: 12px 18px;
            display: flex; align-items: center; gap: 10px; flex-wrap: wrap;
            margin-bottom: 22px;
        }
        .del-bar-lbl { font-size: 12px; font-weight: 500; color: var(--t2); white-space: nowrap; }

        /* ── Grid ── */
        .grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(400px, 1fr)); gap: 14px; }
        .card {
            background: var(--surface); border: 1px solid var(--border);
            border-radius: var(--r); overflow: hidden;
            transition: border-color 0.2s, transform 0.2s, box-shadow 0.2s;
        }
        .card:hover { border-color: var(--border-h); transform: translateY(-2px); box-shadow: 0 8px 32px rgba(0,0,0,0.4); }
        .card img { width: 100%; display: block; image-rendering: pixelated; border-bottom: 1px solid var(--border); }
        .card-body { padding: 14px; }
        .card-hdr { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; }
        .card-time { color: var(--t3); font-size: 11px; }
        .badge {
            font-size: 10px; font-weight: 700; letter-spacing: 0.5px;
            padding: 3px 9px; border-radius: 20px; text-transform: uppercase;
        }
        .badge.ok  { background: rgba(74,222,128,0.1);  color: var(--green); border: 1px solid rgba(74,222,128,0.2); }
        .badge.err { background: rgba(248,113,113,0.1); color: var(--red);   border: 1px solid rgba(248,113,113,0.2); }
        .no-detect {
            text-align: center; padding: 10px; color: var(--red);
            font-size: 12px; font-weight: 500;
            background: rgba(248,113,113,0.05); border-radius: var(--rs); margin-bottom: 10px;
        }
        .dims { display: grid; grid-template-columns: repeat(3, 1fr); gap: 7px; margin-bottom: 10px; }
        .dim-box {
            background: var(--surface2); border: 1px solid var(--border);
            border-radius: var(--rs); padding: 9px 6px; text-align: center;
        }
        .dim-label { color: var(--t3); font-size: 9px; text-transform: uppercase; letter-spacing: 0.8px; margin-bottom: 3px; }
        .dim-value { font-size: 16px; font-weight: 700; line-height: 1.2; }
        .dim-value.w  { color: var(--blue); }
        .dim-value.h  { color: var(--green); }
        .dim-value.wt { color: var(--amber); }
        .dim-value.a  { color: var(--purple); }
        .dim-value.d  { color: var(--orange); }
        .dim-value.ht { color: var(--pink); }
        .dim-sub { color: var(--t3); font-size: 10px; margin-top: 2px; }
        .raw { font-size: 10px; color: var(--t3); font-family: 'Courier New', monospace;
               background: var(--surface2); padding: 5px 8px; border-radius: 5px; margin-bottom: 4px; }
        .raw-grid {
            display: grid; grid-template-columns: repeat(4, 1fr);
            gap: 5px; margin-bottom: 8px;
        }
        .raw-cell {
            background: var(--surface2); border: 1px solid var(--border);
            border-radius: var(--rs); padding: 6px 8px; text-align: center;
        }
        .raw-cell .rc-lbl { font-size: 9px; text-transform: uppercase; letter-spacing: 0.8px; color: var(--t3); margin-bottom: 3px; }
        .raw-cell .rc-val { font-size: 12px; font-weight: 600; font-family: 'Courier New', monospace; color: var(--t2); }
        .card-foot { display: flex; justify-content: flex-end; margin-top: 8px; }
        .empty { text-align: center; padding: 80px 20px; color: var(--t3); }
        .empty-icon { font-size: 40px; opacity: 0.25; display: block; margin-bottom: 14px; }

        @media (max-width: 900px) {
            .panels { grid-template-columns: repeat(auto-fill, minmax(260px, 1fr)); }
            .grid   { grid-template-columns: repeat(auto-fill, minmax(300px, 1fr)); }
        }
        @media (max-width: 600px) {
            .hdr { padding: 0 12px; height: 52px; }
            .hdr-sub, .chip { display: none; }
            .logo { width: 28px; height: 28px; font-size: 11px; border-radius: 7px; }
            .hdr-title { font-size: 13px; }
            .wrap { padding: 12px; }
            .panels, .grid { grid-template-columns: 1fr; }
            .board-grid { grid-template-columns: 1fr 1fr; }
            .board-cell input { width: 44px !important; font-size: 10px !important; }
            .del-bar { flex-direction: column; align-items: flex-start; gap: 8px; }
            .del-bar input[type=date] { width: 100%; }
            .dims { grid-template-columns: repeat(3, 1fr); gap: 5px; }
            .dim-value { font-size: 13px; }
            .raw-grid { grid-template-columns: repeat(2, 1fr); }
            .row label { min-width: 90px; }
        }
    </style>
</head>
<body>
<header class="hdr">
    <div class="hdr-l">
        <div class="logo">4D</div>
        <div>
            <div class="hdr-title">4D Scale Dashboard</div>
            <div class="hdr-sub">ESP32 Measurement System</div>
        </div>
    </div>
    <div class="hdr-r">
        <div class="chip"><b>{{ measurements|length }}</b> measurement{{ 's' if measurements|length != 1 }}</div>
        <a href="/logout" class="logout">Sign out</a>
    </div>
</header>

<div class="wrap">
    <p class="sec-label">Controls</p>
    <div class="panels">

        <!-- MEASURE -->
        <div class="panel blue">
            <div class="ph"><div class="pico">&#9654;</div><div class="ptitle">Trigger Measurement</div></div>
            <p class="pdesc">Fires both ESP32s — camera captures L×W×H, HX711 measures weight.</p>
            <button class="btn btn-blue" id="btnMeasure" onclick="doTrigger('both')">&#9654;&nbsp; Measure Now</button>
            <div class="result" id="resultMeasure"></div>
        </div>

        <!-- BASELINE -->
        <div class="panel green">
            <div class="ph"><div class="pico">&#8645;</div><div class="ptitle">Baseline Distance</div></div>
            <div class="cval">Current: <b id="currentBaseline">{{ config.get('baseline_cm', 0) }} cm</b>{% if config.get('baseline_cm', 0) == 0 %} <span style="color:var(--t3)">(auto at boot)</span>{% endif %}</div>
            <p class="pdesc">Place empty board and re-measure, or enter manually below.</p>
            <button class="btn btn-green" id="btnBaseline" onclick="doTrigger('camera', {remeasure_baseline: true})" style="margin-bottom:10px;">&#8635;&nbsp; Re-measure</button>
            <div class="result" id="resultBaseline"></div>
            <hr>
            <div class="row">
                <label>Distance (cm)</label>
                <input type="number" id="baselineInput" step="0.1" min="1" placeholder="e.g. 149.0">
                <button class="btn btn-green" onclick="doSetBaseline()">Save</button>
            </div>
            <div class="result" id="resultBaselineSave"></div>
        </div>

        <!-- WEIGHT CAL -->
        <div class="panel amber">
            <div class="ph"><div class="pico">&#9878;</div><div class="ptitle">Weight Calibration</div></div>
            <p class="pdesc">
                <b style="color:var(--t1)">1.</b> Empty board + Tare → Weigh<br>
                <b style="color:var(--t1)">2.</b> Known weight, no tare → Weigh<br>
                <b style="color:var(--t1)">3.</b> Enter weight → Calibrate
            </p>
            <label class="check-row">
                <input type="checkbox" id="tareFirst"><span>Tare first (board empty)</span>
            </label>
            <button class="btn btn-amber" id="btnWeightTrigger" onclick="doWeightTrigger()" style="margin-bottom:10px;">&#9654;&nbsp; Weigh</button>
            <div class="result" id="resultWeightTrigger"></div>
            <hr>
            <div class="row">
                <label>Known weight (g)</label>
                <input type="number" id="knownWeight" placeholder="e.g. 500" step="0.1" style="width:95px;">
                <button class="btn btn-amber" onclick="doWeightCalibrate()">Calibrate</button>
            </div>
            <div class="cval" style="margin-top:10px;margin-bottom:0;">cal_factor: <b id="calFactorDisplay">{{ config.get('cal_factor', 27.93) }}</b></div>
            <div class="result" id="resultWeightCal"></div>
        </div>

        <!-- BOARD CAL -->
        <div class="panel pink">
            <div class="ph"><div class="pico">&#9974;</div><div class="ptitle">Board Calibration</div></div>
            <p class="pdesc">Capture raw image, set board corners, tune image params.</p>
            <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:11px;">
                <label class="check-row" style="margin-bottom:0;">
                    <input type="checkbox" id="rawCaptureCheck" {% if config.get('raw_capture') %}checked{% endif %} onchange="doSetRawCapture(this.checked)">
                    <span>Raw capture mode</span>
                </label>
                <button class="btn btn-pink" id="btnCapture" onclick="doCaptureRaw()">&#128247; Capture</button>
            </div>
            <div class="result" id="resultCapture"></div>
            <div class="board-grid">
                <div class="board-cell">
                    <div class="dim-label">P0 Top-Left</div>
                    <div class="inputs">
                        <input type="number" id="bp0x" value="{{ config.get('board_p0x', 50) }}">
                        <input type="number" id="bp0y" value="{{ config.get('board_p0y', 18) }}">
                    </div>
                </div>
                <div class="board-cell">
                    <div class="dim-label">P1 Top-Right</div>
                    <div class="inputs">
                        <input type="number" id="bp1x" value="{{ config.get('board_p1x', 195) }}">
                        <input type="number" id="bp1y" value="{{ config.get('board_p1y', 18) }}">
                    </div>
                </div>
                <div class="board-cell">
                    <div class="dim-label">P3 Bot-Left</div>
                    <div class="inputs">
                        <input type="number" id="bp3x" value="{{ config.get('board_p3x', 50) }}">
                        <input type="number" id="bp3y" value="{{ config.get('board_p3y', 160) }}">
                    </div>
                </div>
                <div class="board-cell">
                    <div class="dim-label">P2 Bot-Right</div>
                    <div class="inputs">
                        <input type="number" id="bp2x" value="{{ config.get('board_p2x', 195) }}">
                        <input type="number" id="bp2y" value="{{ config.get('board_p2y', 160) }}">
                    </div>
                </div>
            </div>
            <div class="row"><label>Pixels/mm</label><input type="number" id="bppmm" value="{{ config.get('pixels_per_mm', 0.1644) }}" step="0.0001" style="width:85px;"></div>
            <div class="row"><label>Sobel Threshold</label><input type="number" id="bsobel" value="{{ config.get('sobel_threshold', 60) }}" min="0" max="2040" step="10" style="width:80px;"></div>
            <div class="row"><label>Dilation Iters</label><input type="number" id="bdilation" value="{{ config.get('dilation_iterations', 1) }}" min="1" max="5" style="width:80px;"></div>
            <div class="row"><label>Min BBox Area %</label><input type="number" id="bminbbox" value="{{ config.get('min_bbox_area_pct', 2) }}" min="1" max="10" style="width:80px;"></div>
            <div class="hint">Image 320×240 px &middot; x, y per corner</div>
            <button class="btn btn-pink" onclick="doSaveCrop()" style="width:100%;margin-top:10px;justify-content:center;">&#10003;&nbsp; Save All</button>
            <div class="result" id="resultCrop"></div>
        </div>

    </div>

    <!-- DELETE BAR -->
    <div class="del-bar">
        <span class="del-bar-lbl">&#128465; Delete by date:</span>
        <input type="date" id="fromDate">
        <span style="color:var(--t3);font-size:12px;">→</span>
        <input type="date" id="toDate">
        <button class="btn btn-red" onclick="doDeleteRange()">Delete Range</button>
        <span id="resultDeleteRange" style="font-size:12px;display:none;"></span>
    </div>

    <!-- MEASUREMENTS -->
    <p class="sec-label">Measurements</p>
    {% if not measurements %}
    <div class="empty">
        <span class="empty-icon">&#128202;</span>
        Waiting for ESP32 to upload results...
    </div>
    {% endif %}

    <div class="grid">
    {% for m in measurements %}
        <div class="card">
            {% if m.image %}<img src="/files/{{ m.image }}" alt="result">{% endif %}
            <div class="card-body">
                <div class="card-hdr">
                    <span class="card-time">{{ m.timestamp }}</span>
                    {% if m.valid %}<span class="badge ok">&#10003; Detected</span>
                    {% else %}<span class="badge err">&#10007; No Object</span>{% endif %}
                </div>
                {% if not m.valid %}<div class="no-detect">No object detected</div>{% endif %}
                <div class="dims">
                    {% if m.valid %}
                    <div class="dim-box">
                        <div class="dim-label">Length</div>
                        <div class="dim-value w">{{ "%.1f"|format(m.get('length_mm', 0) / 10) }}</div>
                        <div class="dim-sub">cm &middot; {{ "%.0f"|format(m.get('length_mm', 0)) }}mm</div>
                    </div>
                    <div class="dim-box">
                        <div class="dim-label">Width</div>
                        <div class="dim-value h">{{ "%.1f"|format(m.get('width_mm', 0) / 10) }}</div>
                        <div class="dim-sub">cm &middot; {{ "%.0f"|format(m.get('width_mm', 0)) }}mm</div>
                    </div>
                    <div class="dim-box">
                        <div class="dim-label">Height</div>
                        {% if m.get('height_cm', 0) > 0 %}
                        <div class="dim-value ht">{{ "%.1f"|format(m.get('height_cm', 0)) }}</div>
                        <div class="dim-sub">cm &middot; {{ "%.0f"|format(m.get('height_cm', 0)*10) }}mm</div>
                        {% else %}<div class="dim-value ht" style="font-size:13px;">N/A</div>{% endif %}
                    </div>
                    {% endif %}
                    <div class="dim-box">
                        <div class="dim-label">Weight</div>
                        <div class="dim-value wt">{{ "%.0f"|format(m.weight_g) }}</div>
                        <div class="dim-sub">g &middot; {{ "%.3f"|format(m.weight_g/1000) }}kg</div>
                    </div>
                    {% if m.valid %}
                    <div class="dim-box">
                        <div class="dim-label">Angle</div>
                        <div class="dim-value a">{{ "%.1f"|format(m.angle) }}&deg;</div>
                        <div class="dim-sub">rotation</div>
                    </div>
                    <div class="dim-box">
                        <div class="dim-label">Obj Dist</div>
                        {% if m.get('object_cm', -1) > 0 %}
                        <div class="dim-value d">{{ "%.1f"|format(m.get('object_cm', 0)) }}</div>
                        <div class="dim-sub">cm</div>
                        {% else %}<div class="dim-value d" style="font-size:13px;">N/A</div>{% endif %}
                    </div>
                    {% else %}
                    <div class="dim-box">
                        <div class="dim-label">Height</div>
                        {% if m.get('height_cm', 0) > 0 %}
                        <div class="dim-value ht">{{ "%.1f"|format(m.get('height_cm', 0)) }}</div>
                        <div class="dim-sub">cm</div>
                        {% else %}<div class="dim-value ht" style="font-size:13px;">N/A</div>{% endif %}
                    </div>
                    <div class="dim-box">
                        <div class="dim-label">Ultrasonic</div>
                        {% if m.get('object_cm', -1) > 0 %}
                        <div class="dim-value d">{{ "%.1f"|format(m.get('object_cm', 0)) }}</div>
                        <div class="dim-sub">cm</div>
                        {% else %}<div class="dim-value d" style="font-size:13px;">N/A</div>{% endif %}
                    </div>
                    {% endif %}
                </div>
                <div class="raw-grid">
                    <div class="raw-cell">
                        <div class="rc-lbl">Tare</div>
                        <div class="rc-val">{{ m.raw_tare }}</div>
                    </div>
                    <div class="raw-cell">
                        <div class="rc-lbl">Avg</div>
                        <div class="rc-val">{{ m.raw_avg }}</div>
                    </div>
                    <div class="raw-cell">
                        <div class="rc-lbl">Diff</div>
                        <div class="rc-val" style="color:var(--blue);">{{ (m.raw_avg - m.raw_tare)|abs }}</div>
                    </div>
                    <div class="raw-cell">
                        <div class="rc-lbl">ppmm</div>
                        <div class="rc-val" style="color:var(--purple);">{% if m.get('pixels_per_mm', 0) > 0 %}{{ "%.4f"|format(m.pixels_per_mm) }}{% else %}—{% endif %}</div>
                    </div>
                </div>
                {% if m.get('tare_readings') %}<div class="raw">tare readings: {{ m.tare_readings }}</div>{% endif %}
                {% if m.get('weight_readings') %}<div class="raw">weight readings: {{ m.weight_readings }}</div>{% endif %}
                <div class="card-foot">
                    <button onclick="doDelete('{{ m.image }}')" class="btn btn-red" style="padding:4px 10px;font-size:11px;">&#128465; Delete</button>
                </div>
            </div>
        </div>
    {% endfor %}
    </div>
</div>

    <script>
    const API_BASE = '';

    async function doTrigger(target, extra) {
        const btnMap = {both: 'btnMeasure', camera: 'btnBaseline', weight: 'btnWeightTrigger'};
        const resultMap = {both: 'resultMeasure', camera: 'resultBaseline', weight: 'resultWeightTrigger'};
        const btn = document.getElementById(btnMap[target]);
        const result = document.getElementById(resultMap[target]);
        if (btn) { btn.disabled = true; btn.textContent = '...'; }
        if (result) result.style.display = 'none';
        try {
            const body = Object.assign({target}, extra || {});
            const resp = await fetch(API_BASE + '/api/trigger', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(body)
            });
            const data = await resp.json();
            if (result) {
                result.style.display = 'block';
                if (resp.ok) {
                    result.style.color = '#81c784';
                    const msgs = {
                        both: 'Triggered both ESP32s — measuring...',
                        camera: 'Triggered camera ESP32...',
                        weight: 'Triggered HX711 ESP32 — taring then weighing...'
                    };
                    result.textContent = msgs[target] || 'Triggered.';
                } else {
                    result.style.color = '#ef5350';
                    result.textContent = 'Error: ' + data.message;
                }
            }
        } catch (e) {
            if (result) {
                result.style.display = 'block';
                result.style.color = '#ef5350';
                result.textContent = 'Request failed: ' + e.message;
            }
        }
        if (btn) {
            btn.disabled = false;
            const labels = {btnMeasure: '\u25b6 Measure', btnBaseline: '\u8635 Re-measure Baseline', btnWeightTrigger: '\u25b6 Tare & Weigh'};
            btn.textContent = labels[btn.id] || 'Trigger';
        }
    }

    async function doWeightTrigger() {
        const tareFirst = document.getElementById('tareFirst').checked;
        const btn = document.getElementById('btnWeightTrigger');
        const result = document.getElementById('resultWeightTrigger');
        btn.disabled = true; btn.textContent = '...';
        result.style.display = 'none';
        try {
            const resp = await fetch(API_BASE + '/api/trigger', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({target: 'weight', tare_first: tareFirst})
            });
            const data = await resp.json();
            result.style.display = 'block';
            if (resp.ok) {
                result.style.color = '#ffb74d';
                result.textContent = tareFirst ? 'Taring then weighing (board empty)...' : 'Weighing with stored tare...';
            } else {
                result.style.color = '#ef5350';
                result.textContent = 'Error: ' + data.message;
            }
        } catch(e) {
            result.style.display = 'block';
            result.style.color = '#ef5350';
            result.textContent = 'Request failed: ' + e.message;
        }
        btn.disabled = false; btn.textContent = '\u25b6 Weigh';
    }

    async function doCaptureRaw() {
        const btn = document.getElementById('btnCapture');
        const result = document.getElementById('resultCapture');
        // Enable raw capture, then trigger camera
        await fetch(API_BASE + '/api/set_board_crop', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({raw_capture: true})
        });
        document.getElementById('rawCaptureCheck').checked = true;
        btn.disabled = true; btn.textContent = '...';
        result.style.display = 'none';
        try {
            const resp = await fetch(API_BASE + '/api/trigger', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({target: 'camera'})
            });
            const data = await resp.json();
            result.style.display = 'block';
            if (resp.ok) {
                result.style.color = '#f06292';
                result.textContent = 'Camera triggered — raw image incoming...';
            } else {
                result.style.color = '#ef5350';
                result.textContent = 'Error: ' + data.message;
            }
        } catch(e) {
            result.style.display = 'block';
            result.style.color = '#ef5350';
            result.textContent = 'Request failed: ' + e.message;
        }
        btn.disabled = false; btn.textContent = '\U0001F4F7 Capture Raw';
    }

    async function doDeleteRange() {
        const from = document.getElementById('fromDate').value;
        const to   = document.getElementById('toDate').value;
        const result = document.getElementById('resultDeleteRange');
        if (!from || !to) {
            result.style.display = 'inline'; result.style.color = '#ef5350';
            result.textContent = 'Select both dates.'; return;
        }
        if (!confirm(`Delete all measurements from ${from} to ${to}?`)) return;
        const resp = await fetch(API_BASE + '/api/delete_range', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({from_date: from, to_date: to})
        });
        const data = await resp.json();
        result.style.display = 'inline';
        if (data.status === 'ok') {
            result.style.color = '#81c784';
            result.textContent = `Deleted ${data.deleted}, kept ${data.kept}.`;
            setTimeout(() => location.reload(), 1000);
        } else {
            result.style.color = '#ef5350';
            result.textContent = 'Error: ' + data.message;
        }
    }

    async function doDelete(filename) {
        if (!confirm('Delete this measurement and image?')) return;
        const resp = await fetch(API_BASE + '/api/delete', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({filename: filename})
        });
        const data = await resp.json();
        if (data.status === 'ok') location.reload();
        else alert('Delete failed: ' + data.message);
    }

    async function doSetRawCapture(enabled) {
        await fetch(API_BASE + '/api/set_board_crop', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({raw_capture: enabled})
        });
    }

    async function doWeightCalibrate() {
        const result = document.getElementById('resultWeightCal');
        const known = parseFloat(document.getElementById('knownWeight').value);
        if (!known || known <= 0) {
            result.style.display = 'block'; result.style.color = '#f44';
            result.textContent = 'Enter a valid known weight in grams.'; return;
        }
        result.style.display = 'block'; result.style.color = '#aaa';
        result.textContent = 'Calibrating...';
        try {
            const resp = await fetch(API_BASE + '/api/calibrate', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({known_weight_g: known})
            });
            const data = await resp.json();
            if (data.status === 'ok') {
                result.style.color = '#4caf50';
                result.textContent = `Done! cal_factor = ${data.cal_factor}  (raw_diff = ${data.raw_diff})`;
                document.getElementById('calFactorDisplay').textContent = data.cal_factor;
            } else {
                result.style.color = '#f44';
                result.textContent = `Error: ${data.message}`;
            }
        } catch(e) {
            result.style.color = '#f44'; result.textContent = 'Request failed.';
        }
    }

    async function doSaveCrop() {
        const result = document.getElementById('resultCrop');
        const body = {
            board_p0x: parseInt(document.getElementById('bp0x').value),
            board_p0y: parseInt(document.getElementById('bp0y').value),
            board_p1x: parseInt(document.getElementById('bp1x').value),
            board_p1y: parseInt(document.getElementById('bp1y').value),
            board_p2x: parseInt(document.getElementById('bp2x').value),
            board_p2y: parseInt(document.getElementById('bp2y').value),
            board_p3x: parseInt(document.getElementById('bp3x').value),
            board_p3y: parseInt(document.getElementById('bp3y').value),
            pixels_per_mm: parseFloat(document.getElementById('bppmm').value),
            sobel_threshold: parseInt(document.getElementById('bsobel').value),
            dilation_iterations: parseInt(document.getElementById('bdilation').value),
            min_bbox_area_pct: parseInt(document.getElementById('bminbbox').value),
        };
        const resp = await fetch(API_BASE + '/api/set_board_crop', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(body)
        });
        const data = await resp.json();
        result.style.display = 'block';
        if (resp.ok) {
            result.style.color = '#f06292';
            result.textContent = 'Corners saved. Takes effect on next trigger.';
        } else {
            result.style.color = '#ef5350';
            result.textContent = 'Error: ' + data.message;
        }
    }

    async function doSetBaseline() {
        const input = document.getElementById('baselineInput');
        const result = document.getElementById('resultBaselineSave');
        const val = parseFloat(input.value);
        if (!val || val <= 0) {
            result.style.display = 'block'; result.style.color = '#ef5350';
            result.textContent = 'Enter a valid distance in cm.'; return;
        }
        const resp = await fetch(API_BASE + '/api/set_baseline', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({baseline_cm: val})
        });
        const data = await resp.json();
        result.style.display = 'block';
        if (resp.ok) {
            result.style.color = '#81c784';
            result.textContent = 'Saved! Takes effect on next trigger.';
            document.getElementById('currentBaseline').textContent = data.baseline_cm;
        } else {
            result.style.color = '#ef5350';
            result.textContent = 'Error: ' + data.message;
        }
    }
    </script>
</body>
</html>
"""


def tcp_server_thread():
    """Accept ESP32 TCP connections (kept for future use)."""
    global tcp_client_conn
    server_sock = sock_lib.socket(sock_lib.AF_INET, sock_lib.SOCK_STREAM)
    server_sock.setsockopt(sock_lib.SOL_SOCKET, sock_lib.SO_REUSEADDR, 1)
    server_sock.bind(("0.0.0.0", 9000))
    server_sock.listen(2)
    print("TCP trigger server listening on port 9000")
    while True:
        try:
            conn, addr = server_sock.accept()
            print(f"TCP: ESP32 connected from {addr}")
            with tcp_client_lock:
                if tcp_client_conn:
                    try:
                        tcp_client_conn.close()
                    except Exception:
                        pass
                tcp_client_conn = conn
        except Exception as e:
            print(f"TCP server error: {e}")
            import time
            time.sleep(1)


if __name__ == "__main__":
    t = threading.Thread(target=tcp_server_thread, daemon=True)
    t.start()

    hostname = sock_lib.gethostname()
    local_ip = sock_lib.gethostbyname(hostname)
    print(f"\n{'='*50}")
    print(f"  ESP32 4D Scale - Image Server")
    print(f"  Upload:  http://{local_ip}:8080/upload")
    print(f"  Viewer:  http://{local_ip}:8080/")
    print(f"  Trigger: POST http://{local_ip}:8080/api/trigger")
    print(f"  Saves to: {UPLOAD_DIR}")
    print(f"{'='*50}\n")
    app.run(host="0.0.0.0", port=8080, threaded=True)
