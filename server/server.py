from flask import Flask, request, send_from_directory, render_template_string
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
UPLOAD_DIR = os.path.join(os.path.dirname(__file__), "uploads")
DATA_FILE = os.path.join(os.path.dirname(__file__), "uploads", "measurements.json")
CONFIG_FILE = os.path.join(os.path.dirname(__file__), "uploads", "config.json")
os.makedirs(UPLOAD_DIR, exist_ok=True)

DEFAULT_CAL_FACTOR = 29.46


def load_config():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE) as f:
            return json.load(f)
    return {"cal_factor": DEFAULT_CAL_FACTOR}


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
        if new_entry.get(key, ""):
            existing[key] = new_entry[key]
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
    }


# User submits known weight to calibrate
@app.route("/api/calibrate", methods=["POST"])
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
    save_config(cfg)
    pt_keys = ["board_p0x","board_p0y","board_p1x","board_p1y",
               "board_p2x","board_p2y","board_p3x","board_p3y","raw_capture","pixels_per_mm"]
    return {"status": "ok", "config": {k: cfg[k] for k in pt_keys if k in cfg}}


# Delete a measurement and its image
@app.route("/api/delete", methods=["POST"])
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
def index():
    measurements = load_measurements()
    cfg = load_config()
    return render_template_string(PAGE_TEMPLATE, measurements=measurements, config=cfg)


@app.route("/files/<filename>")
def serve_file(filename):
    return send_from_directory(UPLOAD_DIR, filename)


PAGE_TEMPLATE = """
<html>
<head>
    <title>ESP32 4D Scale</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { font-family: 'Segoe UI', monospace; background: #0f0f0f; color: #e0e0e0; padding: 24px; }
        h1 { color: #4fc3f7; margin-bottom: 4px; font-size: 22px; }
        .subtitle { color: #888; margin-bottom: 24px; font-size: 13px; }
        .panels { display: flex; flex-wrap: wrap; gap: 16px; margin-bottom: 24px; }
        .panel {
            background: #1a1a1a; border: 1px solid #333; border-radius: 10px;
            padding: 20px; flex: 1 1 320px; max-width: 480px;
        }
        .panel h2 { font-size: 15px; margin-bottom: 14px; }
        .panel h2.blue  { color: #4fc3f7; }
        .panel h2.amber { color: #ffb74d; }
        .panel h2.green { color: #81c784; }
        .panel h2.pink  { color: #f06292; }
        .grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(380px, 1fr)); gap: 16px; }
        .card {
            background: #1a1a1a; border: 1px solid #333; border-radius: 10px;
            overflow: hidden; transition: border-color 0.2s;
        }
        .card:hover { border-color: #4fc3f7; }
        .card img { width: 100%; display: block; image-rendering: pixelated; }
        .info { padding: 14px; }
        .time { color: #888; font-size: 11px; margin-bottom: 10px; }
        .dims {
            display: grid; grid-template-columns: 1fr 1fr; gap: 8px;
            margin-bottom: 10px;
        }
        .dim-box {
            background: #222; border-radius: 6px; padding: 10px;
            text-align: center;
        }
        .dim-label { color: #888; font-size: 10px; text-transform: uppercase; letter-spacing: 1px; }
        .dim-value { font-size: 20px; font-weight: bold; margin-top: 2px; }
        .dim-value.w { color: #4fc3f7; }
        .dim-value.h { color: #81c784; }
        .dim-value.wt { color: #ffb74d; }
        .dim-value.a { color: #ce93d8; }
        .dim-value.d { color: #e57373; }
        .dim-value.ht { color: #f06292; }
        .dim-sub { color: #666; font-size: 11px; }
        .no-detect { color: #ef5350; font-style: italic; padding: 10px; text-align: center; }
        .empty { color: #555; text-align: center; padding: 60px; font-size: 16px; }
        .row { display: flex; gap: 10px; align-items: center; margin-bottom: 10px; flex-wrap: wrap; }
        .row label { color: #aaa; font-size: 13px; min-width: 120px; }
        input[type=number], input[type=text] {
            background: #222; border: 1px solid #444; border-radius: 6px;
            color: #fff; padding: 8px 12px; font-size: 14px; width: 130px;
        }
        input[type=number]:focus { outline: none; border-color: #4fc3f7; }
        .hint { color: #666; font-size: 12px; margin-top: 6px; }
        .result { font-size: 13px; margin-top: 8px; display: none; }
        .current-val { color: #81c784; font-size: 13px; margin-bottom: 10px; }

        /* Buttons */
        .btn {
            border: none; border-radius: 6px;
            padding: 9px 22px; font-size: 14px; font-weight: bold; cursor: pointer;
        }
        .btn:disabled { background: #555 !important; color: #888 !important; cursor: not-allowed; }
        .btn-blue  { background: #4fc3f7; color: #000; }
        .btn-blue:hover  { background: #29b6f6; }
        .btn-amber { background: #ffb74d; color: #000; }
        .btn-amber:hover { background: #ffa726; }
        .btn-green { background: #81c784; color: #000; }
        .btn-green:hover { background: #66bb6a; }
        .btn-pink  { background: #f06292; color: #000; }
        .btn-pink:hover  { background: #ec407a; }
        .btn-red   { background: #c0392b; color: #fff; }
        .btn-red:hover   { background: #e74c3c; }

        .raw-info { color: #666; font-size: 11px; margin-top: 4px; }
        .board-grid {
            display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-bottom: 10px;
        }
        .board-cell { background: #222; border-radius: 6px; padding: 8px; text-align: center; }
        .board-cell .dim-label { color: #888; font-size: 10px; letter-spacing: 1px; margin-bottom: 4px; }
        .board-cell .inputs { display: flex; gap: 6px; justify-content: center; }
        .board-cell input { width: 58px !important; padding: 4px !important; text-align: center; }
    </style>
</head>
<body>
    <h1>ESP32 4D Scale</h1>
    <p class="subtitle">{{ measurements|length }} measurement{{ 's' if measurements|length != 1 }} captured</p>

    <div class="panels">

        <!-- MEASURE -->
        <div class="panel">
            <h2 class="blue">Trigger Measurement</h2>
            <p class="hint" style="margin-bottom:14px;">Triggers both ESP32s simultaneously — camera captures dimensions, HX711 measures weight.</p>
            <button class="btn btn-blue" id="btnMeasure" onclick="doTrigger('both')">&#9654; Measure</button>
            <div class="result" id="resultMeasure"></div>
        </div>

        <!-- BASELINE -->
        <div class="panel">
            <h2 class="green">Baseline Distance (Camera ESP32)</h2>
            <div class="current-val">Current: <strong id="currentBaseline">{{ config.get('baseline_cm', 0) }}</strong> cm
                {% if config.get('baseline_cm', 0) == 0 %}<span style="color:#888"> (auto at boot)</span>{% endif %}
            </div>
            <p class="hint" style="margin-bottom:10px;">Re-measure: place empty board and trigger camera ESP32 to read the ultrasonic baseline.</p>
            <button class="btn btn-green" id="btnBaseline" onclick="doTrigger('camera', {remeasure_baseline: true})" style="margin-bottom:12px;">
                &#8635; Re-measure Baseline
            </button>
            <div class="result" id="resultBaseline"></div>
            <hr style="border-color:#333;margin:12px 0;">
            <p class="hint" style="margin-bottom:8px;">Or enter manually:</p>
            <div class="row">
                <label>Distance (cm):</label>
                <input type="number" id="baselineInput" step="0.1" min="1" placeholder="e.g. 153.5">
                <button class="btn btn-green" onclick="doSetBaseline()">Save</button>
            </div>
            <div class="result" id="resultBaselineSave"></div>
        </div>

        <!-- WEIGHT CALIBRATION -->
        <div class="panel">
            <h2 class="amber">Weight Calibration (HX711 ESP32)</h2>
            <p class="hint" style="margin-bottom:10px;">Place a known weight on the board, trigger a tare+weigh, then enter the actual weight and calibrate.</p>
            <button class="btn btn-amber" id="btnWeightTrigger" onclick="doTrigger('weight', {tare_first: true})" style="margin-bottom:12px;">
                &#9654; Tare &amp; Weigh
            </button>
            <div class="result" id="resultWeightTrigger"></div>
            <hr style="border-color:#333;margin:12px 0;">
            <div class="row">
                <label>Known weight (g):</label>
                <input type="number" id="knownWeight" placeholder="e.g. 500" step="0.1" style="width:100px;">
                <button class="btn btn-amber" onclick="doWeightCalibrate()">Calibrate</button>
            </div>
            <div class="current-val" style="margin-top:10px;">
                cal_factor: <strong id="calFactorDisplay">{{ config.get('cal_factor', 27.93) }}</strong>
            </div>
            <div class="result" id="resultWeightCal"></div>
        </div>

        <!-- BOARD CALIBRATION -->
        <div class="panel">
            <h2 class="pink">Board Calibration (Camera ESP32)</h2>
            <p class="hint" style="margin-bottom:10px;">Capture raw image to identify board corners, then save the coordinates.</p>
            <div class="row" style="margin-bottom:12px;">
                <label style="cursor:pointer;color:#aaa;">
                    <input type="checkbox" id="rawCaptureCheck" {% if config.get('raw_capture') %}checked{% endif %} onchange="doSetRawCapture(this.checked)"> Raw capture mode
                </label>
                <button class="btn btn-pink" id="btnCapture" onclick="doCaptureRaw()">&#128247; Capture Raw</button>
            </div>
            <div class="result" id="resultCapture"></div>
            <div class="board-grid">
                <div class="board-cell">
                    <div class="dim-label">P0 — Top-Left</div>
                    <div class="inputs">
                        <input type="number" id="bp0x" value="{{ config.get('board_p0x', 50) }}">
                        <input type="number" id="bp0y" value="{{ config.get('board_p0y', 18) }}">
                    </div>
                </div>
                <div class="board-cell">
                    <div class="dim-label">P1 — Top-Right</div>
                    <div class="inputs">
                        <input type="number" id="bp1x" value="{{ config.get('board_p1x', 195) }}">
                        <input type="number" id="bp1y" value="{{ config.get('board_p1y', 18) }}">
                    </div>
                </div>
                <div class="board-cell">
                    <div class="dim-label">P3 — Bottom-Left</div>
                    <div class="inputs">
                        <input type="number" id="bp3x" value="{{ config.get('board_p3x', 50) }}">
                        <input type="number" id="bp3y" value="{{ config.get('board_p3y', 160) }}">
                    </div>
                </div>
                <div class="board-cell">
                    <div class="dim-label">P2 — Bottom-Right</div>
                    <div class="inputs">
                        <input type="number" id="bp2x" value="{{ config.get('board_p2x', 195) }}">
                        <input type="number" id="bp2y" value="{{ config.get('board_p2y', 160) }}">
                    </div>
                </div>
            </div>
            <div class="row">
                <label>Pixels/mm:</label>
                <input type="number" id="bppmm" value="{{ config.get('pixels_per_mm', 0.1644) }}" step="0.0001" style="width:90px;">
                <button class="btn btn-pink" onclick="doSaveCrop()">Save Corners</button>
            </div>
            <div class="hint" style="margin-top:6px;">Image is 320×240. x, y per corner.</div>
            <div class="result" id="resultCrop"></div>
        </div>

    </div><!-- end panels -->

    {% if not measurements %}
    <div class="empty">Waiting for ESP32 to upload results...</div>
    {% endif %}

    <div class="grid">
    {% for m in measurements %}
        <div class="card">
            {% if m.image %}<img src="/files/{{ m.image }}" alt="result">{% endif %}
            <div class="info">
                <div class="time">{{ m.timestamp }}</div>
                {% if m.valid %}
                <div class="dims">
                    <div class="dim-box">
                        <div class="dim-label">Length</div>
                        <div class="dim-value w">{{ "%.2f"|format(m.get('length_mm', 0) / 10) }} cm</div>
                        <div class="dim-sub">{{ "%.1f"|format(m.get('length_mm', 0)) }} mm</div>
                    </div>
                    <div class="dim-box">
                        <div class="dim-label">Width</div>
                        <div class="dim-value h">{{ "%.2f"|format(m.get('width_mm', 0) / 10) }} cm</div>
                        <div class="dim-sub">{{ "%.1f"|format(m.get('width_mm', 0)) }} mm</div>
                    </div>
                    <div class="dim-box">
                        <div class="dim-label">Height</div>
                        {% if m.get('height_cm', 0) > 0 %}
                        <div class="dim-value ht">{{ "%.2f"|format(m.get('height_cm', 0)) }} cm</div>
                        <div class="dim-sub">{{ "%.1f"|format(m.get('height_cm', 0) * 10) }} mm</div>
                        {% else %}
                        <div class="dim-value ht">N/A</div>
                        {% endif %}
                    </div>
                    <div class="dim-box">
                        <div class="dim-label">Weight</div>
                        <div class="dim-value wt">{{ "%.1f"|format(m.weight_g) }} g</div>
                        <div class="dim-sub">{{ "%.3f"|format(m.weight_g / 1000) }} kg</div>
                    </div>
                    <div class="dim-box">
                        <div class="dim-label">Angle</div>
                        <div class="dim-value a">{{ "%.1f"|format(m.angle) }}&deg;</div>
                        <div class="dim-sub">rotation</div>
                    </div>
                    <div class="dim-box">
                        <div class="dim-label">Ultrasonic</div>
                        {% if m.get('object_cm', -1) > 0 %}
                        <div class="dim-value d">{{ "%.2f"|format(m.get('object_cm', 0)) }} cm</div>
                        <div class="dim-sub">baseline: {{ "%.2f"|format(m.get('baseline_cm', 0)) }} cm</div>
                        {% else %}
                        <div class="dim-value d">N/A</div>
                        {% endif %}
                    </div>
                </div>
                <div class="raw-info">raw_tare: {{ m.raw_tare }} | raw_avg: {{ m.raw_avg }} | diff: {{ (m.raw_avg - m.raw_tare)|abs }}{% if m.get('pixels_per_mm', 0) > 0 %} | ppmm: {{ "%.4f"|format(m.pixels_per_mm) }}{% endif %}</div>
                {% if m.get('tare_readings') %}<div class="raw-info">tare: {{ m.tare_readings }}</div>{% endif %}
                {% if m.get('weight_readings') %}<div class="raw-info">weight: {{ m.weight_readings }}</div>{% endif %}
                {% else %}
                <div class="no-detect">No object detected</div>
                <div class="dims">
                    <div class="dim-box">
                        <div class="dim-label">Weight</div>
                        <div class="dim-value wt">{{ "%.1f"|format(m.weight_g) }} g</div>
                        <div class="dim-sub">{{ "%.3f"|format(m.weight_g / 1000) }} kg</div>
                    </div>
                    <div class="dim-box">
                        <div class="dim-label">Height</div>
                        {% if m.get('height_cm', 0) > 0 %}
                        <div class="dim-value ht">{{ "%.2f"|format(m.get('height_cm', 0)) }} cm</div>
                        <div class="dim-sub">{{ "%.1f"|format(m.get('height_cm', 0) * 10) }} mm</div>
                        {% else %}
                        <div class="dim-value ht">N/A</div>
                        {% endif %}
                    </div>
                    <div class="dim-box">
                        <div class="dim-label">Ultrasonic</div>
                        {% if m.get('object_cm', -1) > 0 %}
                        <div class="dim-value d">{{ "%.2f"|format(m.get('object_cm', 0)) }} cm</div>
                        <div class="dim-sub">baseline: {{ "%.2f"|format(m.get('baseline_cm', 0)) }} cm</div>
                        {% else %}
                        <div class="dim-value d">N/A</div>
                        {% endif %}
                    </div>
                </div>
                {% if m.get('tare_readings') %}<div class="raw-info">tare: {{ m.tare_readings }}</div>{% endif %}
                {% if m.get('weight_readings') %}<div class="raw-info">weight: {{ m.weight_readings }}</div>{% endif %}
                {% endif %}
                <div style="text-align:right;margin-top:6px;">
                    <button onclick="doDelete('{{ m.image }}')" class="btn btn-red" style="padding:4px 12px;font-size:12px;">Delete</button>
                </div>
            </div>
        </div>
    {% endfor %}
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
        btn.disabled = false; btn.textContent = '\uD83D\uDCF7 Capture Raw';
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
