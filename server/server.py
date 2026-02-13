from flask import Flask, request, send_from_directory, render_template_string
import os
import json
from datetime import datetime
from PIL import Image

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


# ESP32 sends images + measurements here
@app.route("/upload", methods=["POST"])
def upload():
    data = request.get_data()
    if len(data) == 0:
        return {"status": "error", "message": "no data"}, 400

    # Save image
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    pgm_name = f"result_{timestamp}.pgm"
    pgm_path = os.path.join(UPLOAD_DIR, pgm_name)
    with open(pgm_path, "wb") as f:
        f.write(data)

    # Convert to PNG
    img_filename = convert_pgm_to_png(pgm_path)
    print(f"Saved: {img_filename} ({len(data)} bytes)")

    # Parse measurements from query params
    entry = {
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
        "baseline_cm": float(request.args.get("baseline_cm", -1)),
        "object_cm": float(request.args.get("object_cm", -1)),
        "height_cm": float(request.args.get("height_cm", 0)),
        "pixels_per_mm": float(request.args.get("pixels_per_mm", 0)),
    }

    measurements = load_measurements()
    measurements.insert(0, entry)
    save_measurements(measurements)

    print(f"  -> L={entry['length_mm']:.1f} x W={entry['width_mm']:.1f} mm, H={entry['height_cm']:.2f} cm, "
          f"{entry['weight_g']:.1f} g, ppmm={entry['pixels_per_mm']:.4f}")
    return {"status": "ok", "filename": img_filename}, 200


# ESP32 fetches calibration factor on boot
@app.route("/api/config")
def get_config():
    cfg = load_config()
    return {"cal_factor": cfg["cal_factor"]}


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
        .cal-section {
            background: #1a1a1a; border: 1px solid #333; border-radius: 10px;
            padding: 20px; margin-bottom: 24px; max-width: 480px;
        }
        .cal-section h2 { color: #ffb74d; font-size: 16px; margin-bottom: 12px; }
        .cal-row { display: flex; gap: 10px; align-items: center; margin-bottom: 10px; }
        .cal-row label { color: #aaa; font-size: 13px; min-width: 110px; }
        .cal-row input {
            background: #222; border: 1px solid #444; border-radius: 6px;
            color: #fff; padding: 8px 12px; font-size: 14px; width: 140px;
        }
        .cal-row input:focus { outline: none; border-color: #ffb74d; }
        .cal-btn {
            background: #ffb74d; color: #000; border: none; border-radius: 6px;
            padding: 8px 20px; font-size: 14px; font-weight: bold; cursor: pointer;
        }
        .cal-btn:hover { background: #ffa726; }
        .cal-btn:disabled { background: #555; color: #888; cursor: not-allowed; }
        .cal-info { color: #888; font-size: 12px; margin-top: 8px; }
        .cal-result { color: #4fc3f7; font-size: 13px; margin-top: 8px; display: none; }
        .cal-current { color: #81c784; font-size: 13px; margin-bottom: 12px; }
        .raw-info { color: #666; font-size: 11px; margin-top: 4px; }
    </style>
</head>
<body>
    <h1>ESP32 4D Scale</h1>
    <p class="subtitle">{{ measurements|length }} measurement{{ 's' if measurements|length != 1 }} captured</p>

    <div class="cal-section">
        <h2>HX711 Calibration</h2>
        <div class="cal-current">Current cal_factor: <strong id="currentCal">{{ config.cal_factor }}</strong></div>
        <div class="cal-row">
            <label>Known weight (g):</label>
            <input type="number" id="knownWeight" step="0.1" min="0.1" placeholder="e.g. 1220">
            <button class="cal-btn" id="calBtn" onclick="doCalibrate()">Calibrate</button>
        </div>
        <div class="cal-info">Uses the latest measurement's raw tare &amp; weighted values to compute cal_factor.</div>
        <div class="cal-result" id="calResult"></div>
    </div>

    {% if not measurements %}
    <div class="empty">Waiting for ESP32 to upload results...</div>
    {% endif %}

    <div class="grid">
    {% for m in measurements %}
        <div class="card">
            <img src="/files/{{ m.image }}" alt="result">
            <div class="info">
                <div class="time">{{ m.timestamp }}</div>
                {% if m.valid %}
                <div class="dims">
                    <div class="dim-box">
                        <div class="dim-label">Length</div>
                        <div class="dim-value w">{{ "%.2f"|format(m.get('length_mm', m.get('width_mm', 0)) / 10) }} cm</div>
                        <div class="dim-sub">{{ "%.1f"|format(m.get('length_mm', m.get('width_mm', 0))) }} mm</div>
                    </div>
                    <div class="dim-box">
                        <div class="dim-label">Width</div>
                        <div class="dim-value h">{{ "%.2f"|format(m.get('width_mm', 0) / 10) }} cm</div>
                        <div class="dim-sub">{{ "%.1f"|format(m.get('width_mm', 0)) }} mm</div>
                    </div>
                    <div class="dim-box">
                        <div class="dim-label">Height</div>
                        {% if m.get('height_cm', m.get('object_height_cm', 0)) > 0 %}
                        <div class="dim-value ht">{{ "%.2f"|format(m.get('height_cm', m.get('object_height_cm', 0))) }} cm</div>
                        <div class="dim-sub">{{ "%.1f"|format(m.get('height_cm', m.get('object_height_cm', 0)) * 10) }} mm</div>
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
                {% else %}
                <div class="no-detect">No object detected</div>
                {% endif %}
            </div>
        </div>
    {% endfor %}
    </div>

    <script>
    async function doCalibrate() {
        const input = document.getElementById('knownWeight');
        const btn = document.getElementById('calBtn');
        const result = document.getElementById('calResult');
        const weight = parseFloat(input.value);
        if (!weight || weight <= 0) {
            result.style.display = 'block';
            result.style.color = '#ef5350';
            result.textContent = 'Enter a valid weight in grams.';
            return;
        }
        btn.disabled = true;
        btn.textContent = '...';
        try {
            const resp = await fetch('/api/calibrate', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({known_weight_g: weight})
            });
            const data = await resp.json();
            if (resp.ok) {
                result.style.display = 'block';
                result.style.color = '#4fc3f7';
                result.textContent = 'New cal_factor: ' + data.cal_factor + ' (raw_diff: ' + data.raw_diff + ')';
                document.getElementById('currentCal').textContent = data.cal_factor;
            } else {
                result.style.display = 'block';
                result.style.color = '#ef5350';
                result.textContent = 'Error: ' + data.message;
            }
        } catch (e) {
            result.style.display = 'block';
            result.style.color = '#ef5350';
            result.textContent = 'Request failed: ' + e.message;
        }
        btn.disabled = false;
        btn.textContent = 'Calibrate';
    }
    </script>
</body>
</html>
"""


if __name__ == "__main__":
    import socket
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    print(f"\n{'='*50}")
    print(f"  ESP32 4D Scale - Image Server")
    print(f"  Upload:  http://{local_ip}:8080/upload")
    print(f"  Viewer:  http://{local_ip}:8080/")
    print(f"  Saves to: {UPLOAD_DIR}")
    print(f"{'='*50}\n")
    app.run(host="0.0.0.0", port=8080)
