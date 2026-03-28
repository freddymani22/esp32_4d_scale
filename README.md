# 4D Scale

Automated object measurement system that captures **length**, **width**, **height**, and **weight** simultaneously. Two ESP32 devices work together — one handles imaging, one handles weight — coordinated by a central Flask server.

## How It Works

1. **Trigger** — web dashboard sends a trigger to both devices
2. **Tare** — weight device reads empty board tare (optionally)
3. **Weigh** — weight device reads object weight in grams
4. **Height** — camera device measures distance to object top via ultrasonic; height = baseline − object distance
5. **Capture** — camera takes a 320×240 grayscale image
6. **Crop** — image cropped to the configured board region (set via web UI)
7. **Sobel edge detection** — finds object edges within the crop
8. **Dilation** — closes small gaps in detected edges
9. **Contour tracing** — Moore boundary tracing extracts the object outline
10. **Convex hull** — Graham scan simplifies the contour
11. **Minimum area rectangle** — rotating calipers finds the tightest bounding box
12. **Upload** — both devices upload results; server merges into one measurement record

## Hardware

Two ESP32 devices are required, each flashed with a different `DEVICE_MODE`.

| Device | `DEVICE_MODE` | Components |
|---|---|---|
| Camera unit | `1` | AI-Thinker ESP32-CAM, HC-SR04 ultrasonic |
| Weight unit | `0` | ESP32, HX711 + load cell |

### Wiring — Weight unit (DEVICE_MODE 0)

| Signal | GPIO |
|---|---|
| HX711 DOUT | 14 |
| HX711 SCK | 15 |

### Wiring — Camera unit (DEVICE_MODE 1)

| Signal | GPIO |
|---|---|
| HC-SR04 TRIG | 13 |
| HC-SR04 ECHO | 2 |

## Build & Flash

Requires **ESP-IDF v5.5.x**.

```bash
source ~/esp/v5.5.2/esp-idf/export.sh
```

Set `DEVICE_MODE` at the top of `main/main.c` before building:

```c
// 0 = Weight unit (HX711)
// 1 = Camera unit (ESP32-CAM)
#define DEVICE_MODE 0
```

Set WiFi and server address:

```c
#define WIFI_SSID   "your_ssid"
#define WIFI_PASS   "your_password"
#define SERVER_IP   "your.server.address"
#define SERVER_PORT 8080
```

Build and flash:

```bash
idf.py build
idf.py flash monitor
```

Flash each device separately with the correct `DEVICE_MODE`.

## Server

Flask server in `server/server.py` — receives uploads, serves the web dashboard, persists measurements and config.

```bash
cd server
python -m venv venv
source venv/bin/activate
pip install flask pillow
python server.py
```

Dashboard available at `http://SERVER_IP:8080`. Login required (default: `admin` / `scale1234` — change in `server.py`).

## Calibration

All calibration is done through the web dashboard — no reflashing required. Open the **Calibration Settings** accordion.

### Baseline distance

The camera-to-empty-board distance used as the height reference. Either:
- Click **Re-measure** (camera device measures it automatically), or
- Enter the value manually and click **Save**

### Board corners (crop region)

The system needs to know exactly where the measurement board is in the camera's field of view. This is done once using `find_coords.py` — a GUI tool that lets you click the corners directly on the image.

**Step 1 — Capture a raw image**

1. Open the dashboard → Calibration Settings → Board Calibration
2. Enable **Raw capture mode**
3. Click **Capture** — this uploads a raw unprocessed image to `server/uploads/`

**Step 2 — Find the corner coordinates**

Run on the machine that has the `server/uploads/` folder:

```bash
cd server
./venv/bin/python3 find_coords.py
```

This opens a 3× zoomed view of the latest uploaded image. Click the four corners of the plywood board **in order: Top-Left → Top-Right → Bottom-Right → Bottom-Left**. Each click shows the pixel coordinates and draws the outline. When all four are clicked, the terminal prints:

```
  P0 (TL): x=50, y=18
  P1 (TR): x=195, y=18
  P2 (BR): x=195, y=160
  P3 (BL): x=50, y=160
```

**Step 3 — Enter values in the dashboard**

Enter the printed P0–P3 values into the Board Calibration fields, disable **Raw capture mode**, and click **Save All**.

### Pixels per mm

Sets the scale factor that converts pixel distances to real-world millimetres. Two ways to calibrate:

**Option A — Using `find_coords.py` (most accurate)**

```bash
cd server
./venv/bin/python3 find_coords.py --ppmm
```

This opens the image cropped to your board region. Click the **top-left** then **bottom-right** corner of a known-size object on the board. Enter the actual dimensions when prompted and the script calculates and prints the correct `ppmm`:

```
  ppmm from width:  0.2181
  ppmm from height: 0.2194
  Average ppmm:     0.2188  ← use this
```

Enter this value in the **Pixels/mm** field and click **Save All**.

**Option B — From a measurement**

1. Place a known-size object, take a measurement
2. Calculate: `new_ppmm = current_ppmm × (actual_mm / reported_mm)`
3. Enter corrected value → **Save All**

The ppmm scales automatically per measurement using the ultrasonic sensor distance, so calibrate at your typical object height.

### Weight (cal_factor)

1. Place empty board → enable **Tare first** → click **Weigh**
2. Place a known-weight object (no tare) → click **Weigh**
3. Enter the actual weight in grams → click **Calibrate**

The new `cal_factor` is saved to `config.json` and fetched by the weight device on next boot.

## Data

- Measurements stored in `server/uploads/measurements.json`
- Config (cal_factor, corners, ppmm, etc.) in `server/uploads/config.json`
- Uploaded images in `server/uploads/`
- Delete individual records or date ranges from the dashboard
