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

1. Enable **Raw capture mode** in Board Calibration
2. Click **Capture** — uploads a raw unprocessed image
3. Inspect the image to find the board pixel boundaries
4. Enter the four corner coordinates (P0–P3)
5. Disable raw capture mode, click **Save All**

Use `server/find_coords.py` to interactively click corners on a saved image and read pixel coordinates.

### Pixels per mm

Sets the scale factor used to convert pixel distances to real-world mm. To calibrate:
1. Place a known-size object and take a measurement
2. Compare reported dimensions to actual: `new_ppmm = current_ppmm × (reported_mm / actual_mm)`
3. Enter the corrected value in the **Pixels/mm** field and click **Save All**

The scale adjusts dynamically per measurement based on the object's ultrasonic distance from the camera.

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
