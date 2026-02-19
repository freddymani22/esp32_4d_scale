# ESP32 4D Scale

Object measurement system using an **AI-Thinker ESP32-CAM**. Places an object on a white board, then measures its **length**, **width**, **height**, and **weight** — all automatically.

## How It Works

1. **Tare** — reads the empty board weight via HX711 load cell
2. **Weigh** — place object on board, reads weight in grams
3. **Height** — HC-SR04 ultrasonic sensor measures distance to object top; height = baseline − object distance
4. **Capture** — camera takes a 320×240 grayscale image
5. **Crop** — image is cropped to the hardcoded white board region
6. **Sobel edge detection** — finds object edges within the crop
7. **Dilation** — closes small gaps in detected edges
8. **Contour tracing** — Moore boundary tracing extracts the object outline
9. **Convex hull** — Graham scan simplifies the contour
10. **Minimum area rectangle** — rotating calipers finds the tightest bounding box
11. **Upload** — processed image + measurements sent to Flask server over WiFi

## Hardware

| Component | Purpose |
|---|---|
| AI-Thinker ESP32-CAM | Camera + main controller |
| HX711 + load cell | Weight measurement |
| HC-SR04 ultrasonic | Object height measurement |
| White plywood board | Measurement surface / background |

### Wiring

| Signal | GPIO |
|---|---|
| HX711 DOUT | 14 |
| HX711 SCK | 15 |
| HC-SR04 TRIG | 13 |
| HC-SR04 ECHO | 2 |

## Configuration

All tunable parameters are at the top of `main/main.c`:

| Parameter | Default | Description |
|---|---|---|
| `SOBEL_THRESHOLD` | 60 | Edge magnitude cutoff (0–2040) |
| `DILATION_ITERATIONS` | 1 | Passes to close edge gaps |
| `MAX_CONTOUR_POINTS` | 5000 | Max contour points |
| `MIN_BBOX_AREA_PCT` | 2 | Min object bbox as % of crop area (rejects noise) |
| `PIXELS_PER_MM_REF` | 0.2225 | Pixels per mm at reference distance |
| `CALIBRATION_DIST_CM` | 153.28 | Camera-to-board distance (cm) |
| `FIXED_BASELINE_CM` | 153.28 | Fixed baseline for height calculation |
| `BOARD_X0/Y0/X1/Y1` | 50,18,195,160 | Hardcoded board crop region (pixels) |
| `RAW_CAPTURE_MODE` | 0 | Set to 1 to upload raw image for recalibration |

### WiFi & Server

```c
#define WIFI_SSID   "your_ssid"
#define WIFI_PASS   "your_password"
#define SERVER_IP   "192.168.x.x"
#define SERVER_PORT 8080
```

## Server

Flask server in `server/server.py` receives images and measurements.

```bash
cd server
python -m venv venv
source venv/bin/activate
pip install flask pillow
python server.py
```

Results are viewable in the web UI at `http://SERVER_IP:8080`.

## Build & Flash

Requires **ESP-IDF v5.5.x**.

```bash
source ~/esp/v5.5.2/esp-idf/export.sh
idf.py build
idf.py flash monitor
```

## Calibration

### Board crop region
Set `RAW_CAPTURE_MODE 1`, flash, capture an image. Inspect the uploaded raw image to find the white board pixel boundaries. Update `BOARD_X0/Y0/X1/Y1`, then set `RAW_CAPTURE_MODE 0`.

### Pixels per mm
Place a known-size object on the board, run a measurement, compare reported dimensions to actual. Scale `PIXELS_PER_MM_REF` by `measured_mm / actual_mm`.

### Weight (cal_factor)
Use the web UI — enter the known weight of an object after measuring it, click **Calibrate**. The server stores the new `cal_factor` in `config.json` and the ESP32 fetches it on next boot.
