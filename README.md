# ESP32 4D Scale

Object measurement system using an **AI-Thinker ESP32-CAM** module. It captures a grayscale image, detects the object, and computes its dimensions (width and height in mm) using image processing — all running on-device.

## How It Works

The processing pipeline runs entirely on the ESP32:

1. **Sobel edge detection** — computes gradient magnitude to find sharp object edges (ignores soft shadow edges)
2. **Dilation** — closes small gaps in the detected edges so the contour is connected
3. **Contour tracing** — Moore boundary tracing to extract the object outline
4. **Convex hull** — Graham scan algorithm to simplify the contour
5. **Minimum area rectangle** — rotating calipers to find the tightest bounding box
6. **Measurement** — converts pixel dimensions to mm using a calibration factor (`PIXELS_PER_MM`)

Both the original and the processed image (with the bounding rectangle drawn) are saved as `.pgm` files on the SD card.

## Hardware

- AI-Thinker ESP32-CAM (ESP32 + OV2640 camera + PSRAM)
- SD card (required for saving images)

## Configuration

All tunable parameters are at the top of `main/main.c`:

| Parameter | Default | Description |
|---|---|---|
| `SOBEL_THRESHOLD` | 150 | Edge magnitude cutoff (0-2040). Increase to ignore weaker edges like shadows |
| `DILATION_ITERATIONS` | 2 | Dilation passes to close gaps in detected edges |
| `MAX_CONTOUR_POINTS` | 5000 | Maximum points stored during contour tracing |
| `PIXELS_PER_MM` | 0.99 | Calibration factor for 35 cm camera distance |
| `RESET_PHOTO_NUMBER` | true | If true, photo numbering resets to 1 each run |

### Calibration

`PIXELS_PER_MM` must be calibrated for your setup. Place an object with a known size in the camera's field of view, capture an image, and adjust the value until the reported dimensions match reality.

## Build & Flash

Requires **ESP-IDF** (v4.1.0 or later).

```bash
idf.py set-target esp32
idf.py build
idf.py flash monitor
```

## Output

Images are saved to `/sdcard/photos/` on the SD card:

- `imgN.pgm` — original captured frame
- `resN.pgm` — processed frame with the minimum area rectangle drawn

Measurement results (dimensions in pixels and mm, center, angle, corners) are printed to the serial monitor.
