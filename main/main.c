#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include "esp_log.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_timer.h"

// ============== WIFI CONFIGURATION ==============
#define WIFI_SSID       "VithamasTech_Gnd Flr Hall_1"
#define WIFI_PASS       "#Newbusiness$"
#define SERVER_IP       "192.168.0.243"
#define SERVER_PORT     8080
#define WIFI_MAX_RETRY  10
#define HX711_CAL_FACTOR_DEFAULT 27.93f   // raw units per gram (fallback, calibrated with 3416g)
static float hx711_cal_factor = HX711_CAL_FACTOR_DEFAULT;

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;

// ============== ULTRASONIC SENSOR (HC-SR04) ==============
#define US_TRIG_PIN     13
#define US_ECHO_PIN     2
#define ROOM_TEMP_C     30.0f   // Room temperature in °C (affects speed of sound)
#define US_CAL_FACTOR   1.0f    // Distance correction: actual_cm / measured_cm

// ============== HX711 CONFIGURATION ==============
#define HX711_DOUT_PIN  14
#define HX711_SCK_PIN   15

static const char *TAG = "measure";

// ============== CONFIGURATION ==============
#define SOBEL_THRESHOLD     60      // Edge magnitude threshold (0-2040). Increase to ignore weaker edges like shadows
#define DILATION_ITERATIONS 1       // Close edge gaps after Sobel (increase if contour has breaks)
#define MAX_CONTOUR_POINTS  5000    // Max points in contour
#define MAX_CONTOUR_SEARCH  10      // Max contours to evaluate before picking the largest
#define MIN_BBOX_AREA_PCT   2       // Minimum contour bounding box as % of image area (rejects tiny noise)

// Hardcoded white board crop region (pixels) — measured from full 320x240 image at fixed 153.28cm
// Adjust these if the camera position changes
#define BOARD_X0  50
#define BOARD_Y0  18
#define BOARD_X1  195
#define BOARD_Y1  160

// Set to 1 to upload raw image without processing (for finding board coordinates)
// Set back to 0 for normal operation
#define RAW_CAPTURE_MODE  0
#define PIXELS_PER_MM_REF   0.1644f // Calibrated at 153.28 cm with 160x110mm and 210x140mm objects
#define CALIBRATION_DIST_CM 153.28f  // Reference distance (cm) for PIXELS_PER_MM_REF
#define FIXED_BASELINE_CM      153.28f // Fixed baseline distance (sensor to surface) in cm
// ===========================================

// AI-Thinker ESP32-CAM Pin Mapping
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_GPIO_NUM     4      // On-board flash LED

// ============== DATA STRUCTURES ==============

typedef struct {
    float x;
    float y;
} point_t;

typedef struct {
    point_t center;
    float width;        // pixels
    float height;       // pixels
    float angle;        // degrees
    point_t corners[4];
    float length_mm;
    float width_mm;
    bool valid;
} min_area_rect_t;

// ============== HX711 LOAD CELL ==============

static bool hx711_is_ready(void)
{
    return gpio_get_level(HX711_DOUT_PIN) == 0;
}

static int32_t hx711_read_raw(void)
{
    // Wait for HX711 to be ready (DOUT goes LOW)
    int timeout = 1000;
    while (!hx711_is_ready() && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
        timeout--;
    }

    if (timeout == 0) {
        ESP_LOGW(TAG, "HX711 timeout - not ready");
        return 0;
    }

    // Read 24 bits — disable interrupts to prevent bit-bang corruption
    int32_t data = 0;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&mux);
    for (int i = 0; i < 24; i++) {
        gpio_set_level(HX711_SCK_PIN, 1);
        ets_delay_us(1);
        data = (data << 1) | gpio_get_level(HX711_DOUT_PIN);
        gpio_set_level(HX711_SCK_PIN, 0);
        ets_delay_us(1);
    }

    // 25th pulse: set gain to 128 for channel A (default)
    gpio_set_level(HX711_SCK_PIN, 1);
    ets_delay_us(1);
    gpio_set_level(HX711_SCK_PIN, 0);
    ets_delay_us(1);
    taskEXIT_CRITICAL(&mux);

    // Convert from 24-bit two's complement
    if (data & 0x800000) {
        data |= 0xFF000000;  // Sign extend
    }

    return data;
}

static void hx711_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HX711_SCK_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << HX711_DOUT_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    gpio_set_level(HX711_SCK_PIN, 0);
    ESP_LOGI(TAG, "HX711 initialized (DOUT=%d, SCK=%d)", HX711_DOUT_PIN, HX711_SCK_PIN);

    // Wait for HX711 to stabilize and discard first readings
    vTaskDelay(pdMS_TO_TICKS(2000));
    for (int i = 0; i < 5; i++) {
        hx711_read_raw();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "HX711 warm-up done");
}

static int32_t hx711_tare = 0;

#define HX711_NUM_READINGS 5

static int cmp_int32(const void *a, const void *b)
{
    int32_t va = *(const int32_t *)a;
    int32_t vb = *(const int32_t *)b;
    return (va > vb) - (va < vb);
}

// Read N samples, discard zeros, sort, trim outer 25% (IQR), average the middle 50%
static int32_t hx711_read_filtered(const char *label)
{
    int32_t readings[HX711_NUM_READINGS];
    int valid = 0;

    for (int i = 0; i < HX711_NUM_READINGS; i++) {
        int32_t raw = hx711_read_raw();
        ESP_LOGI(TAG, "  %s[%d]: %ld", label, i + 1, (long)raw);
        if (raw != 0) {
            readings[valid++] = raw;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (valid == 0) return 0;

    qsort(readings, valid, sizeof(int32_t), cmp_int32);

    // Trim outer 25% on each side, average the middle 50%
    int trim = valid / 4;
    int start = trim;
    int end = valid - trim;
    if (end <= start) { start = 0; end = valid; }  // fallback if too few

    int64_t sum = 0;
    for (int i = start; i < end; i++) {
        sum += readings[i];
    }
    int32_t result = (int32_t)(sum / (end - start));

    ESP_LOGI(TAG, "  %s result: %ld (trimmed mean of [%d..%d] from %d valid, range: %ld to %ld)",
             label, (long)result, start, end - 1, valid,
             (long)readings[0], (long)readings[valid - 1]);
    return result;
}

static void hx711_read_tare(void)
{
    ESP_LOGI(TAG, ">>> Reading tare (empty plywood)...");
    hx711_tare = hx711_read_filtered("Tare");
}

static int32_t hx711_raw_avg = 0;  // stored for upload to server
static float g_baseline_cm = -1.0f;  // distance to empty surface
static float g_object_cm = -1.0f;    // distance to object top
static float g_object_height_cm = 0.0f;
static float g_dynamic_ppmm = PIXELS_PER_MM_REF;

static float hx711_read_grams(void)
{
    int32_t filtered = hx711_read_filtered("Weight");
    hx711_raw_avg = filtered;
    int32_t diff = filtered - hx711_tare;
    float grams = fabsf((float)diff) / hx711_cal_factor;
    if (grams < 2.0f) grams = 0;  // noise floor
    ESP_LOGI(TAG, "Weight: %.1f g (filtered: %ld, tare: %ld, diff: %ld, cal: %.4f)",
             grams, (long)filtered, (long)hx711_tare, (long)diff, hx711_cal_factor);
    return grams;
}

// ============== ULTRASONIC SENSOR ==============

static void ultrasonic_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << US_TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << US_ECHO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;  // Pull ECHO LOW when idle
    gpio_config(&io_conf);

    gpio_set_level(US_TRIG_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));  // Let sensor settle
    ESP_LOGI(TAG, "Ultrasonic sensor initialized (TRIG=GPIO%d, ECHO=GPIO%d)", US_TRIG_PIN, US_ECHO_PIN);
}

static portMUX_TYPE us_mux = portMUX_INITIALIZER_UNLOCKED;

static float ultrasonic_read_cm(void)
{
    // Wait for ECHO to be LOW before triggering (clear any leftover echo)
    int64_t wait_low = esp_timer_get_time() + 50000;  // 50ms max wait
    while (gpio_get_level(US_ECHO_PIN) == 1) {
        if (esp_timer_get_time() > wait_low) {
            ESP_LOGW(TAG, "  ECHO stuck HIGH before trigger — skipping");
            return -1.0f;
        }
    }

    portENTER_CRITICAL(&us_mux);

    // Send 10us trigger pulse
    gpio_set_level(US_TRIG_PIN, 0);
    ets_delay_us(10);
    gpio_set_level(US_TRIG_PIN, 1);
    ets_delay_us(20);
    gpio_set_level(US_TRIG_PIN, 0);

    // Wait for ECHO to go HIGH (timeout ~30ms)
    int64_t timeout = esp_timer_get_time() + 30000;
    while (gpio_get_level(US_ECHO_PIN) == 0) {
        if (esp_timer_get_time() > timeout) {
            portEXIT_CRITICAL(&us_mux);
            ESP_LOGW(TAG, "  ECHO never went HIGH — no response from sensor");
            return -1.0f;
        }
    }

    // Measure how long ECHO stays HIGH
    int64_t start = esp_timer_get_time();
    timeout = start + 30000;
    while (gpio_get_level(US_ECHO_PIN) == 1) {
        if (esp_timer_get_time() > timeout) {
            portEXIT_CRITICAL(&us_mux);
            ESP_LOGW(TAG, "  ECHO stayed HIGH too long — object too far or no object");
            return -1.0f;
        }
    }
    int64_t duration_us = esp_timer_get_time() - start;

    portEXIT_CRITICAL(&us_mux);
    // === END CRITICAL SECTION ===

    // Distance = (duration * speed_of_sound) / 2
    // Speed of sound = 331.3 + (0.606 * temp_C) m/s => cm/us
    float speed_cm_per_us = (331.3f + 0.606f * ROOM_TEMP_C) / 10000.0f;
    float distance_cm = (float)duration_us * speed_cm_per_us / 2.0f * US_CAL_FACTOR;
    return distance_cm;
}

// Average multiple readings, discard outliers
static float ultrasonic_measure(void)
{
    float readings[5];
    int valid = 0;

    for (int i = 0; i < 5; i++) {
        float d = ultrasonic_read_cm();
        if (d > 0.0f && d < 400.0f) {
            readings[valid++] = d;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (valid == 0) {
        ESP_LOGW(TAG, "Ultrasonic: no valid readings");
        return -1.0f;
    }

    float sum = 0;
    for (int i = 0; i < valid; i++) {
        sum += readings[i];
    }
    return sum / valid;
}

// ============== WIFI ==============

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retrying WiFi connection (%d/%d)...", s_retry_num, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "WiFi connection failed after %d retries", WIFI_MAX_RETRY);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_init(void)
{
    ESP_LOGI(TAG, "Initializing WiFi...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: 0x%x", ret);
        return false;
    }

    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler, NULL, &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "Connecting to '%s'...", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected!");
        return true;
    }
    ESP_LOGE(TAG, "WiFi connection failed!");
    return false;
}

// ============== FETCH CALIBRATION FROM SERVER ==============

// HTTP event handler: accumulates response body into a buffer
typedef struct {
    char *buf;
    int len;
    int capacity;
} http_response_t;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    http_response_t *resp = (http_response_t *)evt->user_data;
    if (evt->event_id == HTTP_EVENT_ON_DATA && resp) {
        int new_len = resp->len + evt->data_len;
        if (new_len < resp->capacity) {
            memcpy(resp->buf + resp->len, evt->data, evt->data_len);
            resp->len = new_len;
            resp->buf[resp->len] = '\0';
        }
    }
    return ESP_OK;
}

static void fetch_cal_factor(void)
{
    char url[128];
    snprintf(url, sizeof(url), "http://%s:%d/api/config", SERVER_IP, SERVER_PORT);

    char body[128] = {0};
    http_response_t resp = { .buf = body, .len = 0, .capacity = sizeof(body) - 1 };

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 5000,
        .event_handler = http_event_handler,
        .user_data = &resp,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK && esp_http_client_get_status_code(client) == 200) {
        // Simple JSON parse: find "cal_factor": <number>
        const char *key = "\"cal_factor\"";
        char *pos = strstr(body, key);
        if (pos) {
            pos += strlen(key);
            // Skip colon and whitespace
            while (*pos == ':' || *pos == ' ' || *pos == '\t') pos++;
            float val = strtof(pos, NULL);
            if (val > 0.0f) {
                hx711_cal_factor = val;
                ESP_LOGI(TAG, "Fetched cal_factor from server: %.4f", hx711_cal_factor);
            } else {
                ESP_LOGW(TAG, "Invalid cal_factor in response, using default");
            }
        } else {
            ESP_LOGW(TAG, "cal_factor not found in response: %s", body);
        }
    } else {
        ESP_LOGW(TAG, "Failed to fetch config (err=%s, status=%d), using default cal_factor=%.4f",
                 esp_err_to_name(err),
                 err == ESP_OK ? esp_http_client_get_status_code(client) : -1,
                 hx711_cal_factor);
    }

    esp_http_client_cleanup(client);
}

// ============== HTTP UPLOAD ==============

static bool upload_result(uint8_t *pgm_data, size_t pgm_size,
                          min_area_rect_t *rect, float weight_g)
{
    char url[512];
    snprintf(url, sizeof(url),
             "http://%s:%d/upload?valid=%d&length_mm=%.1f&width_mm=%.1f"
             "&length_px=%.1f&width_px=%.1f&angle=%.1f&weight_g=%.1f"
             "&raw_tare=%ld&raw_avg=%ld"
             "&baseline_cm=%.2f&object_cm=%.2f&height_cm=%.2f&pixels_per_mm=%.4f",
             SERVER_IP, SERVER_PORT,
             rect->valid ? 1 : 0,
             rect->valid ? rect->length_mm : 0.0f,
             rect->valid ? rect->width_mm : 0.0f,
             rect->valid ? rect->width : 0.0f,
             rect->valid ? rect->height : 0.0f,
             rect->valid ? rect->angle : 0.0f,
             weight_g,
             (long)hx711_tare, (long)hx711_raw_avg,
             g_baseline_cm, g_object_cm, g_object_height_cm, g_dynamic_ppmm);

    ESP_LOGI(TAG, "Upload params: baseline=%.2f cm, object=%.2f cm, object_height=%.2f cm, ppmm=%.4f",
             g_baseline_cm, g_object_cm, g_object_height_cm, g_dynamic_ppmm);
    ESP_LOGI(TAG, "Uploading to %s:%d (%zu bytes)...", SERVER_IP, SERVER_PORT, pgm_size);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
    esp_http_client_set_post_field(client, (const char *)pgm_data, pgm_size);

    esp_err_t err = esp_http_client_perform(client);
    bool success = false;

    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "Upload complete! HTTP %d", status);
        success = (status == 200);
    } else {
        ESP_LOGE(TAG, "Upload failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return success;
}

// ============== STEP 1: SOBEL EDGE DETECTION ==============

static void apply_sobel(uint8_t *src, uint8_t *dst, int width, int height, int edge_thresh)
{
    // Sobel kernels:
    // Gx: [-1, 0, 1]    Gy: [-1, -2, -1]
    //     [-2, 0, 2]        [ 0,  0,  0]
    //     [-1, 0, 1]        [ 1,  2,  1]

    // Border pixels: no edge
    for (int x = 0; x < width; x++) {
        dst[x] = 0;
        dst[(height - 1) * width + x] = 0;
    }
    for (int y = 0; y < height; y++) {
        dst[y * width] = 0;
        dst[y * width + width - 1] = 0;
    }

    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            int p00 = src[(y - 1) * width + (x - 1)];
            int p01 = src[(y - 1) * width +  x];
            int p02 = src[(y - 1) * width + (x + 1)];
            int p10 = src[ y      * width + (x - 1)];
            int p12 = src[ y      * width + (x + 1)];
            int p20 = src[(y + 1) * width + (x - 1)];
            int p21 = src[(y + 1) * width +  x];
            int p22 = src[(y + 1) * width + (x + 1)];

            int gx = -p00 + p02 - 2 * p10 + 2 * p12 - p20 + p22;
            int gy = -p00 - 2 * p01 - p02 + p20 + 2 * p21 + p22;

            // Manhattan magnitude (avoids sqrt, range 0-2040)
            int mag = abs(gx) + abs(gy);
            dst[y * width + x] = (mag > edge_thresh) ? 255 : 0;
        }
    }
}

// ============== STEP 1.5: DILATION (Close edge gaps) ==============
// Dilation expands edge pixels - closes small gaps so contour tracer can follow

static void apply_dilation(uint8_t *img, int width, int height, int iterations)
{
    uint8_t *temp = heap_caps_malloc(width * height, MALLOC_CAP_SPIRAM);
    if (!temp) return;

    for (int iter = 0; iter < iterations; iter++) {
        memcpy(temp, img, width * height);

        for (int y = 1; y < height - 1; y++) {
            for (int x = 1; x < width - 1; x++) {
                // 3x3 dilation: pixel is white if ANY neighbor is white
                int any_white = 0;
                for (int dy = -1; dy <= 1 && !any_white; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (temp[(y + dy) * width + (x + dx)] == 255) {
                            any_white = 1;
                            break;
                        }
                    }
                }
                img[y * width + x] = any_white ? 255 : 0;
            }
        }
    }

    heap_caps_free(temp);
}

// ============== STEP 2: CONTOUR TRACING (Moore Boundary) ==============

// 8-connected neighbor offsets (clockwise from right)
static const int dx8[] = {1, 1, 0, -1, -1, -1, 0, 1};
static const int dy8[] = {0, 1, 1, 1, 0, -1, -1, -1};

static int trace_contour(uint8_t *binary, int width, int height, point_t *contour, int max_points)
{
    int start_x = -1, start_y = -1;

    // Find first object pixel (top-left)
    for (int y = 0; y < height && start_x < 0; y++) {
        for (int x = 0; x < width; x++) {
            if (binary[y * width + x] == 255) {
                start_x = x;
                start_y = y;
                break;
            }
        }
    }

    if (start_x < 0) {
        ESP_LOGW(TAG, "No object found in image!");
        return 0;
    }

    ESP_LOGI(TAG, "Found object starting at (%d, %d)", start_x, start_y);

    int count = 0;
    int x = start_x, y = start_y;
    int dir = 0;  // Start direction

    do {
        if (count < max_points) {
            contour[count].x = (float)x;
            contour[count].y = (float)y;
            count++;
        }

        // Find next boundary pixel (Moore boundary tracing)
        int found = 0;
        int start_dir = (dir + 5) % 8;  // Start search from backtrack direction

        for (int i = 0; i < 8; i++) {
            int check_dir = (start_dir + i) % 8;
            int nx = x + dx8[check_dir];
            int ny = y + dy8[check_dir];

            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                if (binary[ny * width + nx] == 255) {
                    x = nx;
                    y = ny;
                    dir = check_dir;
                    found = 1;
                    break;
                }
            }
        }

        if (!found) break;

    } while ((x != start_x || y != start_y) && count < max_points);

    ESP_LOGI(TAG, "Contour traced: %d points", count);
    return count;
}

// ============== STEP 3: CONVEX HULL (Graham Scan) ==============

static float cross_product(point_t o, point_t a, point_t b)
{
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

static int compare_points(const void *a, const void *b)
{
    point_t *pa = (point_t *)a;
    point_t *pb = (point_t *)b;
    if (pa->x != pb->x) return (pa->x < pb->x) ? -1 : 1;
    return (pa->y < pb->y) ? -1 : 1;
}

static int convex_hull(point_t *points, int n, point_t *hull)
{
    if (n < 3) {
        for (int i = 0; i < n; i++) hull[i] = points[i];
        return n;
    }

    // Sort points
    qsort(points, n, sizeof(point_t), compare_points);

    int k = 0;

    // Build lower hull
    for (int i = 0; i < n; i++) {
        while (k >= 2 && cross_product(hull[k-2], hull[k-1], points[i]) <= 0) k--;
        hull[k++] = points[i];
    }

    // Build upper hull
    int lower_size = k;
    for (int i = n - 2; i >= 0; i--) {
        while (k > lower_size && cross_product(hull[k-2], hull[k-1], points[i]) <= 0) k--;
        hull[k++] = points[i];
    }

    return k - 1;  // Remove last point (same as first)
}

// ============== STEP 4: ROTATING CALIPERS → MIN AREA RECT ==============

static float dot_product(float ax, float ay, float bx, float by)
{
    return ax * bx + ay * by;
}

static min_area_rect_t find_min_area_rect(point_t *hull, int n, float pixels_per_mm)
{
    min_area_rect_t result = {0};
    result.valid = false;

    if (n < 3) return result;

    float min_area = 1e18f;

    // Try each edge of hull as base of rectangle
    for (int i = 0; i < n; i++) {
        point_t p1 = hull[i];
        point_t p2 = hull[(i + 1) % n];

        // Edge vector (normalized)
        float ex = p2.x - p1.x;
        float ey = p2.y - p1.y;
        float len = sqrtf(ex * ex + ey * ey);
        if (len < 0.001f) continue;
        ex /= len;
        ey /= len;

        // Perpendicular vector
        float px = -ey;
        float py = ex;

        // Project all points onto edge and perpendicular
        float min_e = 1e18f, max_e = -1e18f;
        float min_p = 1e18f, max_p = -1e18f;

        for (int j = 0; j < n; j++) {
            float dx = hull[j].x - p1.x;
            float dy = hull[j].y - p1.y;

            float proj_e = dot_product(dx, dy, ex, ey);
            float proj_p = dot_product(dx, dy, px, py);

            if (proj_e < min_e) min_e = proj_e;
            if (proj_e > max_e) max_e = proj_e;
            if (proj_p < min_p) min_p = proj_p;
            if (proj_p > max_p) max_p = proj_p;
        }

        float width = max_e - min_e;
        float height = max_p - min_p;
        float area = width * height;

        if (area < min_area) {
            min_area = area;

            result.width = width;
            result.height = height;
            result.angle = atan2f(ey, ex) * 180.0f / M_PI;

            // Calculate center
            float center_e = (min_e + max_e) / 2.0f;
            float center_p = (min_p + max_p) / 2.0f;
            result.center.x = p1.x + center_e * ex + center_p * px;
            result.center.y = p1.y + center_e * ey + center_p * py;

            // Calculate corners
            result.corners[0].x = p1.x + min_e * ex + min_p * px;
            result.corners[0].y = p1.y + min_e * ey + min_p * py;
            result.corners[1].x = p1.x + max_e * ex + min_p * px;
            result.corners[1].y = p1.y + max_e * ey + min_p * py;
            result.corners[2].x = p1.x + max_e * ex + max_p * px;
            result.corners[2].y = p1.y + max_e * ey + max_p * py;
            result.corners[3].x = p1.x + min_e * ex + max_p * px;
            result.corners[3].y = p1.y + min_e * ey + max_p * py;

            result.valid = true;
        }
    }

    // Ensure width >= height (swap if needed)
    if (result.width < result.height) {
        float tmp = result.width;
        result.width = result.height;
        result.height = tmp;
        result.angle += 90.0f;
    }

    // Normalize angle to [-90, 90]
    while (result.angle > 90.0f) result.angle -= 180.0f;
    while (result.angle < -90.0f) result.angle += 180.0f;

    // Convert to mm: width (longer side in px) = length, height (shorter side in px) = width
    result.length_mm = result.width / pixels_per_mm;
    result.width_mm = result.height / pixels_per_mm;

    return result;
}

// ============== VISUALIZATION: Draw rectangle on image ==============

static void draw_line(uint8_t *img, int width, int height, point_t p1, point_t p2, uint8_t color)
{
    int x0 = (int)p1.x, y0 = (int)p1.y;
    int x1 = (int)p2.x, y1 = (int)p2.y;

    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (1) {
        if (x0 >= 0 && x0 < width && y0 >= 0 && y0 < height) {
            img[y0 * width + x0] = color;
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void draw_min_area_rect(uint8_t *img, int width, int height, min_area_rect_t *rect)
{
    if (!rect->valid) return;

    for (int i = 0; i < 4; i++) {
        draw_line(img, width, height, rect->corners[i], rect->corners[(i+1)%4], 128);
    }
}


// ============== MAIN PROCESSING FUNCTION ==============

static min_area_rect_t process_image(uint8_t *grayscale, int width, int height, float pixels_per_mm)
{
    min_area_rect_t result = {0};

    // Step 0: Crop to hardcoded white board region
    int wb_x0 = BOARD_X0, wb_y0 = BOARD_Y0;
    int wb_x1 = BOARD_X1, wb_y1 = BOARD_Y1;
    ESP_LOGI(TAG, "Step 0: Board crop (%d,%d)-(%d,%d) [%dx%d]",
             wb_x0, wb_y0, wb_x1, wb_y1, wb_x1 - wb_x0, wb_y1 - wb_y0);

    // Crop the white board region into a new buffer
    int cw = wb_x1 - wb_x0;
    int ch = wb_y1 - wb_y0;
    uint8_t *crop = heap_caps_malloc(cw * ch, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *binary = heap_caps_malloc(cw * ch, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    point_t *contour = heap_caps_malloc(MAX_CONTOUR_POINTS * sizeof(point_t), MALLOC_CAP_SPIRAM);
    point_t *hull = heap_caps_malloc(MAX_CONTOUR_POINTS * sizeof(point_t), MALLOC_CAP_SPIRAM);

    if (!crop || !binary || !contour || !hull) {
        ESP_LOGE(TAG, "Failed to allocate processing buffers!");
        goto cleanup;
    }

    // Copy cropped region
    for (int y = 0; y < ch; y++) {
        memcpy(&crop[y * cw], &grayscale[(wb_y0 + y) * width + wb_x0], cw);
    }

    // Step 1: Sobel edge detection on cropped image
    ESP_LOGI(TAG, "Step 1: Sobel edge detection on crop %dx%d (threshold=%d)...", cw, ch, SOBEL_THRESHOLD);
    apply_sobel(crop, binary, cw, ch, SOBEL_THRESHOLD);

    // Step 1.5: Dilation to close edge gaps
    ESP_LOGI(TAG, "Step 1.5: Closing edge gaps (dilation=%d)...", DILATION_ITERATIONS);
    apply_dilation(binary, cw, ch, DILATION_ITERATIONS);

    // Step 2: Trace contours and pick the largest
    int min_bbox_area = (cw * ch * MIN_BBOX_AREA_PCT) / 100;
    int max_bbox_area = cw * ch * 80 / 100;  // Skip contours covering >80% of crop (board boundary)
    ESP_LOGI(TAG, "Step 2: Tracing contours (up to %d, min_area=%d)...", MAX_CONTOUR_SEARCH, min_bbox_area);
    int best_contour_count = 0;
    int best_area = 0;
    int best_attempt = -1;

    for (int attempt = 0; attempt < MAX_CONTOUR_SEARCH; attempt++) {
        int contour_count = trace_contour(binary, cw, ch, contour, MAX_CONTOUR_POINTS);
        if (contour_count < 3) {
            ESP_LOGI(TAG, "Contour search: attempt %d, %d points — stopping", attempt, contour_count);
            break;
        }

        // Compute axis-aligned bounding box area
        int min_x = contour[0].x, max_x = contour[0].x;
        int min_y = contour[0].y, max_y = contour[0].y;
        for (int i = 1; i < contour_count; i++) {
            if (contour[i].x < min_x) min_x = contour[i].x;
            if (contour[i].x > max_x) max_x = contour[i].x;
            if (contour[i].y < min_y) min_y = contour[i].y;
            if (contour[i].y > max_y) max_y = contour[i].y;
        }
        int bbox_area = (max_x - min_x) * (max_y - min_y);
        ESP_LOGI(TAG, "Contour search: attempt %d, %d points, bbox_area=%d%s", attempt, contour_count, bbox_area,
                 bbox_area < min_bbox_area ? " (too small, skipped)" : "");

        // Skip contours that are too small (noise) or too large (board boundary)
        if (bbox_area < min_bbox_area || bbox_area > max_bbox_area) {
            goto erase_contour;
        }

        // Keep the largest contour (stored temporarily in hull buffer)
        if (bbox_area > best_area) {
            best_area = bbox_area;
            best_contour_count = contour_count;
            best_attempt = attempt;
            memcpy(hull, contour, contour_count * sizeof(point_t));
        }

        erase_contour:
        for (int i = 0; i < contour_count; i++) {
            int cx = contour[i].x;
            int cy = contour[i].y;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int nx = cx + dx;
                    int ny = cy + dy;
                    if (nx >= 0 && nx < cw && ny >= 0 && ny < ch) {
                        binary[ny * cw + nx] = 0;
                    }
                }
            }
        }
    }

    if (best_contour_count < 3) {
        ESP_LOGW(TAG, "Not enough contour points!");
        goto cleanup;
    }

    // Copy best contour from hull back into contour, translate to full-image coordinates
    ESP_LOGI(TAG, "Best contour: %d points (attempt %d, bbox_area=%d)", best_contour_count, best_attempt, best_area);
    for (int i = 0; i < best_contour_count; i++) {
        contour[i].x = hull[i].x + wb_x0;
        contour[i].y = hull[i].y + wb_y0;
    }
    int contour_count = best_contour_count;

    // Step 3: Convex hull
    ESP_LOGI(TAG, "Step 3: Computing convex hull...");
    int hull_count = convex_hull(contour, contour_count, hull);
    ESP_LOGI(TAG, "Convex hull: %d points", hull_count);

    // Step 4: Min area rectangle
    ESP_LOGI(TAG, "Step 4: Finding minimum area rectangle...");
    result = find_min_area_rect(hull, hull_count, pixels_per_mm);

    // Mask grayscale outside white board for visualization
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (x < wb_x0 || x >= wb_x1 || y < wb_y0 || y >= wb_y1) {
                grayscale[y * width + x] = 0;
            }
        }
    }

    // Draw result on original image
    if (result.valid) {
        draw_min_area_rect(grayscale, width, height, &result);
    }

cleanup:
    if (crop) heap_caps_free(crop);
    if (binary) heap_caps_free(binary);
    if (contour) heap_caps_free(contour);
    if (hull) heap_caps_free(hull);

    return result;
}

// ============== HARDWARE INIT FUNCTIONS ==============

static void print_memory_info(void)
{
    ESP_LOGI(TAG, "--- Memory Information ---");
    size_t psram_size = esp_psram_get_size();
    if (psram_size > 0) {
        ESP_LOGI(TAG, "PSRAM: %.2f MB (Free: %zu bytes)",
                 (float)psram_size / (1024 * 1024),
                 heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    } else {
        ESP_LOGW(TAG, "PSRAM NOT AVAILABLE!");
    }
    ESP_LOGI(TAG, "Internal RAM Free: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

static esp_err_t init_camera(void)
{
    ESP_LOGI(TAG, "Initializing camera...");
    camera_config_t config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,
        .pin_d0 = Y2_GPIO_NUM,
        .pin_d1 = Y3_GPIO_NUM,
        .pin_d2 = Y4_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM,
        .pin_d4 = Y6_GPIO_NUM,
        .pin_d5 = Y7_GPIO_NUM,
        .pin_d6 = Y8_GPIO_NUM,
        .pin_d7 = Y9_GPIO_NUM,
        .pin_xclk = XCLK_GPIO_NUM,
        .pin_pclk = PCLK_GPIO_NUM,
        .pin_vsync = VSYNC_GPIO_NUM,
        .pin_href = HREF_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,
        .pin_pwdn = PWDN_GPIO_NUM,
        .pin_reset = RESET_GPIO_NUM,
        .xclk_freq_hz = 20000000,
        .pixel_format = PIXFORMAT_GRAYSCALE,
        .frame_size = FRAMESIZE_QVGA,    // 320x240
        .jpeg_quality = 12,
        .fb_count = 1,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init FAILED: 0x%x", err);
        return err;
    }
    ESP_LOGI(TAG, "Camera ready!");
    return ESP_OK;
}

// ============== MAIN ==============

void app_main(void)
{
    // Disable brownout detector (WiFi current spikes cause false triggers)
    CLEAR_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   ESP32-CAM Object Measurement        ║");
    ESP_LOGI(TAG, "║   Min Area Rectangle Detection        ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");

    print_memory_info();

    if (init_camera() != ESP_OK) {
        ESP_LOGE(TAG, "Camera required!");
        return;
    }

    // Let auto-exposure settle by discarding frames
    ESP_LOGI(TAG, "Camera warming up (auto-exposure)...");
    for (int i = 0; i < 15; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Step 1: Read tare on empty board (no WiFi = clean ADC readings)
    hx711_init();
    hx711_read_tare();

    // Step 2: Place object on the board and weigh
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, ">>> Place object on the board... (5 seconds)");
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, ">>> Reading weight...");
    float weight_g = hx711_read_grams();
    ESP_LOGI(TAG, ">>> Weight: %.1f g", weight_g);

    // Step 3: Measure object height (object is already on the board under camera)
    ultrasonic_init();
    g_baseline_cm = FIXED_BASELINE_CM;
    ESP_LOGI(TAG, ">>> Measuring object distance...");
    g_object_cm = ultrasonic_measure();
    ESP_LOGI(TAG, ">>> Ultrasonic raw reading: %.2f cm (baseline=%.2f cm)", g_object_cm, g_baseline_cm);
    if (g_object_cm > 0 && g_baseline_cm > 0) {
        g_object_height_cm = g_baseline_cm - g_object_cm;
        if (g_object_height_cm < 0) g_object_height_cm = 0;
        g_dynamic_ppmm = PIXELS_PER_MM_REF * (CALIBRATION_DIST_CM / g_object_cm);
        ESP_LOGI(TAG, ">>> Object distance: %.2f cm", g_object_cm);
        ESP_LOGI(TAG, ">>> Object height:   %.2f cm", g_object_height_cm);
        ESP_LOGI(TAG, ">>> Dynamic ppmm:    %.4f (ref %.4f at %.1f cm)",
                 g_dynamic_ppmm, PIXELS_PER_MM_REF, CALIBRATION_DIST_CM);
    } else {
        ESP_LOGW(TAG, ">>> Object distance: N/A — using reference ppmm");
        g_dynamic_ppmm = PIXELS_PER_MM_REF;
    }

    // Step 4: Capture image — flush stale frames first
    ESP_LOGI(TAG, ">>> Flushing stale camera frames...");
    for (int i = 0; i < 5; i++) {
        camera_fb_t *discard = esp_camera_fb_get();
        if (discard) esp_camera_fb_return(discard);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, ">>> Capturing image...");
    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ESP_LOGE(TAG, "Capture failed!");
        return;
    }

    ESP_LOGI(TAG, "Captured: %dx%d", fb->width, fb->height);

    // Copy image to our own buffer (camera buffer can have issues after modification)
    size_t img_size = fb->width * fb->height;
    uint8_t *img_copy = heap_caps_malloc(img_size, MALLOC_CAP_SPIRAM);
    if (!img_copy) {
        ESP_LOGE(TAG, "Failed to allocate image buffer!");
        esp_camera_fb_return(fb);
        return;
    }
    memcpy(img_copy, fb->buf, img_size);
    int img_width = fb->width;
    int img_height = fb->height;

    // Return camera buffer immediately
    esp_camera_fb_return(fb);

#if RAW_CAPTURE_MODE
    // RAW MODE: skip processing, upload image as-is for board coordinate calibration
    ESP_LOGI(TAG, ">>> RAW CAPTURE MODE — skipping processing, uploading raw image...");
    min_area_rect_t rect = {0};
#else
    // Process: find min area rectangle (this modifies img_copy)
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, ">>> Processing image...");
    min_area_rect_t rect = process_image(img_copy, img_width, img_height, g_dynamic_ppmm);
#endif

    // Print results
    ESP_LOGI(TAG, "");
    if (rect.valid) {
        ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
        ESP_LOGI(TAG, "║         MEASUREMENT RESULTS           ║");
        ESP_LOGI(TAG, "╠═══════════════════════════════════════╣");
        ESP_LOGI(TAG, "║  Center: (%.1f, %.1f)                 ", rect.center.x, rect.center.y);
        ESP_LOGI(TAG, "║  Angle:  %.1f°                        ", rect.angle);
        ESP_LOGI(TAG, "╠═══════════════════════════════════════╣");
        ESP_LOGI(TAG, "║  Size (pixels):                       ║");
        ESP_LOGI(TAG, "║    Length: %.1f px                    ", rect.width);
        ESP_LOGI(TAG, "║    Width:  %.1f px                    ", rect.height);
        ESP_LOGI(TAG, "╠═══════════════════════════════════════╣");
        ESP_LOGI(TAG, "║  Dimensions:                          ║");
        ESP_LOGI(TAG, "║    Length: %.1f mm                    ", rect.length_mm);
        ESP_LOGI(TAG, "║    Width:  %.1f mm                    ", rect.width_mm);
        if (g_object_cm > 0) {
            ESP_LOGI(TAG, "║    Height: %.2f cm (%.1f mm)          ", g_object_height_cm, g_object_height_cm * 10.0f);
        } else {
            ESP_LOGI(TAG, "║    Height: N/A                        ║");
        }
        ESP_LOGI(TAG, "╠═══════════════════════════════════════╣");
        ESP_LOGI(TAG, "║  Weight:                              ║");
        ESP_LOGI(TAG, "║    %.1f g  (%.3f kg)                  ", weight_g, weight_g / 1000.0f);
        ESP_LOGI(TAG, "╠═══════════════════════════════════════╣");
        ESP_LOGI(TAG, "║  Ultrasonic:                          ║");
        if (g_baseline_cm > 0) {
            ESP_LOGI(TAG, "║    Baseline:  %.2f cm                 ", g_baseline_cm);
        } else {
            ESP_LOGI(TAG, "║    Baseline:  N/A                     ║");
        }
        if (g_object_cm > 0) {
            ESP_LOGI(TAG, "║    Object:    %.2f cm                 ", g_object_cm);
        } else {
            ESP_LOGI(TAG, "║    Object:    N/A                     ║");
        }
        ESP_LOGI(TAG, "║    PPMM:      %.4f                   ", g_dynamic_ppmm);
        ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    } else {
        ESP_LOGW(TAG, "No object detected!");
    }

    // Connect WiFi, fetch cal_factor, and upload
    bool wifi_ok = wifi_init();
    if (wifi_ok) {
        fetch_cal_factor();
    }

    if (wifi_ok) {
        // Build PGM in memory (header + raw pixels)
        char pgm_header[32];
        int hdr_len = snprintf(pgm_header, sizeof(pgm_header), "P5\n%d %d\n255\n", img_width, img_height);
        size_t pgm_size = hdr_len + img_size;
        uint8_t *pgm_buf = heap_caps_malloc(pgm_size, MALLOC_CAP_SPIRAM);

        if (pgm_buf) {
            memcpy(pgm_buf, pgm_header, hdr_len);
            memcpy(pgm_buf + hdr_len, img_copy, img_size);

            upload_result(pgm_buf, pgm_size, &rect, weight_g);
            heap_caps_free(pgm_buf);
        } else {
            ESP_LOGE(TAG, "Failed to allocate upload buffer!");
        }
    }

    // Free image copy
    heap_caps_free(img_copy);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Done!");
}
