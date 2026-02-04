#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"

#define PHOTO_FOLDER "/sdcard/photos"

static const char *TAG = "measure";

// ============== CONFIGURATION ==============
// Chromaticity threshold: how different a pixel's color ratio must be from background
// Range 0.0-1.0. Lower = more sensitive (detects subtle color differences)
// Typical values: 0.05-0.15
#define CHROMA_THRESHOLD    0.08f

#define DILATION_ITERATIONS 2       // Close gaps in detected object mask
#define EROSION_ITERATIONS  1       // Remove noise specks
#define MAX_CONTOUR_POINTS  5000    // Max points in contour
#define PIXELS_PER_MM       1.38f   // Calibrate with known object!
#define RESET_PHOTO_NUMBER  false   // Set to true to always start from 1 (overwrites existing)

// Corner sample size for background detection (samples from all 4 corners)
#define CORNER_SAMPLE_SIZE  10
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
    float width_mm;
    float height_mm;
    bool valid;
} min_area_rect_t;

// ============== RGB565 HELPERS ==============

// RGB565 format: RRRRRGGG GGGBBBBB (big-endian in memory on ESP32-CAM)
static inline void rgb565_to_rgb(uint16_t pixel, uint8_t *r, uint8_t *g, uint8_t *b)
{
    // ESP32-CAM stores RGB565 in big-endian, so swap bytes
    pixel = (pixel >> 8) | (pixel << 8);
    *r = (pixel >> 11) & 0x1F;
    *g = (pixel >> 5) & 0x3F;
    *b = pixel & 0x1F;
    // Scale to 0-255
    *r = (*r << 3) | (*r >> 2);
    *g = (*g << 2) | (*g >> 4);
    *b = (*b << 3) | (*b >> 2);
}

static inline uint16_t rgb_to_rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t pixel = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
    // Swap back to big-endian
    return (pixel >> 8) | (pixel << 8);
}

// ============== STEP 1: COLOR-BASED OBJECT DETECTION ==============
// Uses chromaticity (color ratios) which is invariant to shadows.
// Shadows scale R, G, B proportionally, so r/(r+g+b) stays constant.
// Objects have different chromaticity than background.

static void detect_object_by_color(uint16_t *rgb565, uint8_t *binary, int width, int height, float chroma_thresh)
{
    // Sample background chromaticity from corners
    float bg_cr = 0, bg_cg = 0;  // background chromaticity (r and g channels, b is implicit)
    int sample_count = 0;

    // Sample from all 4 corners
    int corners[4][2] = {
        {0, 0},                           // top-left
        {width - CORNER_SAMPLE_SIZE, 0},  // top-right
        {0, height - CORNER_SAMPLE_SIZE}, // bottom-left
        {width - CORNER_SAMPLE_SIZE, height - CORNER_SAMPLE_SIZE}  // bottom-right
    };

    for (int c = 0; c < 4; c++) {
        int sx = corners[c][0];
        int sy = corners[c][1];
        for (int y = sy; y < sy + CORNER_SAMPLE_SIZE && y < height; y++) {
            for (int x = sx; x < sx + CORNER_SAMPLE_SIZE && x < width; x++) {
                uint8_t r, g, b;
                rgb565_to_rgb(rgb565[y * width + x], &r, &g, &b);
                int sum = r + g + b;
                if (sum > 30) {  // Skip very dark pixels
                    bg_cr += (float)r / sum;
                    bg_cg += (float)g / sum;
                    sample_count++;
                }
            }
        }
    }

    if (sample_count > 0) {
        bg_cr /= sample_count;
        bg_cg /= sample_count;
    } else {
        // Fallback to neutral gray
        bg_cr = 0.33f;
        bg_cg = 0.33f;
    }

    ESP_LOGI(TAG, "Background chromaticity: r=%.3f, g=%.3f (from %d samples)",
             bg_cr, bg_cg, sample_count);

    // Classify each pixel
    int object_pixels = 0;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            uint8_t r, g, b;
            rgb565_to_rgb(rgb565[idx], &r, &g, &b);

            int sum = r + g + b;
            if (sum < 30) {
                // Very dark pixel - likely object (or shadow, but we err on side of detection)
                binary[idx] = 255;
                object_pixels++;
            } else {
                float cr = (float)r / sum;
                float cg = (float)g / sum;

                // Euclidean distance in chromaticity space
                float dist = sqrtf((cr - bg_cr) * (cr - bg_cr) + (cg - bg_cg) * (cg - bg_cg));

                if (dist > chroma_thresh) {
                    binary[idx] = 255;  // Object
                    object_pixels++;
                } else {
                    binary[idx] = 0;    // Background or shadow
                }
            }
        }
    }

    ESP_LOGI(TAG, "Detected %d object pixels (%.1f%% of image)",
             object_pixels, 100.0f * object_pixels / (width * height));
}

// ============== STEP 1.5: MORPHOLOGICAL OPERATIONS ==============

static void apply_erosion(uint8_t *img, int width, int height, int iterations)
{
    uint8_t *temp = heap_caps_malloc(width * height, MALLOC_CAP_SPIRAM);
    if (!temp) return;

    for (int iter = 0; iter < iterations; iter++) {
        memcpy(temp, img, width * height);
        for (int y = 1; y < height - 1; y++) {
            for (int x = 1; x < width - 1; x++) {
                // 3x3 erosion: pixel is white only if ALL neighbors are white
                int all_white = 1;
                for (int dy = -1; dy <= 1 && all_white; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (temp[(y + dy) * width + (x + dx)] == 0) {
                            all_white = 0;
                            break;
                        }
                    }
                }
                img[y * width + x] = all_white ? 255 : 0;
            }
        }
    }
    heap_caps_free(temp);
}

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
    int dir = 0;

    do {
        if (count < max_points) {
            contour[count].x = (float)x;
            contour[count].y = (float)y;
            count++;
        }

        int found = 0;
        int start_dir = (dir + 5) % 8;

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

    return k - 1;
}

// ============== STEP 4: ROTATING CALIPERS → MIN AREA RECT ==============

static float dot_product(float ax, float ay, float bx, float by)
{
    return ax * bx + ay * by;
}

static min_area_rect_t find_min_area_rect(point_t *hull, int n)
{
    min_area_rect_t result = {0};
    result.valid = false;

    if (n < 3) return result;

    float min_area = 1e18f;

    for (int i = 0; i < n; i++) {
        point_t p1 = hull[i];
        point_t p2 = hull[(i + 1) % n];

        float ex = p2.x - p1.x;
        float ey = p2.y - p1.y;
        float len = sqrtf(ex * ex + ey * ey);
        if (len < 0.001f) continue;
        ex /= len;
        ey /= len;

        float px = -ey;
        float py = ex;

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

            float center_e = (min_e + max_e) / 2.0f;
            float center_p = (min_p + max_p) / 2.0f;
            result.center.x = p1.x + center_e * ex + center_p * px;
            result.center.y = p1.y + center_e * ey + center_p * py;

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

    if (result.width < result.height) {
        float tmp = result.width;
        result.width = result.height;
        result.height = tmp;
        result.angle += 90.0f;
    }

    while (result.angle > 90.0f) result.angle -= 180.0f;
    while (result.angle < -90.0f) result.angle += 180.0f;

    result.width_mm = result.width / PIXELS_PER_MM;
    result.height_mm = result.height / PIXELS_PER_MM;

    return result;
}

// ============== VISUALIZATION: Draw rectangle on RGB565 image ==============

static void draw_line_rgb565(uint16_t *img, int width, int height, point_t p1, point_t p2, uint16_t color)
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

static void draw_min_area_rect_rgb565(uint16_t *img, int width, int height, min_area_rect_t *rect)
{
    if (!rect->valid) return;

    // Bright green color for visibility
    uint16_t green = rgb_to_rgb565(0, 255, 0);

    for (int i = 0; i < 4; i++) {
        draw_line_rgb565(img, width, height, rect->corners[i], rect->corners[(i+1)%4], green);
    }
}

// ============== MAIN PROCESSING FUNCTION ==============

static min_area_rect_t process_image(uint16_t *rgb565, int width, int height)
{
    min_area_rect_t result = {0};

    // Allocate buffers in PSRAM
    uint8_t *binary = heap_caps_malloc(width * height, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    point_t *contour = heap_caps_malloc(MAX_CONTOUR_POINTS * sizeof(point_t), MALLOC_CAP_SPIRAM);
    point_t *hull = heap_caps_malloc(MAX_CONTOUR_POINTS * sizeof(point_t), MALLOC_CAP_SPIRAM);

    if (!binary || !contour || !hull) {
        ESP_LOGE(TAG, "Failed to allocate processing buffers!");
        goto cleanup;
    }

    // Step 1: Color-based object detection (shadow-invariant)
    ESP_LOGI(TAG, "Step 1: Color-based detection (chroma_thresh=%.3f)...", CHROMA_THRESHOLD);
    detect_object_by_color(rgb565, binary, width, height, CHROMA_THRESHOLD);

    // Step 1.5: Morphological cleanup (erosion removes noise, dilation fills gaps)
    ESP_LOGI(TAG, "Step 1.5: Morphological cleanup (erosion=%d, dilation=%d)...",
             EROSION_ITERATIONS, DILATION_ITERATIONS);
    apply_erosion(binary, width, height, EROSION_ITERATIONS);
    apply_dilation(binary, width, height, DILATION_ITERATIONS);

    // Step 2: Trace contour
    ESP_LOGI(TAG, "Step 2: Tracing contour...");
    int contour_count = trace_contour(binary, width, height, contour, MAX_CONTOUR_POINTS);
    if (contour_count < 3) {
        ESP_LOGW(TAG, "Not enough contour points!");
        goto cleanup;
    }

    // Step 3: Convex hull
    ESP_LOGI(TAG, "Step 3: Computing convex hull...");
    int hull_count = convex_hull(contour, contour_count, hull);
    ESP_LOGI(TAG, "Convex hull: %d points", hull_count);

    // Step 4: Min area rectangle
    ESP_LOGI(TAG, "Step 4: Finding minimum area rectangle...");
    result = find_min_area_rect(hull, hull_count);

    // Draw result on original color image
    if (result.valid) {
        draw_min_area_rect_rgb565(rgb565, width, height, &result);
    }

cleanup:
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

static esp_err_t init_sd_card(void)
{
    ESP_LOGI(TAG, "Initializing SD card...");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 20,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD card mount failed! Error: 0x%x", ret);
        return ret;
    }
    ESP_LOGI(TAG, "SD card mounted!");
    return ESP_OK;
}

static esp_err_t init_camera(void)
{
    ESP_LOGI(TAG, "Initializing camera (RGB565 mode)...");
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
        .pixel_format = PIXFORMAT_RGB565,   // COLOR capture
        .frame_size = FRAMESIZE_QVGA,       // 320x240
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
    ESP_LOGI(TAG, "Camera ready (RGB565)!");
    return ESP_OK;
}

static void create_photo_folder(void)
{
    struct stat st;
    if (stat(PHOTO_FOLDER, &st) != 0) {
        mkdir(PHOTO_FOLDER, 0775);
    }
}

static int get_next_photo_number(void)
{
#if RESET_PHOTO_NUMBER
    return 1;
#else
    char filename[64];
    struct stat st;
    int num = 1;
    while (num < 10000) {
        snprintf(filename, sizeof(filename), PHOTO_FOLDER "/img%d.ppm", num);
        if (stat(filename, &st) != 0) break;
        num++;
    }
    return num;
#endif
}

// Save RGB565 image as PPM (portable pixmap - color)
static void save_ppm(uint16_t *rgb565, int width, int height, const char *filename)
{
    FILE *f = fopen(filename, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to save: %s (errno=%d: %s)", filename, errno, strerror(errno));
        return;
    }

    // PPM header
    fprintf(f, "P6\n%d %d\n255\n", width, height);

    // Convert and write pixel by pixel (to avoid large temporary buffer)
    for (int i = 0; i < width * height; i++) {
        uint8_t r, g, b;
        rgb565_to_rgb(rgb565[i], &r, &g, &b);
        fputc(r, f);
        fputc(g, f);
        fputc(b, f);
    }

    fflush(f);
    fsync(fileno(f));
    fclose(f);
    ESP_LOGI(TAG, "Saved: %s", filename);
}

// ============== MAIN ==============

void app_main(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   ESP32-CAM Object Measurement        ║");
    ESP_LOGI(TAG, "║   COLOR-BASED Shadow Rejection        ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");

    print_memory_info();

    if (init_sd_card() != ESP_OK) {
        ESP_LOGE(TAG, "SD card required!");
        return;
    }
    create_photo_folder();

    if (init_camera() != ESP_OK) {
        ESP_LOGE(TAG, "Camera required!");
        return;
    }

    ESP_LOGI(TAG, "Waiting for camera to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    int photo_num = get_next_photo_number();

    // Capture
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, ">>> Capturing image (RGB565)...");

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Capture failed!");
        return;
    }

    ESP_LOGI(TAG, "Captured: %dx%d (%zu bytes)", fb->width, fb->height, fb->len);

    // Copy image (RGB565 = 2 bytes per pixel)
    size_t img_size = fb->width * fb->height * 2;
    uint16_t *img_copy = heap_caps_malloc(img_size, MALLOC_CAP_SPIRAM);
    if (!img_copy) {
        ESP_LOGE(TAG, "Failed to allocate image buffer!");
        esp_camera_fb_return(fb);
        return;
    }
    memcpy(img_copy, fb->buf, img_size);
    int img_width = fb->width;
    int img_height = fb->height;

    esp_camera_fb_return(fb);

    // Save original
    char filename[64];
    snprintf(filename, sizeof(filename), PHOTO_FOLDER "/img%d.ppm", photo_num);
    save_ppm(img_copy, img_width, img_height, filename);

    // Process
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, ">>> Processing image...");
    min_area_rect_t rect = process_image(img_copy, img_width, img_height);

    // Save processed (with rectangle drawn)
    snprintf(filename, sizeof(filename), PHOTO_FOLDER "/res%d.ppm", photo_num);
    save_ppm(img_copy, img_width, img_height, filename);

    heap_caps_free(img_copy);

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
        ESP_LOGI(TAG, "║    Width:  %.1f px                    ", rect.width);
        ESP_LOGI(TAG, "║    Height: %.1f px                    ", rect.height);
        ESP_LOGI(TAG, "╠═══════════════════════════════════════╣");
        ESP_LOGI(TAG, "║  Size (mm):                           ║");
        ESP_LOGI(TAG, "║    Width:  %.1f mm                    ", rect.width_mm);
        ESP_LOGI(TAG, "║    Height: %.1f mm                    ", rect.height_mm);
        ESP_LOGI(TAG, "╠═══════════════════════════════════════╣");
        ESP_LOGI(TAG, "║  Corners:                             ║");
        for (int i = 0; i < 4; i++) {
            ESP_LOGI(TAG, "║    [%d]: (%.1f, %.1f)               ", i, rect.corners[i].x, rect.corners[i].y);
        }
        ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    } else {
        ESP_LOGW(TAG, "No object detected!");
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Files saved:");
    ESP_LOGI(TAG, "  - img%d.ppm  (original color)", photo_num);
    ESP_LOGI(TAG, "  - res%d.ppm  (with rectangle)", photo_num);

    esp_vfs_fat_sdcard_unmount("/sdcard", NULL);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, ">>> SD CARD SAFE TO REMOVE! <<<");
}
