#include "ultrasonic.h"

#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

static const char *TAG = "ultrasonic";

// 1 MHz capture resolution = 1 us per tick
#define CAPTURE_RES_HZ   1000000UL

// Speed of sound at ~30°C = 349 m/s
#define SOUND_SPEED_M_S  349.0f

static int s_trig_gpio = -1;
static mcpwm_cap_timer_handle_t   s_cap_timer = NULL;
static mcpwm_cap_channel_handle_t s_cap_chan  = NULL;

typedef struct {
    uint32_t     start_time;   // captured on rising edge (ticks)
    uint32_t     duration;     // falling_tick - rising_tick
    TaskHandle_t task;         // task to notify when echo done
} echo_data_t;

static echo_data_t s_echo = {0};

// ISR: called on every capture edge (rising AND falling)
static bool IRAM_ATTR echo_isr(mcpwm_cap_channel_handle_t cap_chan,
                                const mcpwm_capture_event_data_t *edata,
                                void *user_data)
{
    echo_data_t *echo = (echo_data_t *)user_data;
    BaseType_t wake = pdFALSE;

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        // Rising edge: ECHO went HIGH — start of pulse
        echo->start_time = edata->cap_value;
    } else {
        // Falling edge: ECHO went LOW — end of pulse
        echo->duration = edata->cap_value - echo->start_time;
        // Wake up the measurement task
        vTaskNotifyGiveFromISR(echo->task, &wake);
    }
    return wake == pdTRUE;
}

esp_err_t ultrasonic_init(const ultrasonic_sensor_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    s_trig_gpio = dev->trigger_pin;

    // --- Trigger pin: output ---
    gpio_config_t trig_cfg = {
        .pin_bit_mask = 1ULL << dev->trigger_pin,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&trig_cfg));
    gpio_set_level(dev->trigger_pin, 0);

    // --- MCPWM capture timer ---
    mcpwm_capture_timer_config_t timer_cfg = {
        .clk_src       = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id      = 0,
        .resolution_hz = CAPTURE_RES_HZ,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&timer_cfg, &s_cap_timer));

    // --- Capture channel on ECHO pin (both edges) ---
    mcpwm_capture_channel_config_t chan_cfg = {
        .gpio_num       = dev->echo_pin,
        .prescale       = 1,
        .flags.neg_edge = true,   // capture falling edge
        .flags.pos_edge = true,   // capture rising edge
        .flags.pull_up  = false,  // HC-SR04 drives ECHO actively, no pull-up needed
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(s_cap_timer, &chan_cfg, &s_cap_chan));

    // --- Register ISR callback ---
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = echo_isr,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(s_cap_chan, &cbs, &s_echo));

    // --- Enable and start ---
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(s_cap_chan));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(s_cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(s_cap_timer));

    ESP_LOGI(TAG, "MCPWM capture ready — TRIG=GPIO%d  ECHO=GPIO%d  res=%luHz",
             dev->trigger_pin, dev->echo_pin, (unsigned long)CAPTURE_RES_HZ);
    return ESP_OK;
}

esp_err_t ultrasonic_measure(const ultrasonic_sensor_t *dev, uint32_t max_distance_cm, float *distance_m)
{
    if (!dev || !distance_m) return ESP_ERR_INVALID_ARG;

    // Tell the ISR which task to notify
    s_echo.task     = xTaskGetCurrentTaskHandle();
    s_echo.duration = 0;

    // Clear any stale notification from a previous measurement
    ulTaskNotifyTake(pdTRUE, 0);

    // Send 10us trigger pulse (software — only the ECHO side is hardware)
    gpio_set_level(s_trig_gpio, 0);
    ets_delay_us(10);
    gpio_set_level(s_trig_gpio, 1);
    ets_delay_us(20);
    gpio_set_level(s_trig_gpio, 0);

    // Wait for ISR to notify us (30ms timeout covers up to ~5m range)
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(30)) == 0) {
        return ESP_ERR_ULTRASONIC_PING_TIMEOUT;
    }

    // duration is in capture ticks (1MHz → 1 tick = 1us)
    // distance = (time_s * speed_m_s) / 2
    *distance_m = ((float)s_echo.duration / CAPTURE_RES_HZ) * SOUND_SPEED_M_S / 2.0f;
    return ESP_OK;
}
