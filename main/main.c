#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#define BLINK_GPIO  46
#define M1_CW_GPIO  39
#define M1_CCW_GPIO 40

static const char *TAG = "RoboMAZ";

static uint8_t s_led_state = 0;
static uint32_t cw_count  = 0;
static uint32_t ccw_count = 0;
static uint32_t brake_count = 0;
static uint32_t coast_count = 0;

void init_motor(void)
{
    gpio_reset_pin(M1_CW_GPIO);
    gpio_reset_pin(M1_CCW_GPIO);
    gpio_set_direction(M1_CW_GPIO,  GPIO_MODE_OUTPUT);
    gpio_set_direction(M1_CCW_GPIO, GPIO_MODE_OUTPUT);
    // Always start in COAST (safe default)
    gpio_set_level(M1_CW_GPIO,  0);
    gpio_set_level(M1_CCW_GPIO, 0);
    ESP_LOGI(TAG, "Motor initialized (COAST)");
}

static void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void blink_led(void)
{
    gpio_set_level(BLINK_GPIO, s_led_state);
}

void motor1_CW(void)
{
    gpio_set_level(M1_CW_GPIO,  1);
    gpio_set_level(M1_CCW_GPIO, 0);
    cw_count++;
    ESP_LOGI(TAG, "Motor CW  --> Run #%lu", cw_count);
}

void motor1_CCW(void)
{
    gpio_set_level(M1_CW_GPIO,  0);
    gpio_set_level(M1_CCW_GPIO, 1);
    ccw_count++;
    ESP_LOGI(TAG, "Motor CCW --> Run #%lu", ccw_count);
}

// DRV8833: IN1=0, IN2=0 → motor freewheels to a gradual stop
void motor1_coast(void)
{
    gpio_set_level(M1_CW_GPIO,  0);
    gpio_set_level(M1_CCW_GPIO, 0);
    coast_count++;
    ESP_LOGI(TAG, "Motor COAST (freewheel) --> #%lu", coast_count);
}

// DRV8833: IN1=1, IN2=1 → motor brakes to an immediate stop
void motor1_brake(void)
{
    gpio_set_level(M1_CW_GPIO,  1);
    gpio_set_level(M1_CCW_GPIO, 1);
    brake_count++;
    ESP_LOGI(TAG, "Motor BRAKE (fast stop) --> #%lu", brake_count);
}

void app_main(void)
{
    ESP_LOGI(TAG, "==============================");
    ESP_LOGI(TAG, "  Hello Robo MAZ! Starting up ");
    ESP_LOGI(TAG, "==============================");

    configure_led();
    init_motor();

    while (1) {
        // --- CW for 500ms then BRAKE ---
        blink_led();
        motor1_CW();
        s_led_state = !s_led_state;
        vTaskDelay(500 / portTICK_PERIOD_MS);

        motor1_brake();
        vTaskDelay(500 / portTICK_PERIOD_MS);

        // --- CCW for 1500ms then COAST ---
        motor1_CCW();
        vTaskDelay(1500 / portTICK_PERIOD_MS);

        motor1_coast();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}