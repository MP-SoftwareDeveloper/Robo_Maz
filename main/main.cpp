// ================================================================
//  main.cpp
//  RoboMAZ - Entry point
//  ESP32-S3 / ESP-IDF  |  C++17
// ================================================================

// Required: ESP-IDF app_main must be declared as C, not C++
extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_log.h"
}

#include "MecanumRobot.hpp"

static const char* TAG = "RoboMAZ";

// ── Pin definitions ──────────────────────────────────────────────
//   Viewed from TOP, front of robot facing UP:
//
//       [FL]─────[FR]
//        │   TOP   │
//       [RL]─────[RR]
//
static constexpr MotorPins PINS_FRONT_LEFT  = { .cw_gpio = 39, .ccw_gpio = 40 };
static constexpr MotorPins PINS_FRONT_RIGHT = { .cw_gpio = 42, .ccw_gpio = 41 };
static constexpr MotorPins PINS_REAR_LEFT   = { .cw_gpio =  2, .ccw_gpio =  1 };
static constexpr MotorPins PINS_REAR_RIGHT  = { .cw_gpio =  3, .ccw_gpio =  4 };

// ── Global robot instance ────────────────────────────────────────
static MecanumRobot robot(PINS_FRONT_LEFT,
                           PINS_FRONT_RIGHT,
                           PINS_REAR_LEFT,
                           PINS_REAR_RIGHT);

// ── Helper ───────────────────────────────────────────────────────
static void delay_ms(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

// ================================================================
//  app_main
// ================================================================
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "   Hello Robo MAZ! Starting up  ");
    ESP_LOGI(TAG, "================================");

    robot.begin();
    delay_ms(1000); // settle time

    // ── Demo sequence ────────────────────────────────────────────
    ESP_LOGI(TAG, "--- Demo: Forward");
    robot.moveForward(60.0f);
    delay_ms(1500);
    robot.brake();
    delay_ms(500);

    ESP_LOGI(TAG, "--- Demo: Backward");
    robot.moveBackward(60.0f);
    delay_ms(1500);
    robot.brake();
    delay_ms(500);

    ESP_LOGI(TAG, "--- Demo: Strafe Left");
    robot.strafeLeft(60.0f);
    delay_ms(1500);
    robot.brake();
    delay_ms(500);

    ESP_LOGI(TAG, "--- Demo: Strafe Right");
    robot.strafeRight(60.0f);
    delay_ms(1500);
    robot.brake();
    delay_ms(500);

    ESP_LOGI(TAG, "--- Demo: Rotate CW");
    robot.rotateClockwise(50.0f);
    delay_ms(1500);
    robot.brake();
    delay_ms(500);

    ESP_LOGI(TAG, "--- Demo: Rotate CCW");
    robot.rotateCounterClockwise(50.0f);
    delay_ms(1500);
    robot.coast();

    ESP_LOGI(TAG, "--- Demo complete. Idle.");

    // ── Main loop ────────────────────────────────────────────────
    while (true) {
        delay_ms(1000);
    }
}