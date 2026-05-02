// ================================================================
//  main.cpp
//  RoboMAZ - Entry point
//  ESP32-S3 / ESP-IDF  |  C++20
//
//  Project folder layout:
//
//    main/
//    ├── main.cpp                 ← you are here
//    ├── CMakeLists.txt
//    ├── Motor/
//    │   ├── MecanumRobot.hpp
//    │   └── MecanumRobot.cpp
//    └── PWM/
//        ├── MAZPWM.hpp
//        └── MAZPWM.cpp
//
//  CMakeLists.txt SRCS must include:
//    "Motor/MecanumRobot.cpp"
//    "PWM/MAZPWM.cpp"
// ================================================================

// Required: ESP-IDF app_main must be declared as C, not C++
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
}

#include "Motor/MecanumRobot.hpp" // pulls in PWM/MAZPWM.hpp transitively

static const char *TAG = "RoboMAZ";

#define BlinkLED_GPIO GPIO_NUM_46 // GPIO for an optional status LED (not used in this example)

// ── Pin definitions ──────────────────────────────────────────────
//   Viewed from TOP, front of robot facing UP:
//
//       [FL]─────[FR]
//        │   TOP   │
//       [RL]─────[RR]
//
// Each MotorPins struct maps to one DRV8833 channel:
//   cw_gpio  → IN1 (clockwise when HIGH)
//   ccw_gpio → IN2 (counter-clockwise when HIGH)
static constexpr MotorPins PINS_FRONT_LEFT = {.cw_gpio = 39, .ccw_gpio = 40};
static constexpr MotorPins PINS_FRONT_RIGHT = {.cw_gpio = 42, .ccw_gpio = 41};
static constexpr MotorPins PINS_REAR_LEFT = {.cw_gpio = 2, .ccw_gpio = 1};
static constexpr MotorPins PINS_REAR_RIGHT = {.cw_gpio = 3, .ccw_gpio = 4};

// ── Global robot instance ────────────────────────────────────────
// Constructed before app_main; hardware not touched until begin() is called.
static MecanumRobot robot(PINS_FRONT_LEFT,
                          PINS_FRONT_RIGHT,
                          PINS_REAR_LEFT,
                          PINS_REAR_RIGHT);

// ── Helper ───────────────────────────────────────────────────────
/**
 * @brief Blocks the calling FreeRTOS task for the given number of
 *        milliseconds using the scheduler tick period.
 *
 * @param ms  Duration to wait in milliseconds
 */
static void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// ================================================================
//  app_main
// ================================================================
/**
 * @brief ESP-IDF application entry point.
 *
 * Execution order:
 *   1. Startup banner via ESP_LOGI.
 *   2. robot.begin() → initialises bare-metal MCPWM + GPIO routing.
 *   3. 1 s settle delay for power rails.
 *   4. Demo sequence — exercises every movement primitive.
 *   5. Idle loop — yields to FreeRTOS scheduler every second.
 *
 * Must be declared extern "C" so the linker can find it by its
 * unmangled C symbol name.
 */
static void init_led()
{
    gpio_reset_pin(BlinkLED_GPIO);
    gpio_set_direction(BlinkLED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BlinkLED_GPIO, 0); // off by default
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "   Hello Robo MAZ! Starting up  ");
    ESP_LOGI(TAG, "================================");

    // Initialise status LED
    init_led();

    // Initialise bare-metal MCPWM peripheral + GPIO matrix routing
    robot.begin();
    delay_ms(1000); // settle: let power rails stabilise

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
    while (true)
    {
        delay_ms(1500);
        gpio_set_level(BlinkLED_GPIO, 1); // off
        delay_ms(500);
        gpio_set_level(BlinkLED_GPIO, 0); // on
    }
}
