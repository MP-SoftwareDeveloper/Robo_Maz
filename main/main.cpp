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
//    ├── PWM/
//    │   ├── MAZPWM.hpp
//    │   └── MAZPWM.cpp
//    └── LCD/
//        ├── MAZLCD.hpp
//        └── MAZLCD.cpp
//
//  CMakeLists.txt SRCS must include:
//    "Motor/MecanumRobot.cpp"
//    "PWM/MAZPWM.cpp"
//    "LCD/MAZLCD.cpp"
// ================================================================

// Required: ESP-IDF app_main must be declared as C, not C++
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
}

#include "Motor/MecanumRobot.hpp"  // pulls in PWM/MAZPWM.hpp transitively
#include "LCD/MAZLCD.hpp"
#include <cstdio>

static const char *TAG = "RoboMAZ";

#define BlinkLED_GPIO GPIO_NUM_46

// NOTE: effective speed range is ~70–100 %. Below ~65 % motors stall (dead band — static friction
// exceeds torque at low duty). Values outside this range compile fine but motors will not move.
static constexpr float DRIVE_SPEED  = 65.0f;   // straight / strafe speed  [70–100 %]
static constexpr float ROTATE_SPEED = 70.0f;  // rotation speed           [65–100 %]

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
static MAZLCD lcd;

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
    // GPIO46 — simple green LED
    gpio_reset_pin(BlinkLED_GPIO);
    gpio_set_direction(BlinkLED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BlinkLED_GPIO, 0);

}


extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "   Hello Robo MAZ! Starting up  ");
    ESP_LOGI(TAG, "================================");

    // Initialise status LED
    init_led();

    // Initialise LCD (SDA=GPIO05, SCL=GPIO06, address=0x27)
    lcd.init(GPIO_NUM_5, GPIO_NUM_6, 0x27);  // clears display internally
    lcd.setCursor(0, 0);
    lcd.print("                ");  // pre-clear row 0
    lcd.setCursor(0, 1);
    lcd.print("                ");  // pre-clear row 1
    lcd.setCursor(0, 0);
    lcd.print("  RoboMAZ Ready!");

    robot.begin();

    delay_ms(1000); // settle: let power rails stabilise

    // ── Demo sequence — repeated 4 times ────────────────────────
    for (int pass = 1; pass <= 1; ++pass) {
        ESP_LOGI(TAG, "=== Demo pass %d / 4 ===", pass);

        ESP_LOGI(TAG, "--- Forward");
        robot.moveForward(DRIVE_SPEED);
        delay_ms(500);
        robot.brake();
        delay_ms(200);

        ESP_LOGI(TAG, "--- Backward");
        robot.moveBackward(DRIVE_SPEED);
        delay_ms(500);
        robot.brake();
        delay_ms(200);

        ESP_LOGI(TAG, "--- Strafe Left");
        robot.strafeLeft(DRIVE_SPEED);
        delay_ms(500);
        robot.brake();
        delay_ms(200);

        ESP_LOGI(TAG, "--- Strafe Right");
        robot.strafeRight(DRIVE_SPEED);
        delay_ms(500);
        robot.brake();
        delay_ms(200);

        ESP_LOGI(TAG, "--- Rotate CW");
        robot.rotateClockwise(ROTATE_SPEED);
        delay_ms(500);
        robot.brake();
        delay_ms(200);

        ESP_LOGI(TAG, "--- Rotate CCW");
        robot.rotateCounterClockwise(ROTATE_SPEED);
        delay_ms(500);
        robot.brake();
        delay_ms(200);
    }

    robot.coast();
    ESP_LOGI(TAG, "--- Demo complete. Idle.");

    // ── Main loop — LCD counter 0–255, repeating every 500 ms ───
    uint8_t count = 0;
    while (true)
    {
        char buf[17];
        snprintf(buf, sizeof(buf), "Count: %-9u", count);
        lcd.setCursor(0, 1);
        lcd.print(buf);
        ++count;  // wraps 255 → 0 automatically (uint8_t overflow)

        gpio_set_level(BlinkLED_GPIO, count & 1);
        delay_ms(300);
    }
}
