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
#include "led_strip.h"
#include <cstdio>

static const char *TAG = "RoboMAZ";

#define BlinkLED_GPIO     GPIO_NUM_46
#define FullColorLED_GPIO GPIO_NUM_48  // WS2812B — driven via RMT, not plain GPIO

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
static led_strip_handle_t _rgb_strip = nullptr;

struct RGBColor { uint8_t r, g, b; };
static constexpr RGBColor COLOR_CYCLE[] = {
    {255,   0,   0},  // red
    {255, 165,   0},  // orange
    {255, 255,   0},  // yellow
    {  0, 255,   0},  // green
    {  0, 255, 255},  // cyan
    {  0,   0, 255},  // blue
    {128,   0, 128},  // purple
    {255,   0, 255},  // magenta (pink)
    {255, 255, 255},  // white
};
static constexpr uint8_t COLOR_COUNT = sizeof(COLOR_CYCLE) / sizeof(COLOR_CYCLE[0]);

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

    // GPIO48 — WS2812B RGB LED; needs RMT, not plain GPIO
    led_strip_config_t strip_cfg = {};
    strip_cfg.strip_gpio_num        = (int)FullColorLED_GPIO;
    strip_cfg.max_leds              = 1;
    strip_cfg.led_model             = LED_MODEL_WS2812;
    strip_cfg.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    led_strip_rmt_config_t rmt_cfg = {};
    rmt_cfg.resolution_hz = 10 * 1000 * 1000;  // 10 MHz
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &_rgb_strip));
    led_strip_clear(_rgb_strip);
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
        const RGBColor& c = COLOR_CYCLE[count % COLOR_COUNT];
        led_strip_set_pixel(_rgb_strip, 0, c.r, c.g, c.b);
        led_strip_refresh(_rgb_strip);
        delay_ms(300);
    }
}
