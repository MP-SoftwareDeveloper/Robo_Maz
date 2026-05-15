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
//    ├── LCD/
//    │   ├── MAZLCD.hpp
//    │   └── MAZLCD.cpp
//    └── Compass/
//        ├── MAZGY271.hpp
//        └── MAZGY271.cpp
//
//  CMakeLists.txt SRCS must include:
//    "Motor/MecanumRobot.cpp"
//    "PWM/MAZPWM.cpp"
//    "LCD/MAZLCD.cpp"
//    "Compass/MAZGY271.cpp"
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
#include "Compass/MAZGY271.hpp"
#include "led_strip.h"
#include <cstdio>

static const char *TAG = "RoboMAZ";

#define BlinkLED_GPIO     GPIO_NUM_46
#define FullColorLED_GPIO GPIO_NUM_48  // WS2812B — driven via RMT, not plain GPIO

// NOTE: effective speed range is ~70–100 %. Below ~65 % motors stall (dead band — static friction
// exceeds torque at low duty). Values outside this range compile fine but motors will not move.
static constexpr float DRIVE_SPEED  = 70.0f;   // straight / strafe speed  [70–100 %]
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
static MAZLCD    lcd;
static MAZGY271  compass;
static led_strip_handle_t _rgb_strip = nullptr;

struct RGBColor { uint8_t r, g, b; const char* name; };
static constexpr RGBColor COLOR_CYCLE[] = {
    {255,   0,   0, "Red"},
    {255, 165,   0, "Orange"},
    {255, 255,   0, "Yellow"},
    {  0, 255,   0, "Green"},
    {  0, 255, 255, "Cyan"},
    {  0,   0, 255, "Blue"},
    {128,   0, 128, "Purple"},
    {255,   0, 255, "Magenta"},
    {255, 255, 255, "White"},
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
void motors_demo()
{
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

    // GY-271 joins the same I2C bus the LCD already created
    compass.init(lcd.busHandle());

    robot.begin();

    delay_ms(1000); // settle: let power rails stabilise

    motors_demo();
   

    robot.coast();
    ESP_LOGI(TAG, "--- Demo complete. Idle.");

    // Cardinal direction label from heading (0°=N, clockwise)
    auto compassDir = [](float deg) -> const char* {
        static const char* dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
        return dirs[(int)((deg + 22.5f) / 45.0f) % 8];
    };

    // ── Main loop — compass heading on LCD, color on RGB LED ────
    uint8_t count = 0;
    while (true)
    {
        // RGB LED cycles through colors
        const RGBColor& c = COLOR_CYCLE[count % COLOR_COUNT];
        led_strip_set_pixel(_rgb_strip, 0, c.r, c.g, c.b);
        led_strip_refresh(_rgb_strip);

        // Read compass and display on LCD
        float heading = 0.0f;
        char row0[17], row1[17];

        if (compass.read(heading) == ESP_OK) {
            snprintf(row0, sizeof(row0), "Hdg: %5.1f deg  ", heading);
            snprintf(row1, sizeof(row1), "Dir: %-11s", compassDir(heading));
        } else {
            snprintf(row0, sizeof(row0), "Hdg: --- deg    ");
            snprintf(row1, sizeof(row1), "Dir: ---        ");
        }

        lcd.setCursor(0, 0);
        lcd.print(row0);
        lcd.setCursor(0, 1);
        lcd.print(row1);

        ++count;
        delay_ms(500);
    }
}
