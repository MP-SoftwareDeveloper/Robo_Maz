// ================================================================
//  MecanumRobot.cpp
//  RoboMAZ - Mecanum Wheel Robot Controller
//  ESP32-S3 / ESP-IDF  |  C++17
// ================================================================
#include "MecanumRobot.hpp"
#include "esp_log.h"
#include <algorithm>
#include <cmath>


static const char* TAG = "MecanumRobot";

// ================================================================
//  Constructor
// ================================================================
MecanumRobot::MecanumRobot(MotorPins frontLeft,
                            MotorPins frontRight,
                            MotorPins rearLeft,
                            MotorPins rearRight)
{
    _pins[MOTOR_FRONT_LEFT]  = frontLeft;
    _pins[MOTOR_FRONT_RIGHT] = frontRight;
    _pins[MOTOR_REAR_LEFT]   = rearLeft;
    _pins[MOTOR_REAR_RIGHT]  = rearRight;
}

// ================================================================
//  begin() — call once from app_main
// ================================================================
void MecanumRobot::begin()
{
    ESP_LOGI(TAG, "Initialising MecanumRobot PWM @ %lu Hz, %lu-bit resolution",
             PWM_FREQUENCY_HZ, (uint32_t)LEDC_TIMER_10_BIT);

    _initTimer();

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        _initChannel(_cwChannel (static_cast<MotorID>(i)), _pins[i].cw_gpio);
        _initChannel(_ccwChannel(static_cast<MotorID>(i)), _pins[i].ccw_gpio);
    }

    coast(); // safe default: all motors off
    //ESP_LOGI(TAG, "All motors initialised → COAST");
}

// ================================================================
//  Cardinal movements
// ================================================================
void MecanumRobot::moveForward(float speed_pct)
{
    //ESP_LOGI(TAG, ">> FORWARD  %.1f%%", speed_pct);
    _applyMotorVector(+speed_pct, +speed_pct,
                      +speed_pct, +speed_pct);
}

void MecanumRobot::moveBackward(float speed_pct)
{
    //ESP_LOGI(TAG, ">> BACKWARD  %.1f%%", speed_pct);
    _applyMotorVector(-speed_pct, -speed_pct,
                      -speed_pct, -speed_pct);
}

void MecanumRobot::strafeLeft(float speed_pct)
{
    //ESP_LOGI(TAG, ">> STRAFE LEFT  %.1f%%", speed_pct);
    //        FL        FR        RL        RR
    _applyMotorVector(-speed_pct, +speed_pct,
                      +speed_pct, -speed_pct);
}

void MecanumRobot::strafeRight(float speed_pct)
{
    ESP_LOGI(TAG, ">> STRAFE RIGHT  %.1f%%", speed_pct);
    _applyMotorVector(+speed_pct, -speed_pct,
                      -speed_pct, +speed_pct);
}

// ================================================================
//  Diagonal movements
// ================================================================
void MecanumRobot::moveDiagonalFrontLeft(float speed_pct)
{
    //ESP_LOGI(TAG, ">> DIAGONAL FRONT-LEFT  %.1f%%", speed_pct);
    _applyMotorVector(0,          +speed_pct,
                      +speed_pct, 0);
}

void MecanumRobot::moveDiagonalFrontRight(float speed_pct)
{
    ESP_LOGI(TAG, ">> DIAGONAL FRONT-RIGHT  %.1f%%", speed_pct);
    _applyMotorVector(+speed_pct, 0,
                      0,          +speed_pct);
}

void MecanumRobot::moveDiagonalRearLeft(float speed_pct)
{
    ESP_LOGI(TAG, ">> DIAGONAL REAR-LEFT  %.1f%%", speed_pct);
    _applyMotorVector(-speed_pct, 0,
                      0,          -speed_pct);
}

void MecanumRobot::moveDiagonalRearRight(float speed_pct)
{
    //ESP_LOGI(TAG, ">> DIAGONAL REAR-RIGHT  %.1f%%", speed_pct);
    _applyMotorVector(0,          -speed_pct,
                      -speed_pct, 0);
}

// ================================================================
//  Rotation
// ================================================================
void MecanumRobot::rotateClockwise(float speed_pct)
{
    //ESP_LOGI(TAG, ">> ROTATE CW  %.1f%%", speed_pct);
    _applyMotorVector(+speed_pct, -speed_pct,
                      +speed_pct, -speed_pct);
}

void MecanumRobot::rotateCounterClockwise(float speed_pct)
{
    //ESP_LOGI(TAG, ">> ROTATE CCW  %.1f%%", speed_pct);
    _applyMotorVector(-speed_pct, +speed_pct,
                      -speed_pct, +speed_pct);
}

// ================================================================
//  Stop modes
// ================================================================
void MecanumRobot::brake()
{
    //ESP_LOGI(TAG, "-- BRAKE");
    for (int i = 0; i < MOTOR_COUNT; ++i)
        _driveMotor(static_cast<MotorID>(i), MotorDirection::BRAKE, 0);
}

void MecanumRobot::coast()
{
    //ESP_LOGI(TAG, "-- COAST");
    for (int i = 0; i < MOTOR_COUNT; ++i)
        _driveMotor(static_cast<MotorID>(i), MotorDirection::COAST, 0);
}

// ================================================================
//  setGlobalSpeed — update speed without changing direction
// ================================================================
void MecanumRobot::setGlobalSpeed(float speed_pct)
{
    _currentSpeed = std::clamp(speed_pct, 0.0f, 100.0f);
    //ESP_LOGI(TAG, "Global speed set to %.1f%%", _currentSpeed);
}

// ================================================================
//  _applyMotorVector
//  Positive value  → FORWARD
//  Negative value  → BACKWARD
//  Zero            → COAST
// ================================================================
void MecanumRobot::_applyMotorVector(float fl, float fr,
                                      float rl, float rr)
{
    auto resolve = [&](MotorID id, float val) {
        if (val > 0.0f)
            _driveMotor(id, MotorDirection::FORWARD,  val);
        else if (val < 0.0f)
            _driveMotor(id, MotorDirection::BACKWARD, -val);
        else
            _driveMotor(id, MotorDirection::COAST,    0.0f);
    };

    resolve(MOTOR_FRONT_LEFT,  fl);
    resolve(MOTOR_FRONT_RIGHT, fr);
    resolve(MOTOR_REAR_LEFT,   rl);
    resolve(MOTOR_REAR_RIGHT,  rr);
}

// ================================================================
//  _driveMotor — lowest level PWM write to DRV8833
// ================================================================
void MecanumRobot::_driveMotor(MotorID id, MotorDirection dir, float speed_pct)
{
    uint32_t duty = _percentToDuty(speed_pct);

    switch (dir) {
        case MotorDirection::FORWARD:
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _cwChannel(id),  duty);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ccwChannel(id), 0);
            break;

        case MotorDirection::BACKWARD:
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _cwChannel(id),  0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ccwChannel(id), duty);
            break;

        case MotorDirection::BRAKE:
            // DRV8833: IN1=1 IN2=1 → fast stop
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _cwChannel(id),  PWM_MAX_DUTY);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ccwChannel(id), PWM_MAX_DUTY);
            break;

        case MotorDirection::COAST:
        default:
            // DRV8833: IN1=0 IN2=0 → freewheel
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _cwChannel(id),  0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ccwChannel(id), 0);
            break;
    }

    ledc_update_duty(LEDC_LOW_SPEED_MODE, _cwChannel(id));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, _ccwChannel(id));
}

// ================================================================
//  PWM helpers
// ================================================================
ledc_channel_t MecanumRobot::_cwChannel(MotorID id) const
{
    // CW channels: 0, 2, 4, 6
    return static_cast<ledc_channel_t>(id * 2);
}

ledc_channel_t MecanumRobot::_ccwChannel(MotorID id) const
{
    // CCW channels: 1, 3, 5, 7
    return static_cast<ledc_channel_t>(id * 2 + 1);
}

uint32_t MecanumRobot::_percentToDuty(float pct)
{
    float clamped = std::clamp(pct, 0.0f, 100.0f);
    return static_cast<uint32_t>((clamped / 100.0f) * PWM_MAX_DUTY);
}

void MecanumRobot::_initTimer()
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = PWM_FREQUENCY_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
        .deconfigure     = false
    };
    ledc_timer_config(&timer);
}

void MecanumRobot::_initChannel(ledc_channel_t channel, int gpio)
{
    ledc_channel_config_t cfg = {
        .gpio_num   = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
        .sleep_mode  = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = 0,
        .deconfigure     = false
    };
    ledc_channel_config(&cfg);
}