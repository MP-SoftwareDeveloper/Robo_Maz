// ================================================================
//  MecanumRobot.hpp
//  RoboMAZ - Mecanum Wheel Robot Controller
//  ESP32-S3 / ESP-IDF  |  C++17
// ================================================================
#pragma once

#include <cstdint>
#include "driver/ledc.h"

// ── Pin bundle for one motor ─────────────────────────────────────
struct MotorPins {
    int cw_gpio;   // IN1 on DRV8833 → Clockwise
    int ccw_gpio;  // IN2 on DRV8833 → Counter-Clockwise
};

// ── LEDC PWM config ──────────────────────────────────────────────
static constexpr uint32_t PWM_FREQUENCY_HZ = 20000; // 20kHz (inaudible)
static constexpr uint32_t PWM_RESOLUTION    = LEDC_TIMER_10_BIT; // 0–1023
static constexpr uint32_t PWM_MAX_DUTY      = 1023;

// ── Motor index ──────────────────────────────────────────────────
enum MotorID {
    MOTOR_FRONT_LEFT  = 0,
    MOTOR_FRONT_RIGHT = 1,
    MOTOR_REAR_LEFT   = 2,
    MOTOR_REAR_RIGHT  = 3,
    MOTOR_COUNT       = 4
};

// ── Direction for a single motor ─────────────────────────────────
enum class MotorDirection {
    FORWARD,
    BACKWARD,
    BRAKE,
    COAST
};

// ================================================================
//  MecanumRobot class
// ================================================================
class MecanumRobot {
public:
    // ── Constructor ──────────────────────────────────────────────
    MecanumRobot(MotorPins frontLeft,
                 MotorPins frontRight,
                 MotorPins rearLeft,
                 MotorPins rearRight);

    // ── Initialise GPIO + PWM (call once in app_main) ────────────
    void begin();

    // ── Basic cardinal movements ─────────────────────────────────
    void moveForward (float speed_pct);   // 0.0 – 100.0 %
    void moveBackward(float speed_pct);
    void strafeLeft  (float speed_pct);   // pure sideways left
    void strafeRight (float speed_pct);   // pure sideways right

    // ── Diagonal movements (Mecanum exclusive) ───────────────────
    void moveDiagonalFrontLeft (float speed_pct);
    void moveDiagonalFrontRight(float speed_pct);
    void moveDiagonalRearLeft  (float speed_pct);
    void moveDiagonalRearRight (float speed_pct);

    // ── Rotation ─────────────────────────────────────────────────
    void rotateClockwise       (float speed_pct);
    void rotateCounterClockwise(float speed_pct);

    // ── Stop modes ───────────────────────────────────────────────
    void brake();   // DRV8833: IN1=1 IN2=1 → instant stop
    void coast();   // DRV8833: IN1=0 IN2=0 → freewheel

    // ── Convenience ──────────────────────────────────────────────
    void setGlobalSpeed(float speed_pct); // change speed without direction change

private:
    // ── Per-motor LEDC channels (CW=even, CCW=odd) ───────────────
    // Motor 0 FL: ch0(cw) ch1(ccw)
    // Motor 1 FR: ch2(cw) ch3(ccw)
    // Motor 2 RL: ch4(cw) ch5(ccw)
    // Motor 3 RR: ch6(cw) ch7(ccw)
    MotorPins _pins[MOTOR_COUNT];

    ledc_channel_t _cwChannel (MotorID id) const;
    ledc_channel_t _ccwChannel(MotorID id) const;

    // ── Low-level motor drive ────────────────────────────────────
    void _driveMotor(MotorID id, MotorDirection dir, float speed_pct);

    // ── Apply a 4-motor command in one call ──────────────────────
    // speed_pct can be negative → reverses direction automatically
    void _applyMotorVector(float fl, float fr, float rl, float rr);

    // ── Helper: clamp + convert % → LEDC duty ────────────────────
    static uint32_t _percentToDuty(float pct);

    // ── PWM setup helpers ────────────────────────────────────────
    void _initTimer();
    void _initChannel(ledc_channel_t channel, int gpio);

    float _currentSpeed = 50.0f; // default 50%
};