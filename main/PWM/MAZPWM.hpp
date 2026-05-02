// ================================================================
//  MAZPWM.hpp
//  RoboMAZ - Bare-metal PWM Driver
//  ESP32-S3 / ESP-IDF v6  |  C++20
//
//  Directly programs MCPWM hardware registers.
//  No LEDC, no IDF driver wrappers.
//
//  ESP32-S3 hardware layout (IDF v5 / v6):
//    Two independent MCPWM units: MCPWM0, MCPWM1
//    Each unit has: 3 timers, 3 operators, 2 generators per operator
//    Generator A → CW  pin (DRV8833 IN1)
//    Generator B → CCW pin (DRV8833 IN2)
//
//  Motor → peripheral mapping:
//    Motor 0 (FL) → MCPWM0, operator 0
//    Motor 1 (FR) → MCPWM0, operator 1
//    Motor 2 (RL) → MCPWM0, operator 2
//    Motor 3 (RR) → MCPWM1, operator 0
//
//  IDF v6 bare-metal notes (ESP32-S3 specific):
//    - Clock enable: SYSTEM_PERIP_CLK_EN0_REG (NOT CLK_EN1) — PWM0/1 bits
//      live in register 0 (offset +0x18), confirmed by esp_hal_mcpwm LL
//      RST bits: SYSTEM_PERIP_RST_EN0_REG (NOT RST_EN1), same reason
//    - Operator members: dev->operators[i]  (NOT dev->channel[i])
//    - Timer-operator link: dev->operator_timersel (top-level, not per-operator)
//    - GPIO routing: GPIO struct (soc/gpio_struct.h), NOT register macros
// ================================================================
#pragma once

#include <cstdint>
#include "soc/mcpwm_struct.h"   // mcpwm_dev_t, MCPWM0, MCPWM1
#include "soc/mcpwm_reg.h"      // base address macros (included for completeness)
#include "soc/gpio_struct.h"    // GPIO (gpio_dev_t struct overlay)
#include "soc/gpio_sig_map.h"   // PWM0_OUT0A_IDX … PWM1_OUT0B_IDX
#include "soc/system_reg.h"           // SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_PWM0/1_CLK_EN + RST
#include "soc/io_mux_reg.h"     // PIN_FUNC_SELECT, PIN_FUNC_GPIO
#include "soc/gpio_periph.h"    // GPIO_PIN_MUX_REG[]

// ── Clock & frequency ────────────────────────────────────────────
/**
 * @brief MCPWM source clock (160 MHz at 240 MHz CPU on ESP32-S3).
 */
static constexpr uint32_t MCPWM_CLK_HZ    = 160'000'000UL;

/**
 * @brief PWM carrier frequency — 20 kHz is above human hearing.
 */
static constexpr uint32_t PWM_FREQ_HZ     = 20'000UL;

/**
 * @brief Global clock prescaler written to clk_cfg.clk_prescale.
 *        Timer clock = 160 MHz / (PWM_PRESCALER + 1) = 20 MHz.
 */
static constexpr uint8_t  PWM_PRESCALER   = 7;

/**
 * @brief Timer clock after prescaling: 20 MHz.
 */
static constexpr uint32_t PWM_TIMER_CLK   = MCPWM_CLK_HZ / (PWM_PRESCALER + 1);

/**
 * @brief Timer ticks per PWM period: 20 MHz / 20 kHz = 1000 ticks.
 *        Compare values must be in [0, PWM_PERIOD_TICKS].
 */
static constexpr uint32_t PWM_PERIOD_TICKS = PWM_TIMER_CLK / PWM_FREQ_HZ;

// ── Counts ───────────────────────────────────────────────────────
static constexpr uint8_t MAZ_MOTOR_COUNT    = 4;
static constexpr uint8_t MCPWM_OPS_PER_UNIT = 3; // ESP32-S3 hardware limit

// ================================================================
//  MAZPWM
// ================================================================
/**
 * @brief Bare-metal MCPWM driver for four independent motor channels.
 *
 * Motors 0–2 run on MCPWM0 (operators 0–2).
 * Motor  3   runs on MCPWM1 (operator  0).
 *
 * Each motor gets two PWM outputs routed through the GPIO matrix:
 *   Generator A → CW  pin (DRV8833 IN1)
 *   Generator B → CCW pin (DRV8833 IN2)
 *
 * @code
 *   MAZPWM pwm;
 *   pwm.init();
 *   pwm.attachMotorPin(0, 39, 40);      // motor 0: CW=39, CCW=40
 *   pwm.setMotorDuty(0, 75.0f, 0.0f);  // 75 % forward
 *   pwm.setMotorDuty(0, 0.0f, 60.0f);  // 60 % reverse
 *   pwm.setMotorDuty(0, 0.0f, 0.0f);   // coast
 * @endcode
 */
class MAZPWM {
public:
    /**
     * @brief Enables clocks and fully configures both MCPWM units.
     *        Must be called once before attachMotorPin() or setMotorDuty().
     */
    void init();

    /**
     * @brief Routes generator A (CW) and B (CCW) to physical GPIO pins.
     *
     * @param motorIdx  Motor index 0–3
     * @param cwGpio    GPIO for clockwise output  (DRV8833 IN1)
     * @param ccwGpio   GPIO for counter-clockwise output (DRV8833 IN2)
     */
    void attachMotorPin(uint8_t motorIdx, int cwGpio, int ccwGpio);

    /**
     * @brief Sets duty cycles for both direction pins of one motor.
     *        Changes take effect at the next PWM period boundary.
     *
     *        Forward:  cwDuty > 0,   ccwDuty = 0
     *        Reverse:  cwDuty = 0,   ccwDuty > 0
     *        Coast:    cwDuty = 0,   ccwDuty = 0
     *        Brake:    cwDuty = 100, ccwDuty = 100
     *
     * @param motorIdx  Motor index 0–3
     * @param cwDuty    CW  duty percentage  [0.0 – 100.0]
     * @param ccwDuty   CCW duty percentage  [0.0 – 100.0]
     */
    void setMotorDuty(uint8_t motorIdx, float cwDuty, float ccwDuty);

    /**
     * @brief Coasts one motor (sets both duties to 0 %).
     * @param motorIdx  Motor index 0–3
     */
    void stopMotor(uint8_t motorIdx);

    /**
     * @brief Coasts all four motors. Safe to call at any time.
     */
    void stopAll();

private:
    /**
     * @brief Returns a pointer to the register struct of the MCPWM unit
     *        that owns the given motor index.
     *        Motors 0–2 → &MCPWM0 | Motor 3 → &MCPWM1
     */
    static volatile mcpwm_dev_t* _dev(uint8_t motorIdx);

    /**
     * @brief Returns the operator index (0–2) inside the peripheral
     *        for the given motor. Motor 3 wraps back to op 0 on MCPWM1.
     */
    static uint8_t _opIdx(uint8_t motorIdx);

    /**
     * @brief Enables clock gate for one MCPWM unit on ESP32-S3.
     *        Uses periph_module_enable() with a shared_periph_module_t
     *        cast — ESP32-S3 has no PCR peripheral (that is ESP32-H4/C6 only).
     *        unitIdx: 0 = MCPWM0, 1 = MCPWM1
     */
    static void _enableClock(uint8_t unitIdx);

    /**
     * @brief Configures all three timers and operators of one unit.
     *        Called once for MCPWM0 and once for MCPWM1.
     *
     * @param dev  Pointer to the peripheral's register struct
     */
    static void _initUnit(volatile mcpwm_dev_t* dev);

    /**
     * @brief Converts a duty percentage to compare ticks.
     *        Clamped to [0, PWM_PERIOD_TICKS].
     */
    static uint32_t _dutyToTicks(float pct);

    /**
     * @brief Routes an MCPWM signal to a GPIO via the GPIO matrix
     *        and enables the output driver.
     *
     * @param gpio       Physical GPIO number
     * @param signalIdx  MCPWM signal index from gpio_sig_map.h
     */
    static void _routeGpio(int gpio, uint32_t signalIdx);
};