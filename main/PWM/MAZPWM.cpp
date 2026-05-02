// ================================================================
//  MAZPWM.cpp
//  RoboMAZ - Bare-metal PWM Driver
//  ESP32-S3 / ESP-IDF v6  |  C++20
//
//  IDF v6 struct layout changes (vs older docs / examples):
//
//    BROKEN (old)                  CORRECT (IDF v6, ESP32-S3)
//    ──────────────────────────────────────────────────────────────
//    dev->channel[i]           →   dev->operators[i]
//    dev->timer_sel_and_start  →   dev->operators[i].operator_timer_sel
//    periph_module_t            →   shared_periph_module_t  (renamed in IDF v6)
//    PCR struct (esp32h4/c6)    →   periph_module_enable()  (ESP32-S3 uses DPORT)
//    PERIPH_MCPWM0_MODULE cast  →   static_cast<periph_module_t>(PERIPH_MCPWM0_MODULE)
//    GPIO_FUNC_OUT_SEL_CFG_REG  →   GPIO.func_out_sel_cfg[n].func_sel
//    GPIO_ENABLE_REG            →   GPIO.enable_w1ts.val
//    GPIO_ENABLE1_REG           →   GPIO.enable1_w1ts.val
//
//  GPIO signal table (gpio_sig_map.h, ESP32-S3, IDF v5+):
//    PWM0_OUT0A_IDX / PWM0_OUT0B_IDX  — MCPWM0 op0 gen A/B
//    PWM0_OUT1A_IDX / PWM0_OUT1B_IDX  — MCPWM0 op1 gen A/B
//    PWM0_OUT2A_IDX / PWM0_OUT2B_IDX  — MCPWM0 op2 gen A/B
//    PWM1_OUT0A_IDX / PWM1_OUT0B_IDX  — MCPWM1 op0 gen A/B
// ================================================================
#include "MAZPWM.hpp"

#include <algorithm>
#include <cstdint>

// ── Generator action values ──────────────────────────────────────
// Written into generator[n].utez / .utea / .uteb:
//   0 = no change | 1 = force LOW | 2 = force HIGH | 3 = toggle
static constexpr uint32_t GEN_LOW  = 1;
static constexpr uint32_t GEN_HIGH = 2;

// ── GPIO signal index table ──────────────────────────────────────
// [motorIdx][0] = generator-A signal (CW  / DRV8833 IN1)
// [motorIdx][1] = generator-B signal (CCW / DRV8833 IN2)
static const uint32_t kMotorSignal[MAZ_MOTOR_COUNT][2] = {
    { PWM0_OUT0A_IDX, PWM0_OUT0B_IDX },  // Motor 0 (FL) — MCPWM0 op0
    { PWM0_OUT1A_IDX, PWM0_OUT1B_IDX },  // Motor 1 (FR) — MCPWM0 op1
    { PWM0_OUT2A_IDX, PWM0_OUT2B_IDX },  // Motor 2 (RL) — MCPWM0 op2
    { PWM1_OUT0A_IDX, PWM1_OUT0B_IDX },  // Motor 3 (RR) — MCPWM1 op0
};

// ================================================================
//  _dev()  — select peripheral by motor index
// ================================================================
/**
 * Returns a pointer to the MCPWM register struct for the unit that
 * owns the given motor.
 *   Motors 0–2 → MCPWM0
 *   Motor  3   → MCPWM1
 */
volatile mcpwm_dev_t* MAZPWM::_dev(uint8_t motorIdx)
{
    return (motorIdx < MCPWM_OPS_PER_UNIT) ? &MCPWM0 : &MCPWM1;
}

// ================================================================
//  _opIdx()  — operator index within the peripheral
// ================================================================
/**
 * Returns the operator index (0–2) inside the peripheral:
 *   Motor 0 → 0  (MCPWM0)
 *   Motor 1 → 1  (MCPWM0)
 *   Motor 2 → 2  (MCPWM0)
 *   Motor 3 → 0  (MCPWM1 — wraps with % 3)
 */
uint8_t MAZPWM::_opIdx(uint8_t motorIdx)
{
    return motorIdx % MCPWM_OPS_PER_UNIT;
}

// ================================================================
//  _enableClock()  — enable MCPWM peripheral clock (ESP32-S3 / IDF v6)
// ================================================================
/**
 * Enables the clock and releases reset for one MCPWM unit.
 *
 * ESP32-S3 uses the DPORT clock-gate system, NOT the PCR peripheral
 * (pcr_struct.h only exists for ESP32-H4/C6). The correct API is
 * periph_module_enable(), but IDF v6 renamed the enum type from
 * periph_module_t to shared_periph_module_t. The enum *values*
 * PERIPH_MCPWM0_MODULE and PERIPH_MCPWM1_MODULE still exist — only
 * the type name changed. We cast to periph_module_t explicitly to
 * satisfy the function signature which hasn't been updated yet.
 *
 * @param unitIdx  0 = MCPWM0, 1 = MCPWM1
 */
void MAZPWM::_enableClock(uint8_t unitIdx)
{
    // IDF v6 ESP32-S3: MCPWM is absent from shared_periph_module_t.
    // Enable clock and release reset directly via SYSTEM registers.
    // SYSTEM_PWM0_CLK_EN = BIT(17), SYSTEM_PWM1_CLK_EN = BIT(20)
    if (unitIdx == 0) {
        SET_PERI_REG_MASK(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_PWM0_CLK_EN);
        CLEAR_PERI_REG_MASK(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_PWM0_RST);
    } else {
        SET_PERI_REG_MASK(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_PWM1_CLK_EN);
        CLEAR_PERI_REG_MASK(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_PWM1_RST);
    }
}

// ================================================================
//  _initUnit()  — configure all timers and operators on one unit
// ================================================================
/**
 * Performs the full register setup for one MCPWM peripheral:
 *
 *  For each timer i (0–2):
 *   1. Set global prescaler (clk_cfg.clk_prescale)
 *   2. Set timer prescale = 0 (no additional division on top)
 *   3. Set timer period = PWM_PERIOD_TICKS - 1
 *      (counter counts 0 … period, so period+1 steps total)
 *   4. Up-count mode (timer_cfg1.timer_mod = 1)
 *   5. Free-running  (timer_cfg1.timer_start = 2)
 *
 *  For each operator i (0–2):
 *   6. Bind operator i to timer i via operator_timer_sel
 *   7. Set comparators A and B to shadow-update at TEZ
 *      (upmethod=1 → latch at timer==0, guarantees glitch-free updates)
 *   8. Generator A action: HIGH at period start, LOW at compare-A match
 *   9. Generator B action: HIGH at period start, LOW at compare-B match
 *  10. Initialise both compare values to 0 → outputs LOW (coast)
 *
 * @param dev  Pointer to the MCPWM peripheral register struct
 */
void MAZPWM::_initUnit(volatile mcpwm_dev_t* dev)
{
    // Step 1: global prescaler for this unit
    dev->clk_cfg.clk_prescale = PWM_PRESCALER;

    for (uint8_t i = 0; i < MCPWM_OPS_PER_UNIT; ++i) {

        // ── Steps 2–5: Timer configuration ───────────────────────
        dev->timer[i].timer_cfg0.timer_prescale           = 0;
        dev->timer[i].timer_cfg0.timer_period             = PWM_PERIOD_TICKS - 1;
        dev->timer[i].timer_cfg0.timer_period_upmethod    = 0; // immediate
        dev->timer[i].timer_cfg1.timer_mod                = 1; // up-count
        dev->timer[i].timer_cfg1.timer_start              = 2; // free-run

        // ── Step 6: Bind operator i to timer i ───────────────────
        // operator_timersel is a single top-level register (not per-operator).
        // Each field selects the timer for one operator independently.
        if (i == 0) dev->operator_timersel.operator0_timersel = 0;
        else if (i == 1) dev->operator_timersel.operator1_timersel = 1;
        else             dev->operator_timersel.operator2_timersel = 2;

        // ── Step 7: Comparator shadow update at TEZ ───────────────
        // upmethod=1 → new compare value latches at timer==0 only.
        // This ensures duty changes never glitch mid-cycle.
        dev->operators[i].gen_stmp_cfg.gen_a_upmethod = 1;
        dev->operators[i].gen_stmp_cfg.gen_b_upmethod = 1;

        // ── Steps 8–9: Generator action tables ───────────────────
        // Generator A (CW pin):
        //   utez (timer=0)      → HIGH  (pulse begins)
        //   utea (cnt=compare A) → LOW   (pulse ends)
        //   All other events    → no change (val=0 clears them)
        dev->operators[i].generator[0].val      = 0;
        dev->operators[i].generator[0].gen_utez = GEN_HIGH;
        dev->operators[i].generator[0].gen_utea = GEN_LOW;

        // Generator B (CCW pin): same pattern, uses compare B
        dev->operators[i].generator[1].val      = 0;
        dev->operators[i].generator[1].gen_utez = GEN_HIGH;
        dev->operators[i].generator[1].gen_uteb = GEN_LOW;

        // ── Step 10: Both compare values = 0 → coast on startup ──
        dev->operators[i].timestamp[0].gen = 0; // compare A (CW)
        dev->operators[i].timestamp[1].gen = 0; // compare B (CCW)
    }
}

// ================================================================
//  init()
// ================================================================
/**
 * Enables clocks and initialises both MCPWM units so all four
 * motor channels are ready to use.
 *
 * After this call, all outputs are LOW (coast state).
 * Call attachMotorPin() next to connect GPIOs.
 */
void MAZPWM::init()
{
    _enableClock(0);         // enable MCPWM0 clock gate via DPORT system registers
    _initUnit(&MCPWM0);      // configure MCPWM0 timers + operators

    _enableClock(1);         // enable MCPWM1 clock gate via DPORT system registers
    _initUnit(&MCPWM1);      // configure MCPWM1 timers + operators
}

// ================================================================
//  attachMotorPin()
// ================================================================
/**
 * Routes the two generator outputs for one motor to physical GPIO pins
 * via the ESP32-S3 GPIO matrix.
 *
 * @param motorIdx  Motor index 0–3
 * @param cwGpio    GPIO for CW  output (DRV8833 IN1)
 * @param ccwGpio   GPIO for CCW output (DRV8833 IN2)
 */
void MAZPWM::attachMotorPin(uint8_t motorIdx, int cwGpio, int ccwGpio)
{
    _routeGpio(cwGpio,  kMotorSignal[motorIdx][0]); // gen-A → CW  pin
    _routeGpio(ccwGpio, kMotorSignal[motorIdx][1]); // gen-B → CCW pin
}

// ================================================================
//  setMotorDuty()
// ================================================================
/**
 * Writes new compare values to the shadow registers of one motor's
 * operator. Changes apply at the start of the next PWM period (TEZ).
 *
 * PWM waveform (up-count, generator A):
 *
 *   ┌── period ──────┐
 *   │  HIGH   │ LOW  │   duty% = (cmpA / period) × 100
 *   0        cmpA  period
 *
 * @param motorIdx  Motor index 0–3
 * @param cwDuty    CW  pin duty [0.0 – 100.0 %]
 * @param ccwDuty   CCW pin duty [0.0 – 100.0 %]
 */
void MAZPWM::setMotorDuty(uint8_t motorIdx, float cwDuty, float ccwDuty)
{
    volatile mcpwm_dev_t* dev = _dev(motorIdx);
    const uint8_t op          = _opIdx(motorIdx);

    dev->operators[op].timestamp[0].gen = _dutyToTicks(cwDuty);   // gen-A / CW
    dev->operators[op].timestamp[1].gen = _dutyToTicks(ccwDuty);  // gen-B / CCW
}

// ================================================================
//  stopMotor()
// ================================================================
/**
 * Coasts one motor immediately by setting both compare values to 0.
 * Both GPIO outputs will be driven LOW for the entire PWM period.
 */
void MAZPWM::stopMotor(uint8_t motorIdx)
{
    setMotorDuty(motorIdx, 0.0f, 0.0f);
}

// ================================================================
//  stopAll()
// ================================================================
/**
 * Coasts all four motors. Safe to call at any time including
 * fault conditions — always executes without preconditions.
 */
void MAZPWM::stopAll()
{
    for (uint8_t i = 0; i < MAZ_MOTOR_COUNT; ++i)
        stopMotor(i);
}

// ================================================================
//  _dutyToTicks()  (private, static)
// ================================================================
/**
 * Converts a duty percentage to a compare-register tick count.
 *   0 %   →  0                  → output LOW  the full period (off)
 *   100 % →  PWM_PERIOD_TICKS   → output HIGH the full period (full on)
 *
 * Input is clamped to [0.0, 100.0] before conversion.
 */
uint32_t MAZPWM::_dutyToTicks(float pct)
{
    const float clamped = std::clamp(pct, 0.0f, 100.0f);
    return static_cast<uint32_t>((clamped / 100.0f) * PWM_PERIOD_TICKS);
}

// ================================================================
//  _routeGpio()  (private, static)
// ================================================================
/**
 * Connects a physical GPIO to an MCPWM output signal using the
 * ESP32-S3 IO MUX and GPIO matrix (IDF v6 struct API).
 *
 * Step 1 — IO MUX pad function:
 *   Set to PIN_FUNC_GPIO (= 1 on ESP32-S3) so the pad is driven
 *   by the GPIO matrix instead of a direct IO MUX bypass route.
 *   Uses GPIO_PIN_MUX_REG[] + PIN_FUNC_SELECT from io_mux_reg.h.
 *
 * Step 2 — GPIO matrix output function:
 *   Write the MCPWM signal index into
 *   GPIO.func_out_sel_cfg[gpio].func_sel so the matrix routes
 *   that peripheral output to this pin.
 *   (Replaces the broken GPIO_FUNC_OUT_SEL_CFG_REG macro.)
 *
 * Step 3 — Output enable:
 *   Set the OE bit so the pad output buffer is actually driven.
 *   GPIOs 0–31 → GPIO.enable_w1ts.val
 *   GPIOs 32+  → GPIO.enable1_w1ts.val
 *   (Replaces the broken GPIO_ENABLE_REG / GPIO_ENABLE1_REG macros.)
 *
 * @param gpio       Physical GPIO number
 * @param signalIdx  MCPWM signal index (from gpio_sig_map.h)
 */
void MAZPWM::_routeGpio(int gpio, uint32_t signalIdx)
{
    // Step 1: IO MUX → GPIO matrix pass-through mode
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);

    // Step 2: GPIO matrix → route MCPWM signal to this pin
    GPIO.func_out_sel_cfg[gpio].func_sel = signalIdx;

    // Step 3: enable pad output driver
    if (gpio < 32) {
        GPIO.enable_w1ts      = (1U << gpio);        // plain uint32_t
    } else {
        GPIO.enable1_w1ts.val = (1U << (gpio - 32)); // union — use .val
    }
}