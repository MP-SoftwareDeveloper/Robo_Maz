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
#include "esp_rom_sys.h"              // esp_rom_delay_us()
#include "esp_private/periph_ctrl.h"  // PERIPH_RCC_ATOMIC() — dual-core-safe critical section

// ── Generator action values ──────────────────────────────────────
// Written into generator[n].utez / .utea / .uteb:
//   0 = no change | 1 = force LOW | 2 = force HIGH | 3 = toggle
static constexpr uint32_t GEN_LOW  = 1;
static constexpr uint32_t GEN_HIGH = 2;

// ── Pre-built generator.val words (single 32-bit write, no RMW race) ─
// Generator register layout (mcpwm_gen_reg_t):
//   bits [1:0] gen_utez — action at TEZ  (up-count timer=0)
//   bits [3:2] gen_utep — action at TEP  (up-count timer=period)
//   bits [5:4] gen_utea — action at TEA  (up-count timer=compare A)
//   bits [7:6] gen_uteb — action at TEB  (up-count timer=compare B)
//
// GEN_A_0PCT: gen_utez=LOW  → output forced LOW at every TEZ, stays LOW.
//             Avoids the TEZ(→HIGH)/compare=0(→LOW) same-tick collision that
//             makes 0 % randomly produce 100 % output.
// GEN_A_PWM:  gen_utez=HIGH, gen_utea=LOW → standard up-count PWM waveform.
//             compare=PWM_PERIOD_TICKS (1000) is beyond timer range 0-999,
//             so TEA never fires → output stays HIGH = 100 % duty.
static constexpr uint32_t GEN_A_0PCT = (GEN_LOW  << 0);                    // 0x01
static constexpr uint32_t GEN_A_PWM  = (GEN_HIGH << 0) | (GEN_LOW << 4);  // 0x12
static constexpr uint32_t GEN_B_0PCT = (GEN_LOW  << 0);                    // 0x01
static constexpr uint32_t GEN_B_PWM  = (GEN_HIGH << 0) | (GEN_LOW << 6);  // 0x42

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
 * ESP32-S3 uses the DPORT clock-gate system. SYSTEM_PWM0/1_CLK_EN and
 * SYSTEM_PWM0/1_RST live in PERIP_CLK_EN0 / PERIP_RST_EN0 (offset +0x18/+0x20),
 * NOT in the _EN1/_RST_EN1 registers. Verified against esp_hal_mcpwm LL:
 *   SYSTEM.perip_clk_en0.pwm0_clk_en = 1
 *   SYSTEM.perip_rst_en0.pwm0_rst = 1; SYSTEM.perip_rst_en0.pwm0_rst = 0
 *
 * @param unitIdx  0 = MCPWM0, 1 = MCPWM1
 */
void MAZPWM::_enableClock(uint8_t unitIdx)
{
    // PERIPH_RCC_ATOMIC() is required by IDF for ALL SYSTEM register access on
    // the dual-core ESP32-S3. Without it, Core 1 (APP_CPU) init races our writes
    // to SYSTEM.perip_clk_en0, randomly clearing the clock-enable bit.
    // The critical section provides both dual-core exclusion and a memory barrier,
    // so no explicit memw instructions are needed for the writes inside.
    PERIPH_RCC_ATOMIC() {
        if (unitIdx == 0) {
            SYSTEM.perip_clk_en0.pwm0_clk_en = 1;
            SYSTEM.perip_rst_en0.pwm0_rst = 1;
            SYSTEM.perip_rst_en0.pwm0_rst = 0;
        } else {
            SYSTEM.perip_clk_en0.pwm1_clk_en = 1;
            SYSTEM.perip_rst_en0.pwm1_rst = 1;
            SYSTEM.perip_rst_en0.pwm1_rst = 0;
        }
    }
    esp_rom_delay_us(5);
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
    // Force the peripheral's APB register-file clock on so all writes land.
    dev->clk.clk_en = 1;

    // Step 1: global prescaler for this unit
    dev->clk_cfg.clk_prescale = PWM_PRESCALER;

    for (uint8_t i = 0; i < MCPWM_OPS_PER_UNIT; ++i) {

        // ── Steps 2–5: Timer configuration ───────────────────────
        // Single val writes to avoid APB RMW hazard: two sequential
        // bitfield writes to the same register can have the second read
        // return a stale value, corrupting the first write's field.
        //
        // timer_cfg0 layout: [7:0]=prescale [23:8]=period [25:24]=upmethod
        //   prescale=0, period=999, upmethod=0 (immediate) → 0x0003E700
        dev->timer[i].timer_cfg0.val =
            static_cast<uint32_t>(PWM_PERIOD_TICKS - 1) << 8;

        // timer_cfg1 layout: [2:0]=timer_start (SC) [4:3]=timer_mod
        //   timer_start=2 (free-run), timer_mod=1 (up-count) → 0x0A
        // Writing both in one store guarantees timer_mod=1 is committed
        // before hardware reads timer_start and launches the counter.
        dev->timer[i].timer_cfg1.val = (1u << 3) | (2u << 0);

        // ── Step 7: Comparator shadow update — at TEZ ────────────
        // gen_stmp_cfg layout (IDF LL confirmed):
        //   bits[3:0] = compare A upmethod  (bit 0 = TEZ enable)
        //   bits[7:4] = compare B upmethod  (bit 4 = TEZ enable)
        // 0x11 → both A and B update their shadow→active at TEZ.
        // Using TEZ (not "immediate") eliminates the race where a new
        // compare value is written mid-period while the timer has already
        // passed that value, causing TEA to fire too early or not at all.
        // Compare and TEZ always fire together → first PWM cycle is clean.
        dev->operators[i].gen_stmp_cfg.val = 0x11;

        // ── Steps 8–10: Generator tables + coast on startup ─────
        // Initialise compare values to 0 and put both generators in
        // GEN_x_0PCT mode (gen_utez=LOW): at every TEZ the output goes
        // LOW and stays LOW.  This avoids the TEZ(→HIGH)/compare=0(→LOW)
        // same-tick collision that would otherwise leave the pin HIGH.
        // setMotorDuty() swaps to GEN_x_PWM (TEZ→HIGH, compare→LOW) when
        // a non-zero duty is requested.
        dev->operators[i].timestamp[0].gen = 0;
        dev->operators[i].timestamp[1].gen = 0;
        dev->operators[i].generator[0].val = GEN_A_0PCT;   // CW  pin LOW
        dev->operators[i].generator[1].val = GEN_B_0PCT;   // CCW pin LOW
    }

    // Flush store buffer so all timer/generator writes above have
    // committed to the peripheral before we bind operators to timers.
    __asm__ volatile("memw" ::: "memory");

    // ── Step 6: Bind op0→timer0, op1→timer1, op2→timer2 ─────────
    // Single val write — avoids three RMW writes to the same register.
    // Layout: bits[1:0]=op0sel, bits[3:2]=op1sel, bits[5:4]=op2sel
    dev->operator_timersel.val = (2u << 4) | (1u << 2) | (0u << 0);  // 0x24
    __asm__ volatile("memw" ::: "memory");
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
 * Sets duty cycle for both direction pins of one motor.
 *
 * Writes the generator action table directly (single 32-bit val store).
 * 0 % → GEN_x_0PCT: gen_utez=LOW keeps the pin LOW every period.
 * 1–100 % → GEN_x_PWM: gen_utez=HIGH / compare→LOW normal waveform.
 *   At 100 % the compare equals PWM_PERIOD_TICKS (1000), which is one
 *   tick beyond the timer range (0–999), so the compare never fires and
 *   the output stays HIGH the full period.
 *
 * @param motorIdx  Motor index 0–3
 * @param cwDuty    CW  pin duty [0.0 – 100.0 %]
 * @param ccwDuty   CCW pin duty [0.0 – 100.0 %]
 */
void MAZPWM::setMotorDuty(uint8_t motorIdx, float cwDuty, float ccwDuty)
{
    volatile mcpwm_dev_t* dev  = _dev(motorIdx);
    const uint8_t  op          = _opIdx(motorIdx);
    const uint32_t cwTicks     = _dutyToTicks(cwDuty);
    const uint32_t ccwTicks    = _dutyToTicks(ccwDuty);

    // gen-A (CW pin)
    // Write compare BEFORE switching to PWM table so the comparator
    // never sees a stale value when the event table is enabled.
    if (cwTicks == 0) {
        dev->operators[op].generator[0].val = GEN_A_0PCT;
    } else {
        dev->operators[op].timestamp[0].gen = cwTicks;
        dev->operators[op].generator[0].val = GEN_A_PWM;
    }

    // gen-B (CCW pin)
    if (ccwTicks == 0) {
        dev->operators[op].generator[1].val = GEN_B_0PCT;
    } else {
        dev->operators[op].timestamp[1].gen = ccwTicks;
        dev->operators[op].generator[1].val = GEN_B_PWM;
    }
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

    // Step 2: GPIO matrix → route MCPWM signal to this pin.
    // Full val write (not bitfield RMW) so oen_sel and inv_sel are
    // explicitly 0: output-enable is controlled by enable_w1ts below,
    // not by the peripheral's own OE signal.
    GPIO.func_out_sel_cfg[gpio].val = signalIdx;

    // Step 3: enable pad output driver
    if (gpio < 32) {
        GPIO.enable_w1ts      = (1U << gpio);        // plain uint32_t
    } else {
        GPIO.enable1_w1ts.val = (1U << (gpio - 32)); // union — use .val
    }
}