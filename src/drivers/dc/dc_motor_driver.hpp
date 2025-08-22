#pragma once

#include <driver/ledc.h>
#include <Arduino.h>
#include "drivers/dc/idc_driver.hpp"
#include "drivers/ledc/ledc_allocator.hpp"

class DcMotorDriver final : public IDcDriver
{
public:
    DcMotorDriver() = default;
    ~DcMotorDriver() override { end(); }

    // ---- IDcDriver ----
    bool begin(uint8_t pwmPin, uint32_t pwmFreqHz) override;
    void end() override;

    DcCapabilities caps() const override;

    // 0..1 duty (unsigned). If a direction pin is configured, this is "forward".
    void writeNormalized(float duty01) override;

    // Optional signed command in [-1..+1] (uses direction pin if present).
    void writeSigned(float dutySigned) override;

    void setOutputLimits(float min01, float max01) override;

    void arm(bool on) override;
    void setUpdateRate(uint32_t pwmFreqHz) override;

    bool setBrake(bool on) override; // requires brake pin configured
    bool coast() override;           // high-Z/LOW output (depends on driver wiring)

    bool readTelemetry(Telemetry &out) override
    {
        (void)out;
        return false;
    }

    // ---- Optional helpers (non-virtual) ----
    // Configure optional pins (call before begin()).
    // dirPin: H-bridge direction (>=0 to enable signed control)
    // enPin:  optional enable pin for the power stage
    // brakePin: optional active-brake control (level true = brake)
    void configurePins(int dirPin = -1, int enPin = -1, int brakePin = -1);

    // Invert logical forward direction (useful if wiring is flipped).
    void setDirectionInverted(bool inv) { _dir_inverted = inv; }

    // Change LEDC resolution (bits). Call before begin(); defaults to 12 bits.
    void setResolutionBits(uint8_t bits) { _res_bits = bits; }

private:
    void applyDutyRaw_(uint32_t duty);
    void ensureAttached_();
    void updateDirFromSigned_(float dutySigned);

private:
    // LEDC allocation/state
    ledcalloc::Lease ch_; // RAII LEDC channel
    int _pin_pwm = -1;
    uint8_t _res_bits = 12; // 0..4095 at 12-bit
    uint32_t _freq_hz = 0;
    uint32_t _max_duty = 0;
    bool _initalized = false;
    bool _armed = false;

    // Optional GPIOs
    int _pin_dir = -1;
    int _pin_en = -1;
    int _pin_brake = -1;
    bool _dir_inverted = false;

    // Output limits
    float _out_min = 0.0f;
    float _out_max = 1.0f;
};
