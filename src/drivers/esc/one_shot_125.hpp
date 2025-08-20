#pragma once
#include <driver/rmt.h>
#include "../iesc_driver.hpp"
#include <cmath>

// OneShot125: 125..250 us pulse width, period chosen by FC (update rate)
class OneShot125Driver final : public IEscDriver
{
public:
    OneShot125Driver() = default;
    ~OneShot125Driver() override { end(); }

    bool begin(uint8_t pin, uint16_t rateHz) override;
    void end() override;
    bool calibrate() override { return false; } // TODO: implement

    EscCapabilities caps() const override
    {
        // No bidir telemetry, analog-style and many ESCs historically need calibration.
        // Max useful rate ~2 kHz (500 us period)
        return EscCapabilities{
            /*bidirTelemetry*/ false,
            /*needsCalibrate*/ true,
            /*maxRateHz*/ 2000};
    }

    void writeNormalized(float norm01) override; // 0..1 -> 125..250 us pulse

    void setZeroThrottleValue(float norm01) override { _zeroThrottleValue = norm01; }

    void setUpdateRate(uint16_t rateHz) override; // sets target period

    void arm(bool on) override { _armed = on; }
    void beginFrame() override { /* no-op */ }
    void endFrame() override { /* no-op */ }

    bool readTelemetry(Telemetry &out) override
    {
        (void)out;
        return false;
    }

private:
    // RMT at 80 MHz APB, clk_div=80 -> 1 us per tick (fits OS125 nicely)
    static constexpr int kClkDiv = 80;

    // Protocol timing (microseconds)
    static constexpr int kMinPulseUs = 125;
    static constexpr int kMaxPulseUs = 250;

    // Map normalized [0..1] to pulse width [125..250] us
    static inline uint16_t mapNormToPulseUs(float x)
    {
        if (__builtin_isnan(x))
            x = 0.f;
        if (x < 0.f)
            x = 0.f;
        if (x > 1.f)
            x = 1.f;
        // inclusive range
        return static_cast<uint16_t>(kMinPulseUs + lroundf(x * float(kMaxPulseUs - kMinPulseUs)));
    }

private:
    float _zeroThrottleValue = 0.0f;

    rmt_channel_t _ch = RMT_CHANNEL_MAX;
    uint8_t _pin = 0xFF;
    bool _armed = false;
    bool _initialized = false;

    // Desired frame period in microseconds (time from start of pulse N to start of pulse N+1)
    // Default to 2 kHz (500 us), which is common for OneShot125.
    uint32_t _period_us = 500;
};
