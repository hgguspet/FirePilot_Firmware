#pragma once
#include <driver/rmt.h>
#include "../iesc_driver.hpp"
#include <cmath>

// Classic PWM for ESCs: 1000..2000 us pulse width, period chosen by FC (update rate)
class PwmDriver final : public IEscDriver
{
public:
    PwmDriver() = default;
    ~PwmDriver() override { end(); }

    bool begin(uint8_t pin, uint16_t rateHz) override;
    void end() override;

    // Many analog-style ESCs historically need calibration
    bool calibrate() override { return false; } // optional, left to caller/project policy

    EscCapabilities caps() const override
    {
        // No bidir telemetry; practical max rate ~490 Hz (needs >= maxPulse + idle).
        return EscCapabilities{
            /*bidirTelemetry*/ false,
            /*needsCalibrate*/ true,
            /*maxRateHz*/ 490};
    }

    // 0..1 -> 1000..2000 us pulse
    void writeNormalized(float norm01) override;

    // Sets target period via rate; clamped so period >= maxPulse + safety idle
    void setUpdateRate(uint16_t rateHz) override;

    void setMinPulseUs(uint16_t minPulseUs) { _minPulseUs = minPulseUs; }
    void setMaxPulseUs(uint16_t maxPulseUs) { _maxPulseUs = maxPulseUs; }

    void arm(bool on) override { _armed = on; }
    void beginFrame() override { /* no-op */ }
    void endFrame() override { /* no-op */ }

    bool readTelemetry(Telemetry &out) override
    {
        (void)out;
        return false;
    }

private:
    // RMT at 80 MHz APB; clk_div=80 -> 1 us per tick
    static constexpr int kClkDiv = 80;

    // Safety low time before next pulse (microseconds)
    static constexpr uint32_t kMinIdleUs = 10;

    // Map normalized [0..1] -> [kMinPulseUs..kMaxPulseUs] (inclusive)
    inline uint16_t mapNormToPulseUs(float x)
    {
        if (__builtin_isnan(x))
            x = 0.f;
        if (x < 0.f)
            x = 0.f;
        if (x > 1.f)
            x = 1.f;
        return static_cast<uint16_t>(
            _minPulseUs + lroundf(x * float(_maxPulseUs - _minPulseUs)));
    }

private:
    rmt_channel_t _ch = RMT_CHANNEL_MAX;
    uint8_t _pin = 0xFF;
    bool _armed = false;
    bool _initialized = false;

    // Protocol timing (microseconds)
    int _minPulseUs = 1000;
    int _maxPulseUs = 2000;

    // Desired frame period (start-to-start) in microseconds.
    // Default to 400 Hz (2500 us), common for PWM ESCs.
    uint32_t _periodUs = 2500;
};
