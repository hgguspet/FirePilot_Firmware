#pragma once
#include <driver/rmt.h>
#include "iesc_driver.hpp"

class DShot600Driver final : public IEscDriver
{
public:
    DShot600Driver() = default;
    ~DShot600Driver() override { end(); }

    bool begin(uint8_t pin, uint16_t rateHz) override;
    void end() override;
    bool calibrate() override { return true; } // calibration not necessary

    EscCapabilities caps() const override
    {
        // DShot command/update rate (not bitrate!) is typically 1–4 kHz.
        return EscCapabilities{
            /*.bidirTelemetry=*/true,
            /*.needsCalibrate=*/false,
            /*.maxRateHz=*/4000};
    }

    void writeNormalized(float norm01) override; // 0..1 -> DShot throttle
    void setZeroThrottleValue(float norm01) override { _zeroThrottleValue = norm01; };
    void setUpdateRate(uint16_t rateHz) override; // now enforced internally

    void arm(bool on) override { _armed = on; }
    void beginFrame() override {}
    void endFrame() override {}

    bool readTelemetry(Telemetry &out) override
    {
        (void)out;
        return false;
    }

    // ---- DShot-only convenience (optional) ----
    // Send special command 0..47 (e.g., 20=fwd, 21=rev, 23=save on BLHeli_32)
    bool sendSpecial(uint16_t code) override;

private:
    static constexpr int kClkDiv = 2;      // 80MHz/2 = 40MHz → 25 ns per tick
    static constexpr int kBits = 16;       // 11+1+4
    static constexpr int kTtot_ticks = 67; // 1.667 us / 25 ns ≈ 66.68

    static constexpr int kOneHigh = int(0.75 * kTtot_ticks);   // ≈ 50
    static constexpr int kOneLow = kTtot_ticks - kOneHigh;     // ≈ 17
    static constexpr int kZeroHigh = int(0.375 * kTtot_ticks); // ≈ 25
    static constexpr int kZeroLow = kTtot_ticks - kZeroHigh;   // ≈ 42

    uint16_t mapNormToCmd(float x); // 48..2047
    static uint16_t buildPacket(uint16_t throttle_or_cmd, bool telemetry);
    void buildItems(uint16_t packet, rmt_item32_t (&items)[kBits]) const;

private:
    float _zeroThrottleValue = 0.0f;

    rmt_channel_t _ch = RMT_CHANNEL_MAX;
    uint8_t _pin = 0xFF;
    bool _armed = false;
    bool _initialized = false;
    bool _throttle_above_idle = false;

    // simple internal scheduler
    uint32_t _period_us = 500; // default 2 kHz
    uint64_t _next_due_us = 0; // next time we're allowed to TX
};
