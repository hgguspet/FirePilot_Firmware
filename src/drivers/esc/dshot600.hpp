#pragma once
#include <driver/rmt.h>
#include "../iesc_driver.hpp"

// DShot600 timing with 80Mhz APB clock
class DShot600Driver final : public IEscDriver
{
public:
    DShot600Driver() = default;
    ~DShot600Driver() override { end(); }

    bool begin(uint8_t pin, uint16_t rateHz) override;
    void end() override;

    EscCapabilities caps() const override
    {
        return EscCapabilities{/*bidirTelemetry*/ true, /*needsCalibrate*/ false, /*maxRateHz*/ 60000}; // DShot600 supports bidirectional telemetry
    }

    void writeNormalized(float norm01) override;  // 0..1 -> DShot command
    void setUpdateRate(uint16_t rateHz) override; // no hard rate limit here

    void arm(bool on) override { _armed = on; }

    void beginFrame() override { /* no-op, not yet implemented */ }
    void endFrame() override { /* no-op, not yet implemented */ }

    bool readTelemetry(Telemetry &out) override
    {
        (void)out;
        return false;
    }

private:
    static constexpr int kClkDiv = 2;                 // 80MHz/2 = 40MHz → 25 ns per tick
    static constexpr int kBits = 16;                  // DShot frame (11+1+4crc)
    static constexpr int kTtot_us = 1000000 / 600000; // 1.666... us (integer math -> 1)
    static constexpr int kTtot_ticks = 67;            // 1.666us / 0.025us ≈ 67 ticks (rounded)

    // duty ratios per spec-ish
    static constexpr int kOneHigh = (int)(0.75 * kTtot_ticks);   // ≈ 50
    static constexpr int kOneLow = kTtot_ticks - kOneHigh;       // ≈ 17
    static constexpr int kZeroHigh = (int)(0.375 * kTtot_ticks); // ≈ 25
    static constexpr int kZeroLow = kTtot_ticks - kZeroHigh;     // ≈ 42

    // Map normalized [0..1] to DShot command [48..2047]
    static uint16_t mapNormToCmd(float x);

    // Build 16-bit DShot packet (throttle + telem + CRC4)
    static uint16_t buildPacket(uint16_t throttle_cmd, bool telemetry);

    // Fill RMT items for this packet
    void buildItems(uint16_t packet, rmt_item32_t (&items)[kBits]) const;

private:
    rmt_channel_t _ch = RMT_CHANNEL_MAX;
    uint8_t _pin = -1; // GPIO pin number
    bool _armed = false;
    bool _initialized = false;
};