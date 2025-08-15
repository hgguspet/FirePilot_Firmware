#pragma once
#include <driver/rmt.h>
#include "../iesc_driver.hpp"

class DShot600Driver final : public IEscDriver
{
public:
    DShot600Driver() = default;
    ~DShot600Driver() override { end(); }

    bool begin(uint8_t pin, uint16_t rateHz) override;
    void end() override;

    EscCapabilities caps() const override
    {
        // DShot command/update rate (not bitrate!) is typically 1–4 kHz.
        return EscCapabilities{
            /*.bidirTelemetry=*/true,
            /*.needsCalibrate=*/false,
            /*.maxRateHz=*/4000};
    }

    void writeNormalized(float norm01) override;  // 0..1 -> DShot throttle
    void setUpdateRate(uint16_t rateHz) override; // command/update rate
    void arm(bool on) override { _armed = on; }
    void beginFrame() override {}
    void endFrame() override {}

    bool readTelemetry(Telemetry &out) override; // fills .valid, .rpm, .temperatureC, .millivolts, .milliamps

    // DShot special command 0..47 (e.g., 20=fwd, 21=rev, 23=save on BLHeli_32)
    bool sendSpecial(uint16_t code) override;

    void setMotorPolePairs(uint8_t pp) { _pole_pairs = pp ? pp : 7; } // default 14 pole motors -> 7 PP

private:
    // ---- DShot600 timings (TX) ----
    static constexpr int kClkDiv = 2;      // 80MHz/2 = 40MHz → 25 ns per tick
    static constexpr int kBits = 16;       // 11+1+4
    static constexpr int kTtot_ticks = 67; // 1.667 us / 25 ns ≈ 66.68

    static constexpr int kOneHigh = int(0.75 * kTtot_ticks);   // ≈ 50
    static constexpr int kOneLow = kTtot_ticks - kOneHigh;     // ≈ 17
    static constexpr int kZeroHigh = int(0.375 * kTtot_ticks); // ≈ 25
    static constexpr int kZeroLow = kTtot_ticks - kZeroHigh;   // ≈ 42

    // BDShot reply bit is 5/4 faster than TX bit
    static constexpr int kReplyBitTicks = (kTtot_ticks * 4) / 5; // ≈ 53 @ 40 MHz

    uint16_t mapNormToCmd(float x); // 48..2047
    static uint16_t buildPacket(uint16_t throttle_or_cmd, bool telemetry);
    void buildItems(uint16_t packet, rmt_item32_t (&items)[kBits]) const;

private:
    // Motor info
    uint8_t _pole_pairs = 7; // 14 poles typical

    // State
    bool _armed = false;
    bool _initialized = false;

    // Pin/RMT
    uint8_t _pin = 0xFF;
    rmt_channel_t _ch = RMT_CHANNEL_MAX;    // TX
    rmt_channel_t _rx_ch = RMT_CHANNEL_MAX; // RX

    // Telemetry cadence (process every Nth frame)
    uint8_t _tlm_request_div = 32;
    uint8_t _tlm_request_ctr = 0;

    // Telemetry cache (filled by RX/decoder)
    volatile bool _tlm_valid = false;
    volatile uint16_t _last_rpm = 0; // mechanical RPM
    volatile uint8_t _last_tempC = 0;
    volatile uint16_t _last_mV = 0;
    volatile uint16_t _last_mA = 0;

    // Simple internal scheduler
    uint32_t _period_us = 500; // default 2 kHz update
    uint64_t _next_due_us = 0; // next time we’re allowed to TX
};
