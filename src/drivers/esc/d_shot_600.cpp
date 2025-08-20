// dshot600.cpp
#include "d_shot_600.hpp"

#include <algorithm>
#include <cmath>

#include "esp_timer.h" // esp_timer_get_time()
#include "../rmt/rmt_allocator.hpp"

// ===== Zero handling: deadband + hysteresis =====
// Enter ZERO when x <= 2%, leave ZERO only when x >= 4%.
static constexpr float kZeroEnter = 0.02f; // 2%
static constexpr float kZeroExit = 0.04f;  // 4%

// Optional: extra 'digital idle' (fraction of throttle range) injected on our side.
// Leave 0.00f and prefer ESC-side Digital Idle % in BLHeli.
static constexpr float kDigitalIdlePc = 0.00f; // 0..0.20

// For zero keepalive we deliberately DO NOT use DShot command 0,
// because it's not implemented on BLHeli and can jitter.
// We keep the link alive with the minimum throttle code instead.
static constexpr uint16_t kThrottleMinCode = 48; // lowest throttle step

// ----- Mapping / packet helpers -----

// Map normalized (0..1] -> [48..2047] linearly (we handle "zero" outside)
uint16_t DShot600Driver::mapNormToCmd(float x)
{
    if (std::isnan(x) || x <= 0.f)
        return kThrottleMinCode; // won't be used for zero, but safe
    if (x >= 1.f)
        x = 1.f;

    constexpr int kMin = 48;
    constexpr int kMax = 2047;

    int idle_offset = int(std::lround(kDigitalIdlePc * float(kMax - kMin)));
    if (idle_offset < 0)
        idle_offset = 0;
    if (kMin + idle_offset > kMax)
        idle_offset = (kMax - kMin);

    int code = (kMin + idle_offset) + int(std::lround(x * float(kMax - (kMin + idle_offset))));
    if (code < kMin + idle_offset)
        code = kMin + idle_offset;
    if (code > kMax)
        code = kMax;

    return static_cast<uint16_t>(code);
}

// Canonical DShot build:
// v = ((value & 0x07FF) << 1) | telemetry;  // 12-bit payload
// CRC = XOR of the three nibbles of v
// packet = (v << 4) | CRC
uint16_t DShot600Driver::buildPacket(uint16_t throttle_or_cmd, bool telemetry)
{
    const uint16_t v = static_cast<uint16_t>(((throttle_or_cmd & 0x07FF) << 1) |
                                             (telemetry ? 1u : 0u));

    uint16_t csum = 0;
    uint16_t tmp = v;
    for (int i = 0; i < 3; ++i)
    {
        csum ^= (tmp & 0xF);
        tmp >>= 4;
    }
    csum &= 0xF;

    return static_cast<uint16_t>((v << 4) | csum);
}

void DShot600Driver::buildItems(uint16_t packet, rmt_item32_t (&items)[kBits]) const
{
    // MSB first (bit 15 -> bit 0)
    for (int i = 0; i < kBits; ++i)
    {
        const bool bit = (packet & (1u << (15 - i))) != 0;
        items[i].level0 = 1;
        items[i].duration0 = bit ? kOneHigh : kZeroHigh;
        items[i].level1 = 0;
        items[i].duration1 = bit ? kOneLow : kZeroLow;
    }
}

// ----- Driver lifecycle -----

bool DShot600Driver::begin(uint8_t pin, uint16_t rateHz)
{
    if (_initialized)
        return true;

    // Allocate RMT channel
    rmt_channel_t ch = RMT_CHANNEL_MAX;
    if (!rmtalloc::alloc(ch))
    {
        return false;
    }

    // Configure RMT TX
    rmt_config_t cfg{};
    cfg.rmt_mode = RMT_MODE_TX;
    cfg.channel = ch;
    cfg.gpio_num = static_cast<gpio_num_t>(pin);
    cfg.mem_block_num = 1; // 16 items fit easily
    cfg.clk_div = kClkDiv; // 80 MHz / 2 = 40 MHz => 25 ns ticks
    cfg.tx_config.loop_en = false;
    cfg.tx_config.carrier_en = false;
    cfg.tx_config.idle_output_en = true;
    cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    if (rmt_config(&cfg) != ESP_OK ||
        rmt_set_source_clk(ch, RMT_BASECLK_APB) != ESP_OK ||
        rmt_driver_install(ch, 0, 0) != ESP_OK)
    {
        rmtalloc::free(ch);
        return false;
    }

    _ch = ch;
    _pin = pin;
    _initialized = true;

    if (rateHz)
        setUpdateRate(rateHz);           // Hz = command rate, not bitrate
    _next_due_us = esp_timer_get_time(); // allow immediate first send

    writeNormalized(_zeroThrottleValue); // set initial pulse
    return true;
}

void DShot600Driver::end()
{
    if (!_initialized)
        return;

    rmt_driver_uninstall(_ch);
    rmtalloc::free(_ch);

    _ch = RMT_CHANNEL_MAX;
    _pin = 0xFF;
    _initialized = false;
    _armed = false;

    _period_us = 500; // reset to default 2 kHz
    _next_due_us = 0;
}

// ----- Rate / command control -----

void DShot600Driver::setUpdateRate(uint16_t rateHz)
{
    if (rateHz == 0)
        return;
    uint32_t p = 1000000UL / rateHz; // e.g. 2000 -> 500 us
    if (p < 100)
        p = 100; // clamp to something sane
    _period_us = p;
}

// DShot special command (0..47), e.g. 20=fwd, 21=rev, 23=save (BLHeli_32)
bool DShot600Driver::sendSpecial(uint16_t code)
{
    if (!_initialized || !_armed)
        return false;
    if (code > 47)
        code = 47;

    const uint64_t now = esp_timer_get_time();
    if (now < _next_due_us)
        return false; // simple rate limiting
    _next_due_us = now + _period_us;

    const uint16_t pkt = buildPacket(code, /*telemetry=*/false);

    rmt_item32_t items[kBits];
    buildItems(pkt, items);

    return rmt_write_items(_ch, items, kBits, /*wait_tx_done=*/true) == ESP_OK;
}

// ----- Hot path: throttle with zero hysteresis + safe keepalive -----

void DShot600Driver::writeNormalized(float norm01)
{
    if (!_initialized || !_armed)
        return;

    // Hysteresis latch (function-local, keeps header unchanged)
    static bool s_inZero = true;

    float x = norm01;
    if (std::isnan(x))
        x = 0.f;
    if (x < 0.f)
        x = 0.f;
    if (x > 1.f)
        x = 1.f;

    // Hysteresis around zero
    if (s_inZero)
    {
        if (x >= kZeroExit)
            s_inZero = false;
        else
            x = 0.f;
    }
    else
    {
        if (x <= kZeroEnter)
        {
            x = 0.f;
            s_inZero = true;
        }
    }

    const uint64_t now = esp_timer_get_time();
    if (now < _next_due_us)
        return; // rate limit to _period_us
    _next_due_us = now + _period_us;

    uint16_t code;
    if (s_inZero)
    {
        // Keep the ESC link alive without using command 0:
        // send minimum throttle code (48). With Motor Stop enabled in BLHeli,
        // this will NOT spin. Without Motor Stop it will idle slowly (by design).
        code = kThrottleMinCode;
    }
    else
    {
        code = mapNormToCmd(x); // 48..2047
    }

    const uint16_t pkt = buildPacket(code, /*telemetry=*/false);

    rmt_item32_t items[kBits];
    buildItems(pkt, items);

    (void)rmt_write_items(_ch, items, kBits, /*wait_tx_done=*/true);
}
