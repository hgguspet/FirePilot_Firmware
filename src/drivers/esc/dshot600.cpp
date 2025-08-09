// dshot600.cpp
#include "dshot600.hpp"
#include <cstring>
#include <algorithm>
#include <cmath>

#include "../rmt/rmt_allocator.hpp"

uint16_t DShot600Driver::mapNormToCmd(float x)
{
    // Clamp to [0,1]
    if (std::isnan(x))
        x = 0.f;
    x = std::min(1.f, std::max(0.f, x));

    // DShot throttle usable range: 48..2047 (48 reserved for lowest throttle step)
    // Map x=0 → 0 (disarm/command) if you want true stop at 0,
    // or → 48 for “always throttle”; here we choose 48..2047.
    const int min_cmd = 48;
    const int max_cmd = 2047;
    int cmd = static_cast<int>(std::lround(min_cmd + x * (max_cmd - min_cmd)));
    cmd = std::max(min_cmd, std::min(max_cmd, cmd));
    return static_cast<uint16_t>(cmd);
}

uint16_t DShot600Driver::buildPacket(uint16_t throttle_cmd, bool telemetry)
{
    // 11-bit throttle, 1-bit telemetry, 4-bit CRC (XOR of nibbles)
    uint16_t value = static_cast<uint16_t>((throttle_cmd & 0x07FF) << 5);
    if (telemetry)
        value |= (1u << 4);

    uint16_t csum = 0;
    uint16_t csum_data = value;
    for (int i = 0; i < 3; ++i)
    {
        csum ^= (csum_data & 0xF);
        csum_data >>= 4;
    }
    csum &= 0xF;

    return static_cast<uint16_t>((value << 4) | csum);
}

void DShot600Driver::buildItems(uint16_t packet, rmt_item32_t (&items)[kBits]) const
{
    // MSB first
    for (int i = 0; i < kBits; ++i)
    {
        bool bit = (packet & (1u << (15 - i))) != 0;
        items[i].level0 = 1;
        items[i].duration0 = bit ? kOneHigh : kZeroHigh;
        items[i].level1 = 0;
        items[i].duration1 = bit ? kOneLow : kZeroLow;
    }
}

bool DShot600Driver::begin(uint8_t pin, uint16_t /*rateHz*/)
{
    if (_initialized)
        return true;

    // ---- allocate a channel ----
    rmt_channel_t ch = RMT_CHANNEL_MAX;
    if (!rmtalloc::alloc(ch))
    {
        // No free channels
        return false;
    }

    // ---- configure RMT TX ----
    rmt_config_t cfg{};
    cfg.rmt_mode = RMT_MODE_TX;
    cfg.channel = ch;
    cfg.gpio_num = static_cast<gpio_num_t>(pin);
    cfg.mem_block_num = 1; // 1 block is plenty for 16 items
    cfg.clk_div = kClkDiv; // 80MHz / 2 = 40MHz tick (25 ns)
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
    return true;
}

void DShot600Driver::end()
{
    if (!_initialized)
        return;

    rmt_driver_uninstall(_ch);
    rmtalloc::free(_ch);
    _ch = RMT_CHANNEL_MAX;
    _pin = static_cast<uint8_t>(-1);
    _initialized = false;
    _armed = false;
}

void DShot600Driver::setUpdateRate(uint16_t /*rateHz*/)
{
    // DShot bit timing is fixed; update rate is under your control by how often you call writeNormalized().
    // No-op here.
}

void DShot600Driver::writeNormalized(float norm01)
{
    if (!_initialized || !_armed)
        return;

    uint16_t cmd = mapNormToCmd(norm01);
    uint16_t pkt = buildPacket(cmd, /*telemetry=*/false);

    rmt_item32_t items[kBits];
    buildItems(pkt, items);

    // blocking = true to ensure waveform fully sent before return
    rmt_write_items(_ch, items, kBits, /*wait_tx_done=*/true);
    // Optionally: rmt_wait_tx_done(_ch, portMAX_DELAY);
}
