#include "one_shot_125.hpp"
#include <algorithm>
#include <cmath>
#include "../rmt/rmt_allocator.hpp"

bool OneShot125Driver::begin(uint8_t pin, uint16_t rateHz)
{
    if (_initialized)
        return true;

    // ---- allocate a channel ----
    rmt_channel_t ch = RMT_CHANNEL_MAX;
    if (!rmtalloc::alloc(ch))
        return false;

    // ---- configure RMT TX: 1 us ticks ----
    rmt_config_t cfg{};
    cfg.rmt_mode = RMT_MODE_TX;
    cfg.channel = ch;
    cfg.gpio_num = static_cast<gpio_num_t>(pin);
    cfg.mem_block_num = 1;
    cfg.clk_div = kClkDiv; // 80 MHz / 80 = 1 us per tick
    cfg.tx_config.loop_en = false;
    cfg.tx_config.carrier_en = false;
    cfg.tx_config.idle_output_en = true; // keep line low when idle
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

    // apply desired rate (if provided)
    setUpdateRate(rateHz);
    writeNormalized(_zeroThrottleValue); // set initial pulse
    return true;
}

void OneShot125Driver::end()
{
    if (!_initialized)
        return;

    rmt_driver_uninstall(_ch);
    rmtalloc::free(_ch);

    _ch = RMT_CHANNEL_MAX;
    _pin = 0xFF;
    _initialized = false;
    _armed = false;
    _period_us = 500;
}

void OneShot125Driver::setUpdateRate(uint16_t rateHz)
{
    if (rateHz == 0)
        return; // keep existing

    // period_us = 1e6 / rateHz, but ensure it's not shorter than a max pulse + a tiny idle
    uint32_t period = static_cast<uint32_t>(1000000UL / rateHz);
    constexpr uint32_t kMinIdleUs = 10; // ensure a short low before next pulse
    uint32_t min_period = static_cast<uint32_t>(kMaxPulseUs + kMinIdleUs);

    if (period < min_period)
        period = min_period;

    _period_us = period;
}

void OneShot125Driver::writeNormalized(float norm01)
{
    if (!_initialized || !_armed)
        return;

    const uint16_t high_us = mapNormToPulseUs(norm01);

    // Ensure we always leave some low time before next call
    uint32_t low_us = (_period_us > high_us) ? (_period_us - high_us) : 10u;
    if (low_us == 0)
        low_us = 10;

    // One RMT item encodes: HIGH for high_us, then LOW for low_us
    rmt_item32_t item{};
    item.level0 = 1;
    item.duration0 = high_us; // ticks are 1 us
    item.level1 = 0;
    item.duration1 = static_cast<uint16_t>(std::min<uint32_t>(low_us, 0x7FFF)); // duration is 15-bit

    // Blocking send to keep a stable rate controlled by caller cadence
    rmt_write_items(_ch, &item, 1, /*wait_tx_done=*/true);
    // Optionally: rmt_wait_tx_done(_ch, portMAX_DELAY);
}
