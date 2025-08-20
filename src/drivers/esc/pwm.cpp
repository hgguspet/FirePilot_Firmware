#include "pwm.hpp"
#include <algorithm>
#include <cmath>
#include "../rmt/rmt_allocator.hpp"

bool PwmDriver::begin(uint8_t pin, uint16_t rateHz)
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
    cfg.tx_config.idle_output_en = true; // hold line low when idle
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

    setUpdateRate(rateHz); // applies caller’s rate if non-zero
    return true;
}

void PwmDriver::end()
{
    if (!_initialized)
        return;

    rmt_driver_uninstall(_ch);
    rmtalloc::free(_ch);

    _ch = RMT_CHANNEL_MAX;
    _pin = 0xFF;
    _initialized = false;
    _armed = false;
    _periodUs = 2500; // back to default ~400 Hz
}

void PwmDriver::setUpdateRate(uint16_t rateHz)
{
    if (rateHz == 0)
        return; // keep existing

    // Ensure a short LOW after the pulse.
    uint32_t period = static_cast<uint32_t>(1000000UL / rateHz);
    const uint32_t min_period = static_cast<uint32_t>(_maxPulseUs + kMinIdleUs);
    if (period < min_period)
        period = min_period;

    _periodUs = period;
}

void PwmDriver::writeNormalized(float norm01)
{
    if (!_initialized || !_armed)
        return;

    const uint16_t high_us = mapNormToPulseUs(norm01);

    // Ensure some low time before the next frame
    uint32_t low_us = (_periodUs > high_us) ? (_periodUs - high_us) : kMinIdleUs;
    if (low_us == 0)
        low_us = kMinIdleUs;

    // Single RMT item: HIGH then LOW
    rmt_item32_t item{};
    item.level0 = 1;
    item.duration0 = high_us; // 1 us ticks
    item.level1 = 0;
    item.duration1 = static_cast<uint16_t>(std::min<uint32_t>(low_us, 0x7FFF)); // 15-bit field

    // Blocking send so caller cadence defines the update rate.
    rmt_write_items(_ch, &item, 1, /*wait_tx_done=*/true);
    // Optionally: rmt_wait_tx_done(_ch, portMAX_DELAY);
}
