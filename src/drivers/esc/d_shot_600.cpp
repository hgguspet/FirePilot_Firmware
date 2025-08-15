#include "d_shot_600.hpp"
#include "bdshot_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <driver/rmt.h>

#include "esp_timer.h" // esp_timer_get_time()
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "../rmt/rmt_allocator.hpp"

// ===== Zero handling: deadband + hysteresis =====
static constexpr float kZeroEnter = 0.02f; // enter ZERO when x <= 2%
static constexpr float kZeroExit = 0.04f;  // leave ZERO when x >= 4%

// Optional: extra 'digital idle' injected on our side. Prefer ESC-side Digital Idle.
static constexpr float kDigitalIdlePc = 0.00f; // 0..0.20

// For zero keepalive we do NOT use command 0 (can jitter on some stacks).
static constexpr uint16_t kThrottleMinCode = 48; // lowest throttle step

// ----- Mapping / packet helpers -----

uint16_t DShot600Driver::mapNormToCmd(float x)
{
    if (std::isnan(x) || x <= 0.f)
        return kThrottleMinCode;
    if (x >= 1.f)
        x = 1.f;

    constexpr int kMin = 48;
    constexpr int kMax = 2047;

    int idle_offset = int(std::lround(kDigitalIdlePc * float(kMax - kMin)));
    idle_offset = std::clamp(idle_offset, 0, kMax - kMin);

    int code = (kMin + idle_offset) + int(std::lround(x * float(kMax - (kMin + idle_offset))));
    code = std::clamp(code, kMin + idle_offset, kMax);
    return static_cast<uint16_t>(code);
}

uint16_t DShot600Driver::buildPacket(uint16_t throttle_or_cmd, bool telemetry)
{
    const uint16_t v = static_cast<uint16_t>(((throttle_or_cmd & 0x07FF) << 1) | (telemetry ? 1u : 0u));

    uint16_t csum = 0, tmp = v;
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

// ----- Telemetry readout -----

bool DShot600Driver::readTelemetry(Telemetry &out)
{
    if (!_tlm_valid)
        return false;
    out.valid = true;
    out.rpm = _last_rpm;
    out.temperatureC = _last_tempC;
    out.millivolts = _last_mV;
    out.milliamps = _last_mA;
    _tlm_valid = false; // consume-on-read; make sticky if you prefer
    return true;
}

// ----- Driver lifecycle -----

bool DShot600Driver::begin(uint8_t pin, uint16_t rateHz)
{
    if (_initialized)
        return true;

    // Allocate TX channel
    rmt_channel_t ch = RMT_CHANNEL_MAX;
    if (!rmtalloc::alloc(ch))
        return false;

    rmt_config_t tx{};
    tx.rmt_mode = RMT_MODE_TX;
    tx.channel = ch;
    tx.gpio_num = static_cast<gpio_num_t>(pin);
    tx.mem_block_num = 1;
    tx.clk_div = kClkDiv;
    tx.tx_config.loop_en = false;
    tx.tx_config.carrier_en = false;
    tx.tx_config.idle_output_en = true;
    tx.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    if (rmt_config(&tx) != ESP_OK ||
        rmt_set_source_clk(ch, RMT_BASECLK_APB) != ESP_OK ||
        rmt_driver_install(ch, 0, 0) != ESP_OK)
    {
        rmtalloc::free(ch);
        return false;
    }

    // Shared one-wire: open-drain + pull-up on same pin
    gpio_config_t io{};
    io.pin_bit_mask = 1ULL << pin;
    io.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    io.pull_up_en = GPIO_PULLUP_ENABLE; // also add an external pull-up (e.g., 4.7–10k)
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);

    // Allocate RX channel
    rmt_channel_t rx = RMT_CHANNEL_MAX;
    if (!rmtalloc::alloc(rx))
    {
        rmt_driver_uninstall(ch);
        rmtalloc::free(ch);
        return false;
    }

    rmt_config_t rxcfg{};
    rxcfg.rmt_mode = RMT_MODE_RX;
    rxcfg.channel = rx;
    rxcfg.gpio_num = static_cast<gpio_num_t>(pin); // same pad
    rxcfg.clk_div = kClkDiv;
    rxcfg.mem_block_num = 1;
    rxcfg.rx_config.filter_en = true;
    rxcfg.rx_config.filter_ticks_thresh = 10; // ~250 ns
    rxcfg.rx_config.idle_threshold = 200;     // ~3*Tbit @ DShot600

    if (rmt_config(&rxcfg) != ESP_OK ||
        rmt_set_source_clk(rx, RMT_BASECLK_APB) != ESP_OK ||
        rmt_driver_install(rx, 256, 0) != ESP_OK) // small RX ring buffer
    {
        rmt_driver_uninstall(ch);
        rmtalloc::free(ch);
        rmtalloc::free(rx);
        return false;
    }

    _ch = ch;
    _rx_ch = rx;
    _pin = pin;
    _initialized = true;

    if (rateHz)
        setUpdateRate(rateHz);
    _next_due_us = esp_timer_get_time();
    return true;
}

void DShot600Driver::end()
{
    if (!_initialized)
        return;

    if (_rx_ch != RMT_CHANNEL_MAX)
    {
        rmt_driver_uninstall(_rx_ch);
        rmtalloc::free(_rx_ch);
        _rx_ch = RMT_CHANNEL_MAX;
    }
    if (_ch != RMT_CHANNEL_MAX)
    {
        rmt_driver_uninstall(_ch);
        rmtalloc::free(_ch);
        _ch = RMT_CHANNEL_MAX;
    }

    _pin = 0xFF;
    _initialized = false;
    _armed = false;
    _period_us = 500;
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

// DShot special command (0..47)
bool DShot600Driver::sendSpecial(uint16_t code)
{
    if (!_initialized || !_armed)
        return false;
    if (code > 47)
        code = 47;

    const uint64_t now = esp_timer_get_time();
    if (now < _next_due_us)
        return false;
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

    static bool s_inZero = true;
    float x = std::isnan(norm01) ? 0.f : std::clamp(norm01, 0.f, 1.f);

    // zero hysteresis
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
        return;
    _next_due_us = now + _period_us;

    // cadence gate for RX processing
    const bool want_tlm = (++_tlm_request_ctr >= _tlm_request_div);
    if (want_tlm)
        _tlm_request_ctr = 0;

    const uint16_t code = s_inZero ? kThrottleMinCode : mapNormToCmd(x);
    const uint16_t pkt = buildPacket(code, /*telemetry=*/want_tlm); // harmless for BDShot
    rmt_item32_t items[kBits];
    buildItems(pkt, items);

    // TX the frame
    (void)rmt_write_items(_ch, items, kBits, /*wait_tx_done=*/true);

    if (!want_tlm || _rx_ch == RMT_CHANNEL_MAX)
        return;

    // Start RX immediately; non-blocking poll of ringbuffer
    RingbufHandle_t rb = nullptr;
    rmt_get_ringbuf_handle(_rx_ch, &rb);
    if (!rb)
        return;

    rmt_rx_start(_rx_ch, true);

    size_t nbytes = 0;
    const rmt_item32_t *rx_items = (const rmt_item32_t *)xRingbufferReceive(rb, &nbytes, 0);
    if (rx_items)
    {
        const size_t n_items = nbytes / sizeof(rmt_item32_t);
        bdshot::Timings t{.reply_bit_ticks = kReplyBitTicks};
        bdshot::Decoded d;
        if (bdshot::decode_from_rmt(rx_items, n_items, t, d) && d.ok)
        {
            switch (d.kind)
            {
            case bdshot::Kind::ERPM:
            {
                const uint32_t mech = _pole_pairs ? (d.erpm / _pole_pairs) : d.erpm;
                _last_rpm = (mech > 0xFFFFu) ? 0xFFFFu : (uint16_t)mech;
                _tlm_valid = true;
            }
            break;
            case bdshot::Kind::EDT_Temperature:
                _last_tempC = d.edt_value;
                _tlm_valid = true;
                break;
            case bdshot::Kind::EDT_Voltage:
                _last_mV = (uint16_t)d.edt_value * 250u;
                _tlm_valid = true;
                break; // 0.25 V/LSB
            case bdshot::Kind::EDT_Current:
                _last_mA = (uint16_t)d.edt_value * 1000u;
                _tlm_valid = true;
                break; // 1 A/LSB → mA
            default:
                break;
            }
        }
        vRingbufferReturnItem(rb, (void *)rx_items);
    }

    rmt_rx_stop(_rx_ch);
}
