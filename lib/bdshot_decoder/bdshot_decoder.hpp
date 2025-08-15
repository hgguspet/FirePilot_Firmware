#pragma once
#include <stdint.h>
#include <stddef.h>

// Detect ESP32 build and include the real RMT type.
// Fall back to a tiny stand-in for host/unit tests.
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ESP32)
#include <driver/rmt.h>
#define BDSHOT_HAVE_RMT 1
#else
#define BDSHOT_HAVE_RMT 0
struct rmt_item32_t
{
    uint16_t duration0;
    uint16_t level0;
    uint16_t duration1;
    uint16_t level1;
};
#endif // BDSHOT_HAVE_RMT

namespace bdshot
{

    struct Timings
    {
        int reply_bit_ticks;
    };

    enum class Kind : uint8_t
    {
        Invalid = 0,
        ERPM,
        EDT_Temperature,
        EDT_Voltage,
        EDT_Current,
        EDT_Other,
    };

    struct Decoded
    {
        bool ok = false;
        Kind kind = Kind::Invalid;
        uint16_t value16 = 0;
        uint16_t payload12 = 0;
        uint8_t crc4 = 0;
        uint32_t erpm = 0;
        uint8_t edt_type = 0;
        uint8_t edt_value = 0;
    };

    // Host-testable path: provide alternating high/low run lengths (ticks)
    bool decode_from_runs(const uint16_t *highs, const uint16_t *lows, size_t n_items,
                          const Timings &t, Decoded &out);

#if BDSHOT_HAVE_RMT
    // Target convenience: decode directly from ESP32 RMT items
    bool decode_from_rmt(const rmt_item32_t *items, size_t nitems,
                         const Timings &t, Decoded &out);
#endif // BDSHOT_HAVE_RMT

} // namespace bdshot
