#include "bdshot_decoder.hpp"

#if !BDSHOT_HAVE_RMT
// Nothing extra needed: we already declared a minimal rmt_item32_t in the header
#endif

#ifndef ESP_PLATFORM
struct rmt_item32_t
{
    uint16_t duration0;
    uint16_t level0;
    uint16_t duration1;
    uint16_t level1;
};
#endif

namespace bdshot
{

    static inline uint32_t round_div(int x, int d) { return (x + d / 2) / d; }
    static inline uint8_t crc4_payload12(uint16_t p12)
    {
        uint8_t c = 0;
        for (int i = 0; i < 3; ++i)
        {
            c ^= (p12 & 0xF);
            p12 >>= 4;
        }
        return c & 0xF;
    }

    // 5b GCR → 4b LUT (invalid = 0xFF). Matches Betaflight docs.
    static constexpr uint8_t GCR5_TO_4[32] = {
        /*00-07*/ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        /*08-0F*/ 0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F,
        /*10-17*/ 0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07,
        /*18-1F*/ 0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF};

    static bool nrzi_gcr_decode(const uint16_t *highs, const uint16_t *lows, size_t n_items,
                                int reply_bit_ticks, uint16_t &out16)
    {
        // 1) Build 21 NRZI bits by run-length expanding highs/lows at reply_bit_ticks
        uint32_t nrzi = 0;
        uint32_t nbits = 0;
        int curr = 0; // prepend a 0 before NRZI stream

        for (size_t i = 0; i < n_items && nbits < 21; ++i)
        {
            uint32_t b0 = round_div(highs[i], reply_bit_ticks);
            if (b0 < 1)
                b0 = 1;
            for (uint32_t b = 0; b < b0 && nbits < 21; ++b)
            {
                nrzi = (nrzi << 1) | curr;
                ++nbits;
            }
            curr ^= 1;

            uint32_t b1 = round_div(lows[i], reply_bit_ticks);
            if (b1 < 1)
                b1 = 1;
            for (uint32_t b = 0; b < b1 && nbits < 21; ++b)
            {
                nrzi = (nrzi << 1) | curr;
                ++nbits;
            }
            curr ^= 1;
        }
        if (nbits < 21)
            return false;

        // 2) NRZI → GCR: g[i] = nrzi[i] XOR nrzi[i-1]; drop first bit → 20-bit GCR
        const uint32_t x = nrzi ^ (nrzi >> 1);
        const uint32_t gcr20 = x & ((1u << 20) - 1);

        // 3) 20 GCR → 16 data (4×5b to 4×4b)
        const uint8_t n0 = (gcr20 >> 15) & 0x1F;
        const uint8_t n1 = (gcr20 >> 10) & 0x1F;
        const uint8_t n2 = (gcr20 >> 5) & 0x1F;
        const uint8_t n3 = (gcr20 >> 0) & 0x1F;
        const uint8_t a = GCR5_TO_4[n0], b = GCR5_TO_4[n1], c = GCR5_TO_4[n2], d = GCR5_TO_4[n3];
        if ((a | b | c | d) > 0x0F)
            return false;

        out16 = (uint16_t)((a << 12) | (b << 8) | (c << 4) | d);
        return true;
    }

    bool decode_from_runs(const uint16_t *highs, const uint16_t *lows, size_t n_items,
                          const Timings &t, Decoded &out)
    {
        out = Decoded{};
        uint16_t v16;
        if (!nrzi_gcr_decode(highs, lows, n_items, t.reply_bit_ticks, v16))
            return false;

        const uint16_t payload = v16 >> 4;
        const uint8_t crc = v16 & 0xF;
        if (crc4_payload12(payload) != crc)
            return false;

        out.ok = true;
        out.value16 = v16;
        out.payload12 = payload;
        out.crc4 = crc;

        // EDT vs eRPM: if MSB of 9-bit mantissa is 0 → EDT; else eRPM
        const bool is_edt = ((payload & 0x100) == 0);

        if (is_edt)
        {
            const uint8_t type = (payload >> 8) & 0x0F;
            const uint8_t value = payload & 0xFF;
            out.edt_type = type;
            out.edt_value = value;
            switch (type)
            {
            case 0x02:
                out.kind = Kind::EDT_Temperature;
                break;
            case 0x04:
                out.kind = Kind::EDT_Voltage;
                break;
            case 0x06:
                out.kind = Kind::EDT_Current;
                break;
            default:
                out.kind = Kind::EDT_Other;
                break;
            }
        }
        else
        {
            const uint8_t exp = (payload >> 9) & 0x7;
            const uint16_t base = payload & 0x1FF;
            const uint32_t period_us = (uint32_t)((base ? base : 1) << exp);
            out.kind = Kind::ERPM;
            out.erpm = period_us ? (60000000u / period_us) : 0; // eRPM
        }
        return true;
    }
#if BDSHOT_HAVE_RMT
    bool decode_from_rmt(const rmt_item32_t *items, size_t nitems,
                         const Timings &t, Decoded &out)
    {
        if (!items || nitems == 0)
            return false;
        constexpr size_t MAX = 64;
        if (nitems > MAX)
            nitems = MAX;

        uint16_t highs[MAX], lows[MAX];
        for (size_t i = 0; i < nitems; ++i)
        {
            highs[i] = items[i].duration0;
            lows[i] = items[i].duration1;
        }
        return decode_from_runs(highs, lows, nitems, t, out);
    }
#endif // BDSHOT_HAVE_RMT
} // namespace bdshot
