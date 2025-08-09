#pragma once
#include <driver/rmt.h>

namespace rmtalloc
{

    // Initialize is optional—first use will auto-init. Returns true if ready.
    bool init();

    // Allocate any free channel. Returns true on success and writes to `out`.
    bool alloc(rmt_channel_t &out);

    // Allocate within [first, last] (inclusive). Handy if you want to keep some channels reserved.
    bool alloc_range(rmt_channel_t &out, int first, int last);

    // Mark a channel as free. Safe to call on an already-free/invalid channel.
    void free(rmt_channel_t ch);

    // Introspection
    int count_free();
    bool is_taken(rmt_channel_t ch);

    // Optional RAII lease helper (auto-free in destructor)
    class Lease
    {
    public:
        Lease() = default;
        explicit Lease(rmt_channel_t ch) : _ch(ch), _owned(false) {}
        ~Lease();

        // Acquire any channel or a range
        bool acquire_any();
        bool acquire_range(int first, int last);

        // Release if owned
        void release();

        // Access
        bool valid() const { return _ch >= 0 && _ch < RMT_CHANNEL_MAX; }
        rmt_channel_t get() const { return _ch; }

        // Disown so caller takes responsibility (prevents auto-free)
        rmt_channel_t disown();

    private:
        rmt_channel_t _ch = RMT_CHANNEL_MAX;
        bool _owned = false;
    };

} // namespace rmtalloc
