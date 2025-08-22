#pragma once
#include <driver/ledc.h>

namespace ledcalloc
{
    // Optional: explicit init (first use auto-inits too). Returns true if ready.
    bool init();

    // Allocate any free LEDC channel. Returns true and writes to `out` on success.
    bool alloc(ledc_channel_t &out);

    // Allocate within [first, last] inclusive (channel indices).
    bool alloc_range(ledc_channel_t &out, int first, int last);

    // Mark a channel free. Safe to call on invalid/already-free channels.
    void free(ledc_channel_t ch);

    // Introspection
    int count_free();
    bool is_taken(ledc_channel_t ch);

    // Optional RAII lease (auto-free on destruction if owned)
    class Lease
    {
    public:
        Lease() = default;
        explicit Lease(ledc_channel_t ch) : _ch(ch), _owned(false) {}
        ~Lease();

        bool acquire_any();
        bool acquire_range(int first, int last);

        void release();

        bool valid() const { return _ch >= 0 && _ch < LEDC_CHANNEL_MAX; }
        ledc_channel_t get() const { return _ch; }

        // Transfer ownership to caller; allocator will not auto-free afterwards.
        ledc_channel_t disown();

    private:
        ledc_channel_t _ch = LEDC_CHANNEL_MAX;
        bool _owned = false;
    };

} // namespace ledcalloc
