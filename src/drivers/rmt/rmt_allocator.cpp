#include "rmt_allocator.hpp"

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
}

namespace
{
    // One mutex + bitmap for all users of the allocator.
    StaticSemaphore_t s_mtx_buf;
    SemaphoreHandle_t s_mtx = nullptr;
    bool s_inited = false;

    // Channel ownership table
    bool s_taken[RMT_CHANNEL_MAX] = {false};

    inline void lock() { xSemaphoreTake(s_mtx, portMAX_DELAY); }
    inline void unlock() { xSemaphoreGive(s_mtx); }

    void lazy_init()
    {
        if (s_inited)
            return;
        s_mtx = xSemaphoreCreateMutexStatic(&s_mtx_buf);
        configASSERT(s_mtx);
        // s_taken[] already zeroed by static storage
        s_inited = true;
    }

    bool alloc_impl(rmt_channel_t &out, int first, int last)
    {
        lazy_init();
        if (first < 0)
            first = 0;
        if (last >= RMT_CHANNEL_MAX)
            last = RMT_CHANNEL_MAX - 1;
        if (first > last)
            return false;

        lock();
        bool ok = false;
        for (int i = first; i <= last; ++i)
        {
            if (!s_taken[i])
            {
                s_taken[i] = true;
                out = static_cast<rmt_channel_t>(i);
                ok = true;
                break;
            }
        }
        unlock();
        return ok;
    }
}

namespace rmtalloc
{

    bool init()
    {
        lazy_init();
        return s_inited && (s_mtx != nullptr);
    }

    bool alloc(rmt_channel_t &out)
    {
        return alloc_impl(out, 0, RMT_CHANNEL_MAX - 1);
    }

    bool alloc_range(rmt_channel_t &out, int first, int last)
    {
        return alloc_impl(out, first, last);
    }

    void free(rmt_channel_t ch)
    {
        lazy_init();
        if (!(ch >= 0 && ch < RMT_CHANNEL_MAX))
            return;
        lock();
        s_taken[ch] = false;
        unlock();
    }

    int count_free()
    {
        lazy_init();
        lock();
        int cnt = 0;
        for (int i = 0; i < RMT_CHANNEL_MAX; ++i)
            if (!s_taken[i])
                ++cnt;
        unlock();
        return cnt;
    }

    bool is_taken(rmt_channel_t ch)
    {
        lazy_init();
        if (!(ch >= 0 && ch < RMT_CHANNEL_MAX))
            return false;
        lock();
        bool v = s_taken[ch];
        unlock();
        return v;
    }

    // -------- RAII Lease --------
    rmtalloc::Lease::~Lease()
    {
        release();
    }

    bool rmtalloc::Lease::acquire_any()
    {
        if (_owned)
            return true;
        rmt_channel_t ch;
        if (!alloc(ch))
            return false;
        _ch = ch;
        _owned = true;
        return true;
    }

    bool rmtalloc::Lease::acquire_range(int first, int last)
    {
        if (_owned)
            return true;
        rmt_channel_t ch;
        if (!alloc_range(ch, first, last))
            return false;
        _ch = ch;
        _owned = true;
        return true;
    }

    void rmtalloc::Lease::release()
    {
        if (_owned && valid())
        {
            rmtalloc::free(_ch);
        }
        _owned = false;
        _ch = RMT_CHANNEL_MAX;
    }

    rmt_channel_t rmtalloc::Lease::disown()
    {
        _owned = false;
        rmt_channel_t old = _ch;
        _ch = RMT_CHANNEL_MAX;
        return old;
    }

} // namespace rmtalloc
