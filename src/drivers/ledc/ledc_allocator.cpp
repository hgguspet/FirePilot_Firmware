#include "ledc_allocator.hpp"
#include "logging/logger.hpp"

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
}

namespace
{
    // Global mutex + state (static storage zero-inits s_taken[])
    StaticSemaphore_t s_mtx_buf;
    SemaphoreHandle_t s_mtx = nullptr;
    bool s_inited = false;

    bool s_taken[LEDC_CHANNEL_MAX] = {false};

    inline void lock() { xSemaphoreTake(s_mtx, portMAX_DELAY); }
    inline void unlock() { xSemaphoreGive(s_mtx); }

    void lazy_init()
    {
        if (s_inited)
            return;
        s_mtx = xSemaphoreCreateMutexStatic(&s_mtx_buf);
        configASSERT(s_mtx);
        s_inited = true;
    }

    bool alloc_impl(ledc_channel_t &out, int first, int last)
    {
        lazy_init();

        if (first < 0)
            first = 0;
        if (last >= LEDC_CHANNEL_MAX)
            last = LEDC_CHANNEL_MAX - 1;
        if (first > last)
            return false;

        lock();
        bool ok = false;
        for (int i = first; i <= last; ++i)
        {
            if (!s_taken[i])
            {
                s_taken[i] = true;
                out = static_cast<ledc_channel_t>(i);
                ok = true;
                break;
            }
        }
        unlock();
        return ok;
    }
} // namespace

namespace ledcalloc
{
    bool init()
    {
        lazy_init();
        return s_inited && (s_mtx != nullptr);
    }

    bool alloc(ledc_channel_t &out)
    {
        bool ok = alloc_impl(out, 0, LEDC_CHANNEL_MAX - 1);
        if (!ok)
        {
            LOGE("LedcAllocator", "Failed to allocate channel");
        }
        return ok;
    }

    bool alloc_range(ledc_channel_t &out, int first, int last)
    {
        return alloc_impl(out, first, last);
    }

    void free(ledc_channel_t ch)
    {
        lazy_init();
        if (!(ch >= 0 && ch < LEDC_CHANNEL_MAX))
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
        for (int i = 0; i < LEDC_CHANNEL_MAX; ++i)
            if (!s_taken[i])
                ++cnt;
        unlock();
        return cnt;
    }

    bool is_taken(ledc_channel_t ch)
    {
        lazy_init();
        if (!(ch >= 0 && ch < LEDC_CHANNEL_MAX))
            return false;

        lock();
        bool v = s_taken[ch];
        unlock();
        return v;
    }

    // ---- RAII Lease ----
    ledcalloc::Lease::~Lease()
    {
        release();
    }

    bool ledcalloc::Lease::acquire_any()
    {
        if (_owned)
            return true;
        ledc_channel_t ch;
        if (!alloc(ch))
            return false;
        _ch = ch;
        _owned = true;
        return true;
    }

    bool ledcalloc::Lease::acquire_range(int first, int last)
    {
        if (_owned)
            return true;
        ledc_channel_t ch;
        if (!alloc_range(ch, first, last))
            return false;
        _ch = ch;
        _owned = true;
        return true;
    }

    void ledcalloc::Lease::release()
    {
        if (_owned && valid())
        {
            ledcalloc::free(_ch);
        }
        _owned = false;
        _ch = LEDC_CHANNEL_MAX;
    }

    ledc_channel_t ledcalloc::Lease::disown()
    {
        _owned = false;
        ledc_channel_t old = _ch;
        _ch = LEDC_CHANNEL_MAX;
        return old;
    }
} // namespace ledcalloc
