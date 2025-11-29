// src/logging/logger.cpp
#include "logger.hpp"
#include "ilog_sink.hpp"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_timer.h" // esp_timer_get_time()
}

// ===== Tunables ===============================================================
#ifndef LOG_MSG_MAX
#define LOG_MSG_MAX 120 // bytes reserved for formatted message (excluding NUL)
#endif

#ifndef LOG_SINK_MAX
#define LOG_SINK_MAX 6
#endif

#ifndef LOG_TASK_STACK
#define LOG_TASK_STACK 4096 // bytes
#endif

#ifndef LOG_TASK_PRIO
#define LOG_TASK_PRIO 2
#endif

#ifndef LOG_TASK_CORE
#define LOG_TASK_CORE tskNO_AFFINITY
#endif

// ===== Internal queue item (POD, copied into FreeRTOS queue) ==================
namespace
{
    struct QueueItem
    {
        uint32_t ts_us;
        LogLevel level;
        const char *tag; // expected to be a literal or long-lived string
        bool from_isr;
        uint16_t len;          // used bytes in msg[]
        char msg[LOG_MSG_MAX]; // preformatted payload (no NUL required)
    };

    struct State
    {
        QueueHandle_t q = nullptr;
        LogLevel level = LogLevel::Info;
        ILogSink *sinks[LOG_SINK_MAX] = {};
        uint8_t sink_count = 0;
        TaskHandle_t task = nullptr;
        portMUX_TYPE sinks_mux = portMUX_INITIALIZER_UNLOCKED; // for sink access
    } S;

    static inline uint32_t monotonic_us()
    {
        return static_cast<uint32_t>(esp_timer_get_time());
    }

    static void log_task_trampoline(void * /*arg*/)
    {
        Logger::instance().consumeTask();
    }

    static inline void format_into_qi(QueueItem &qi, const char *fmt, va_list args)
    {
        va_list copy;
        va_copy(copy, args);
        int n = vsnprintf(qi.msg, LOG_MSG_MAX, fmt ? fmt : "", copy);
        va_end(copy);

        if (n < 0)
        {
            qi.len = 0;
            qi.msg[0] = '\0';
        }
        else if (n >= LOG_MSG_MAX)
        {
            qi.len = LOG_MSG_MAX - 1; // payload bytes (reserve 1 for NUL)
            qi.msg[qi.len] = '\0';    // ensure NUL
        }
        else
        {
            qi.len = static_cast<uint16_t>(n); // exact payload
            // vsnprintf already wrote NUL at msg[len]
        }
    }
} // namespace

// ===== Singleton boilerplate ==================================================
Logger &Logger::instance()
{
    static Logger inst;
    return inst;
}

// ===== Public API =============================================================
void Logger::init(uint16_t queue_capacity)
{
    if (S.q)
        return; // already initialized

    S.q = xQueueCreate(queue_capacity, sizeof(QueueItem));

#if (LOG_TASK_CORE == tskNO_AFFINITY)
    xTaskCreate(&log_task_trampoline, "log_consumer", LOG_TASK_STACK, nullptr, LOG_TASK_PRIO, &S.task);
#else
    xTaskCreatePinnedToCore(&log_task_trampoline, "log_consumer", LOG_TASK_STACK, nullptr, LOG_TASK_PRIO, &S.task, LOG_TASK_CORE);
#endif
}

void Logger::setMinLevel(LogLevel level)
{
    S.level = level;
}

LogLevel Logger::level() const
{
    return S.level;
}

void Logger::addSink(ILogSink *sink)
{
    if (!sink || S.sink_count >= LOG_SINK_MAX)
        return;
    portENTER_CRITICAL(&S.sinks_mux);
    S.sinks[S.sink_count++] = sink;
    portEXIT_CRITICAL(&S.sinks_mux);
}

void Logger::logf(LogLevel level, const char *tag, const char *fmt, ...)
{
    if (!S.q || level < S.level)
        return;
    QueueItem qi{};
    qi.ts_us = monotonic_us();
    qi.level = level;
    qi.tag = tag ? tag : "";
    qi.from_isr = false;

    va_list ap;
    va_start(ap, fmt);
    format_into_qi(qi, fmt, ap);
    va_end(ap);

    (void)xQueueSend(S.q, &qi, 0);
}

void Logger::logfIsr(LogLevel level, const char *tag, const char *fmt, ...)
{
    if (!S.q || level < S.level)
        return;
    QueueItem qi{};
    qi.ts_us = monotonic_us();
    qi.level = level;
    qi.tag = tag ? tag : "";
    qi.from_isr = true;

    va_list ap;
    va_start(ap, fmt);
    format_into_qi(qi, fmt, ap);
    va_end(ap);

    BaseType_t woken = pdFALSE;
    (void)xQueueSendFromISR(S.q, &qi, &woken);
    if (woken == pdTRUE)
        portYIELD_FROM_ISR();
}

void Logger::vlogf(LogLevel level, const char *tag, const char *fmt, va_list args, bool from_isr)
{
    if (!S.q || level < S.level)
        return;
    QueueItem qi{};
    qi.ts_us = monotonic_us();
    qi.level = level;
    qi.tag = tag ? tag : "";
    qi.from_isr = from_isr;

    format_into_qi(qi, fmt, args);

    if (from_isr)
    {
        BaseType_t woken = pdFALSE;
        (void)xQueueSendFromISR(S.q, &qi, &woken);
        if (woken == pdTRUE)
            portYIELD_FROM_ISR();
    }
    else
    {
        (void)xQueueSend(S.q, &qi, 0);
    }
}

// ===== Private helpers ========================================================
void Logger::enqueue(LogRecord &&r)
{
    // Convert the external LogRecord view into our QueueItem and enqueue (non-ISR).
    if (!S.q)
        return;
    QueueItem qi{};
    qi.ts_us = r.ts_us;
    qi.level = r.level;
    qi.tag = r.tag ? r.tag : "";
    qi.from_isr = r.from_isr;

    if (r.msg && r.msg_len)
    {
        size_t copy_len = (r.msg_len > (LOG_MSG_MAX - 1)) ? (LOG_MSG_MAX - 1) : r.msg_len;
        memcpy(qi.msg, r.msg, copy_len);
        qi.len = static_cast<uint16_t>(copy_len);
        qi.msg[qi.len] = '\0'; // ensure NUL
    }
    else if (r.fmt && r.va)
    {
        format_into_qi(qi, r.fmt, *r.va);
    }
    else
    {
        qi.len = 0;
        qi.msg[0] = '\0';
    }

    (void)xQueueSend(S.q, &qi, 0);
}

void Logger::consumeTask()
{
    QueueItem qi;
    for (;;)
    {
        if (xQueueReceive(S.q, &qi, portMAX_DELAY) != pdPASS)
            continue;

        LogRecord r{};
        r.ts_us = qi.ts_us;
        r.level = qi.level;
        r.tag = qi.tag;
        r.from_isr = qi.from_isr;
        r.msg = qi.len ? qi.msg : nullptr;
        r.msg_len = qi.len;
        r.channel = "log";

        // snapshot sinks to minimize time in critical section
        ILogSink *
            local[LOG_SINK_MAX];
        uint8_t cnt;
        portENTER_CRITICAL(&S.sinks_mux);
        cnt = S.sink_count;
        for (uint8_t i = 0; i < cnt; ++i)
            local[i] = S.sinks[i];
        portEXIT_CRITICAL(&S.sinks_mux);

        for (uint8_t i = 0; i < cnt; ++i)
        {
            local[i]->write(r);
        }
    }
}
