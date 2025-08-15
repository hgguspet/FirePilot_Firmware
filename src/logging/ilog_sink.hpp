#pragma once
#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>

enum class LogLevel : uint8_t
{
    Debug,
    Info,
    Warn,
    Error,
    Critical,
    None
};

struct LogRecord
{
    uint32_t ts_us; // monotonic microseconds (or ms, your choice)
    LogLevel level;
    const char *tag; // static or short-lived string literal
    const char *fmt; // original printf fmt (for deferred formatting if desired)
    va_list *va;     // optional pointer (see below)
    const char *msg; // optional preformatted message (when formatting at producer)
    size_t msg_len;  // length if msg is set
    bool from_isr;   // produced from ISR?
};

struct ILogSink
{
    virtual ~ILogSink() = default;
    virtual void write(const LogRecord &r) = 0; // must be non-blocking/quick
    virtual void flush() {}                     // optional
};