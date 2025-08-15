#include "serial_sink.hpp"
#include <Arduino.h> // HardwareSerial
#include <stdio.h>
#include <inttypes.h> // PRIu64 if you want it

extern "C"
{
#include "esp_timer.h" // esp_timer_get_time()
}

SerialSink::SerialSink(void *serial_like)
    : _ser(serial_like) {}

static inline const char *lvl_str(LogLevel l)
{
    switch (l)
    {
    case LogLevel::Debug:
        return "D";
    case LogLevel::Info:
        return "I";
    case LogLevel::Warn:
        return "W";
    case LogLevel::Error:
        return "E";
    case LogLevel::Critical:
        return "C";
    default:
        return "?";
    }
}

void SerialSink::write(const LogRecord &r)
{
    auto *ser = reinterpret_cast<HardwareSerial *>(_ser);
    if (!ser)
        return;

    // Timestamp in ms since boot (uint64_t)
    uint64_t ts_ms = static_cast<uint64_t>(esp_timer_get_time()) / 1000ull;

    // Build prefix separately to avoid clobbering message buffer
    char prefix[96];
    int p = snprintf(prefix, sizeof(prefix),
                     "[%llu][%s][%s] ",
                     static_cast<unsigned long long>(ts_ms),
                     lvl_str(r.level),
                     (r.tag ? r.tag : ""));
    if (p < 0)
        p = 0;
    if (p > (int)sizeof(prefix))
        p = sizeof(prefix);

    // Emit prefix
    ser->write(reinterpret_cast<const uint8_t *>(prefix), (size_t)p);

    // Emit payload: prefer preformatted r.msg; otherwise format locally
    if (r.msg && r.msg_len)
    {
        ser->write(reinterpret_cast<const uint8_t *>(r.msg), r.msg_len);
    }
    else if (r.fmt && r.va)
    {
        char msgbuf[128];
        va_list copy;
        va_copy(copy, *r.va);
        int n = vsnprintf(msgbuf, sizeof(msgbuf), r.fmt, copy);
        va_end(copy);
        if (n > 0)
        {
            size_t to_write = (n >= (int)sizeof(msgbuf)) ? sizeof(msgbuf) : (size_t)n;
            ser->write(reinterpret_cast<const uint8_t *>(msgbuf), to_write);
        }
    }
    // Newline (CRLF for most terminals)
    ser->write(reinterpret_cast<const uint8_t *>("\r\n"), 2);
}
