#pragma once
#include <Arduino.h>
#include "logging/ilog_sink.hpp"

struct ProbeSink : public ILogSink
{
    volatile uint32_t count = 0;

    // last values captured (copied in write)
    LogLevel last_level = LogLevel::Info;
    char last_tag[16] = {};
    char last_msg[256] = {}; // big enough for tests
    size_t last_len = 0;
    volatile uint32_t hits = 0; // increment each write

    void write(const LogRecord &r) override
    {
        // snapshot for assertions
        last_level = r.level;

        // copy tag
        const char *t = r.tag ? r.tag : "";
        strncpy(last_tag, t, sizeof(last_tag) - 1);
        last_tag[sizeof(last_tag) - 1] = '\0';

        // copy message
        size_t len = 0;
        if (r.msg && r.msg_len)
        {
            len = (r.msg_len < sizeof(last_msg) - 1) ? r.msg_len : (sizeof(last_msg) - 1);
            memcpy(last_msg, r.msg, len);
            last_msg[len] = '\0';
        }
        else
        {
            last_msg[0] = '\0';
        }
        last_len = len;

        hits++;
        count++;
    }
};

// tiny helper to wait until a sink receives N messages (with timeout)
inline bool wait_for_count(volatile uint32_t &counter, uint32_t target, uint32_t timeout_ms = 1000)
{
    uint32_t start = millis();
    while (millis() - start < timeout_ms)
    {
        if (counter >= target)
            return true;
        delay(1);
    }
    return false;
}
