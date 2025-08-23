#include "mqtt_sink.hpp"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

static inline const char *safe_tag(const char *t) { return t ? t : ""; }

const char *MqttSink::level_str(LogLevel l)
{
    switch (l)
    {
    case LogLevel::Debug:
        return "DEBUG";
    case LogLevel::Info:
        return "INFO";
    case LogLevel::Warn:
        return "WARN";
    case LogLevel::Error:
        return "ERROR";
    case LogLevel::Critical:
        return "CRITICAL";
    default:
        return "?";
    }
}

// Minimal JSON escaper (quotes, backslash, control chars). Returns bytes written.
// Guarantees NUL termination if out_cap>0.
size_t MqttSink::json_escape(char *out, size_t out_cap, const char *in, size_t in_len)
{
    if (out_cap == 0)
        return 0;
    size_t o = 0;
    auto put = [&](char c)
    {
        if (o + 1 < out_cap)
            out[o++] = c;
    };
    auto put2 = [&](char a, char b)
    {
        if (o + 2 < out_cap)
        {
            out[o++] = a;
            out[o++] = b;
        }
    };

    for (size_t i = 0; i < in_len; ++i)
    {
        unsigned char c = static_cast<unsigned char>(in[i]);
        switch (c)
        {
        case '\"':
            put2('\\', '\"');
            break;
        case '\\':
            put2('\\', '\\');
            break;
        case '\b':
            put2('\\', 'b');
            break;
        case '\f':
            put2('\\', 'f');
            break;
        case '\n':
            put2('\\', 'n');
            break;
        case '\r':
            put2('\\', 'r');
            break;
        case '\t':
            put2('\\', 't');
            break;
        default:
            if (c < 0x20)
            {
                // \u00XX (4 hex digits). Keep it minimal to save code size.
                const char hex[] = "0123456789ABCDEF";
                if (o + 6 < out_cap)
                {
                    out[o++] = '\\';
                    out[o++] = 'u';
                    out[o++] = '0';
                    out[o++] = '0';
                    out[o++] = hex[(c >> 4) & 0xF];
                    out[o++] = hex[c & 0xF];
                }
            }
            else
            {
                put(static_cast<char>(c));
            }
        }
    }
    out[o] = '\0';
    return o;
}

void MqttSink::write(const LogRecord &r)
{
    // Fast-exit if broker isn’t up; we don’t block/retry here.
    if (!_svc.mqttConnected())
    {
        _dropped++;
        return;
    }

    // ---- Build payload JSON ----
    // We prefer the preformatted message r.msg; if absent (unlikely with your logger),
    // we fall back to formatting r.fmt/r.va into a small temp buffer first.
    char msgbuf[160];
    const char *msg_ptr = nullptr;
    size_t msg_len = 0;

    if (r.msg && r.msg_len)
    {
        msg_ptr = r.msg;
        msg_len = r.msg_len;
    }
    else if (r.fmt && r.va)
    {
        va_list copy;
        va_copy(copy, *r.va);
        int n = vsnprintf(msgbuf, sizeof(msgbuf), r.fmt, copy);
        va_end(copy);
        if (n > 0)
        {
            msg_ptr = msgbuf;
            msg_len = (n >= (int)sizeof(msgbuf)) ? (sizeof(msgbuf) - 1) : (size_t)n;
            msgbuf[msg_len] = '\0';
        }
        else
        {
            msg_ptr = "";
            msg_len = 0;
        }
    }
    else
    {
        msg_ptr = "";
        msg_len = 0;
    }

    // Escape into a bounded buffer
    char esc_msg[192];
    size_t esc_len = json_escape(esc_msg, sizeof(esc_msg), msg_ptr, msg_len);

    const char *tag = safe_tag(r.tag);
    char esc_tag[64];
    (void)json_escape(esc_tag, sizeof(esc_tag), tag, strnlen(tag, 63));

    // t: timestamp in microseconds; keep it numeric for easy parsing
    // Payload shape: {"t":12345678,"lvl":"WARN","tag":"DShot","msg":"..."}
    char json[256];
    int jl = snprintf(json, sizeof(json),
                      "{\"t\":%lu,\"lvl\":\"%s\",\"tag\":\"%s\",\"msg\":\"%s\"}",
                      (unsigned long)r.ts_us, level_str(r.level), esc_tag, esc_msg);
    if (jl < 0 || jl >= (int)sizeof(json))
    {
        _dropped++; // message too large to encode
        return;
    }

    // Best-effort publish (non-blocking). If the client TX buffer is full, publish() returns false.
    if (!_svc.publishRel((String("log/") + level_str(r.level)).c_str(), json, (size_t)jl, (MqttService::QoS)_qos, _retain))
    {
        _dropped++;
    }
}
