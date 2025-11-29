#include "mqtt_sink.hpp"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

static inline const char *safe_cstr(const char *s) { return s ? s : ""; }

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
    case LogLevel::None:
        return ""; // used to omit level segment
    default:
        return "?";
    }
}

size_t MqttSink::json_escape(char *out, size_t cap, const char *in, size_t len)
{
    if (cap == 0)
        return 0;
    size_t o = 0;
    auto put = [&](char c)
    { if (o + 1 < cap) out[o++] = c; };
    auto put2 = [&](char a, char b)
    { if (o + 2 < cap) { out[o++] = a; out[o++] = b; } };

    for (size_t i = 0; i < len; ++i)
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
                static const char hex[] = "0123456789ABCDEF";
                if (o + 6 < cap)
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
    if (!_svc.mqttConnected())
    {
        _dropped++;
        return;
    }

    // ---- message text (prefer preformatted) ----
    char msgbuf[160];
    const char *msg_ptr = "";
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
            msg_len = (n >= (int)sizeof(msgbuf)) ? sizeof(msgbuf) - 1 : (size_t)n;
            msgbuf[msg_len] = '\0';
            msg_ptr = msgbuf;
        }
    }

    // ---- payload JSON ----
    char esc_msg[192];
    (void)json_escape(esc_msg, sizeof(esc_msg), msg_ptr, msg_len);

    char esc_tag[64];
    (void)json_escape(esc_tag, sizeof(esc_tag), safe_cstr(r.tag),
                      strnlen(safe_cstr(r.tag), sizeof(esc_tag) - 1));

    char json[256];
    int jl = snprintf(
        json, sizeof(json),
        "{\"t\":%lu,\"lvl\":\"%s\",\"tag\":\"%s\",\"msg\":\"%s\"}",
        (unsigned long)r.ts_us, level_str(r.level), esc_tag, esc_msg);
    if (jl < 0 || jl >= (int)sizeof(json))
    {
        _dropped++;
        return;
    }

    // ---- topic: <channel>/<level?> ----
    const char *channel = (r.channel && r.channel[0]) ? r.channel : _base;
    const char *lvl = level_str(r.level);

    char topic[128];
    if (r.level == LogLevel::None || !lvl[0])
    {
        // only channel
        snprintf(topic, sizeof(topic), "%s", safe_cstr(channel));
    }
    else
    {
        // channel/LEVEL
        snprintf(topic, sizeof(topic), "%s/%s", safe_cstr(channel), lvl);
    }

    // ---- publish (non-blocking) ----
    if (!_svc.publishRel(topic, json, (size_t)jl, (MqttService::QoS)_qos, _retain))
    {
        _dropped++;
    }
}
