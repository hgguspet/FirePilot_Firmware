#pragma once
#include "../ilog_sink.hpp"
#include "services/mqtt_service.hpp"

class MqttSink : public ILogSink
{
public:
    // as by this projects mqtt topic convention, the topic equates to: <deviceId>/log/<severity>
    // qos/retain: MQTT flags (keep qos=0, retain=false for logs)
    MqttSink(MqttService &svc,
             const char *baseTopic = "log",
             const char *deviceId = nullptr,
             uint8_t qos = 0,
             bool retain = false)
        : svc_(svc), base_(baseTopic), dev_(deviceId), qos_(qos), retain_(retain) {}

    void write(const LogRecord &r) override;

    // Optional counters for observability
    uint32_t droppedPublishes() const { return dropped_; }

private:
    MqttService &svc_;
    const char *base_;
    const char *dev_;
    uint8_t qos_;
    bool retain_;
    uint32_t dropped_ = 0;

    static const char *level_str(LogLevel l);
    static size_t json_escape(char *out, size_t out_cap, const char *in, size_t in_len);
};
