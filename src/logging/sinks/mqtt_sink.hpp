#pragma once
#include "../ilog_sink.hpp"
#include "services/mqtt_service.hpp"

class MqttSink : public ILogSink
{
public:
    // as by this projects mqtt topic convention, the topic equates to: <deviceId>/log/<severity>
    // qos/retain: MQTT flags (keep qos=0, retain=false for logs)
    MqttSink(
        MqttService::MqttService &svc,
        const char *baseTopic = "log",
        uint8_t qos = 0,
        bool retain = false)
        : _svc(svc), _base(baseTopic), _qos(qos), _retain(retain) {}

    void write(const LogRecord &r) override;

    // Optional counters for observability
    uint32_t droppedPublishes() const { return _dropped; }

private:
    MqttService::MqttService &_svc;
    const char *_base;
    uint8_t _qos;
    bool _retain;
    uint32_t _dropped = 0;

    static const char *level_str(LogLevel l);
    static size_t json_escape(char *out, size_t out_cap, const char *in, size_t in_len);
};
