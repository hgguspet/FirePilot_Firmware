// telemetry_service.cpp
#include "telemetry_service.hpp"
#include "logging/logger.hpp"
#include "mqtt_service.hpp" // TODO: Allow for publishing through sinks and logger instead? or something

#include <cstring> // strlen

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
}

TelemetryService &TelemetryService::instance()
{
    static TelemetryService inst;
    return inst;
}

void TelemetryService::begin(
    const char *droneId,
    size_t queueLen,
    UBaseType_t txPrio,
    uint32_t txStackWords,
    BaseType_t txCore)
{
    _deviceId = droneId ? droneId : "Drone";

    _queue = xQueueCreate(queueLen, sizeof(TelemetrySample));
    if (!_queue)
    {
        LOGE("Telemetry", "Failed to create telemetry queue");
        return;
    }

    _i2cMutex = xSemaphoreCreateMutex();
    if (!_i2cMutex)
    {
        LOGW("Telemetry", "Failed to create I2C mutex");
        // Continue, but providers that use I2C should check for null
    }

    const BaseType_t ok = xTaskCreatePinnedToCore(
        &_txThunk, "TelemetryTx", txStackWords, this, txPrio, &_txTask, txCore);
    if (ok != pdPASS)
    {
        LOGE("Telemetry", "Failed to create TX task");
    }
}

void TelemetryService::addProvider(ITelemetryProvider *provider)
{
    if (!provider || !_queue)
        return;

    provider->setOutputQueue(_queue);

    if (provider->begin())
    {
        _providers.push_back(provider);
        LOGI("Telemetry", "Provider added: %s", provider->name());
    }
    else
    {
        LOGE("Telemetry", "Provider init failed: %s", provider->name());
    }
}

void TelemetryService::_txThunk(void *arg)
{
    static_cast<TelemetryService *>(arg)->_txLoop();
}

void TelemetryService::_txLoop()
{
    TelemetrySample s{};
    for (;;)
    {
        if (xQueueReceive(_queue, &s, portMAX_DELAY) == pdTRUE)
        {
            // Build final topic
            String topic;
            if (s.meta.full_topic)
            {
                topic = s.topic_suffix ? s.topic_suffix : "";
            }
            else
            {
                const size_t suffixLen = s.topic_suffix ? std::strlen(s.topic_suffix) : 0;
                topic.reserve(_deviceId.length() + 1 + suffixLen);
                topic = _deviceId;
                topic += '/';
                if (s.topic_suffix)
                    topic += s.topic_suffix;
            }

            // Transmit now (do not stash pointers for later)
            transmit(topic.c_str(), s);
        }
    }
}

void TelemetryService::transmit(const char *topic, const TelemetrySample &s)
{
    // (For now) Publish directly through MQTT
    MqttService::MqttService::instance().publish(
        topic, reinterpret_cast<const char *>(s.payload), s.payload_length,
        (MqttService::QoS)s.meta.qos, s.meta.retain);
}
