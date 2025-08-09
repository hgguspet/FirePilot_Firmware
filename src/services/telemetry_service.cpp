#include "telemetry_service.hpp"
#include "mqtt_service.hpp"

TelemetryService &TelemetryService::instance()
{
    static TelemetryService instance;
    return instance;
}

void TelemetryService::addProvider(ITelemetryProvider *provider)
{
    if (!provider)
        return;

    Slot slot;
    slot.provider = provider;

    // Period in ticks  (tickHz_ / rate_hq), min 1
    slot.periodTicks = provider->sampleRateHz() ? max(1, static_cast<int>(_tickHz / provider->sampleRateHz())) : _tickHz;
    slot.nextTick = _tick; // Start immediately
    _slots.push_back(slot);
}

void TelemetryService::begin(const char *droneId, uint32_t tickHz)
{
    if (droneId && *droneId)
        _droneId = droneId;

    //   Init  providers
    for (auto &slot : _slots)
    {
        if (!slot.provider->begin())
        {
            Serial.printf("[TelemetryService] Failed to initialize provider: %s\n", slot.provider->name());
        }
    }

    // Shared I2C mutex
    _i2cMutex = xSemaphoreCreateMutex();

    // Start task
    xTaskCreatePinnedToCore(&TelemetryService::taskEntry, "telemetry_task",
                            4096, this, 1, &_taskHandle, 1 /* core */);

    Serial.printf("[TelemetryService] Started @ %u Hz\n", _tickHz);
}

void TelemetryService::taskEntry(void *pvParameters)
{
    reinterpret_cast<TelemetryService *>(pvParameters)->run();
}

void TelemetryService::run()
{
    const TickType_t tickPeriod = pdMS_TO_TICKS(1000 / _tickHz);
    TickType_t last = xTaskGetTickCount();
    while (true)
    {
        vTaskDelayUntil(&last, tickPeriod);
        _tick++;

        // Iterate providers and trigger due ones
        for (auto &slot : _slots)
        {
            if (_tick >= slot.nextTick)
            {
                TelemetrySample sample;
                TelemetryStatus status = slot.provider->sample(sample);
                slot.nextTick += slot.periodTicks; // Schedule next sample

                if (status == TelemetryStatus::OK && sample.payload && sample.payload_length)
                {
                    if (sample.meta.full_topic)
                    {
                        // Publish to full topic
                        MqttService::instance().publish(sample.topic_suffix, reinterpret_cast<const char *>(sample.payload), sample.payload_length,
                                                        0 /*  QoS */, false /* retain */);
                    }
                    else
                    {
                        // Compose topic
                        String topic = _droneId + "/";
                        topic += "telemetry/";
                        topic += sample.topic_suffix ? sample.topic_suffix : slot.provider->name();

                        // Publish (ignore if not connected)
                        MqttService::instance().publish(topic.c_str(),
                                                        reinterpret_cast<const char *>(sample.payload), sample.payload_length,
                                                        0 /*  QoS */, false /* retain */);
                    }
                }
            }
        }
    }
}