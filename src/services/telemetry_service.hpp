#pragma once
#include <vector>
#include <atomic>
#include <Arduino.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
}
#include "telemetry/itelemetry_provider.hpp"

class TelemetryService
{
public:
    static TelemetryService &instance();

    void begin(const char *droneId, size_t queueLen = 64, UBaseType_t txPrio = 2,
               uint32_t txStackWords = 4096, BaseType_t txCore = tskNO_AFFINITY); // create telemetry post task @ `tickHz`
    void addProvider(ITelemetryProvider *provider);

    /**
     * @brief For i2c bus safety always request the handle before using the bus.
     */
    SemaphoreHandle_t i2cMutex() const { return _i2cMutex; };

private:
    explicit TelemetryService() = default;

    static void _txThunk(void *arg);
    void _txLoop();

    void transmit(const char *topic, const TelemetrySample &s);

private:
    std::vector<ITelemetryProvider *> _providers;
    QueueHandle_t _queue{nullptr};
    TaskHandle_t _txTask{nullptr};
    SemaphoreHandle_t _i2cMutex{nullptr};
    String _deviceId{"Drone"};
};