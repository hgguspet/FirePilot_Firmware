#pragma once
#include <vector>
#include <Arduino.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
}
#include "telemetry/interface_telemetry_provider.hpp"

class TelemetryService
{
public:
    static TelemetryService &instance();

    void begin(const char *droneId, uint32_t tickHz = 100); // create task @ `tickHz`
    void addProvider(ITelemetryProvider *provider);

    // I2C mutex for bus safety
    SemaphoreHandle_t i2cMutex() const { return _i2cMutex; };

private:
    TelemetryService() = default;
    TelemetryService(const TelemetryService &) = delete;
    TelemetryService &operator=(const TelemetryService &) = delete;

    struct Slot
    {
        ITelemetryProvider *provider{nullptr};
        uint32_t periodTicks{0}; // in FreeRTOS ticks
        uint32_t nextTick{0};    // next tick to sample
    };

    static void taskEntry(void *pvParameters);
    void run();

private:
    std::vector<Slot> _slots;
    TaskHandle_t _taskHandle{nullptr};
    SemaphoreHandle_t _i2cMutex{nullptr};
    String _droneId{"drone-01"};
    uint32_t _tickHz{100}; // default tick rate
    uint32_t _tick{0};
};