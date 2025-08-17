#pragma once
#include "../itelemetry_provider.hpp"
#include <json_buffer_writer.hpp>

// lib
#include <Arduino.h>
#include <MPU9250.h>
#include <esp23_fast_timestamp.h>

class IMU_MPU9250 final : public ITelemetryProvider
{
public:
    explicit IMU_MPU9250(SemaphoreHandle_t i2cMutex = nullptr, uint32_t rateHz = 200, const char *topicSuffix = "telemetry/imu")
        : _i2cMutex(i2cMutex), _rateHz(rateHz), _topicSuffix(topicSuffix) {}

    const char *name() const override { return "IMU_MPU_9250"; }
    uint32_t sampleRateHz() const override { return _rateHz; }

    bool begin() override;

    void setI2CMutex(SemaphoreHandle_t i2cMutex) { _i2cMutex = i2cMutex; }

    void onSamplingRateChange(uint32_t newRateHz) override
    {
        _rateHz = (newRateHz == 0 ? 1 : newRateHz);
    }

private:
    // Tasking / synchronization
    static void _taskThunk(void *arg);
    void _runLoop();
    TaskHandle_t _taskHandle;
    SemaphoreHandle_t _i2cMutex;

    // Config
    uint32_t _rateHz;
    const char *_topicSuffix;

    // Sensor
    MPU9250 _imu;

    // Double Buffering
    uint8_t _buf[2][256];
    size_t _len[2] = {0, 0};
    uint8_t _buf_index = 0;

    // Json Encoding
    JsonBufWriter _jw[2] = {
        JsonBufWriter(_buf[0], sizeof(_buf[0])),
        JsonBufWriter(_buf[1], sizeof(_buf[1]))};
};