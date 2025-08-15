#pragma once
#include "../itelemetry_provider.hpp"
#include <json_buffer_writer.hpp>

// lib
#include <Arduino.h>
#include <MPU9250.h>
#include <esp23_fast_timestamp.h>

#define FILTER_UPDATE_RATE_HZ 100

class IMU_MPU9250 final : public ITelemetryProvider
{
public:
    explicit IMU_MPU9250()
        : _jw(_buf, sizeof(_buf)) {}

    const char *name() const override { return "IMU_MPU_9250"; }
    uint32_t sampleRateHz() const override { return 100; } // 100Hz update rate
    bool begin() override;
    TelemetryStatus sample(TelemetrySample &out) override;

private:
    // Timing
    fasttime::Timestamp _last_sample_time;

    // Sensor
    MPU9250 _imu;

    // Json Encoding
    uint8_t _buf[256];
    constexpr static size_t _buf_size = sizeof(_buf);
    JsonBufWriter _jw{_buf, _buf_size};
};