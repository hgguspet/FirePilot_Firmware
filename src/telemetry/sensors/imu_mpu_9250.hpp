#pragma once
#include "../itelemetry_provider.hpp"
#include "../encoders/json_encoder.hpp"

// lib
#include "MPU9250.h"

#define FILTER_UPDATE_RATE_HZ 100

class IMU_MPU9250 : public ITelemetryProvider
{
public:
    explicit IMU_MPU9250(JsonEncoder &enc)
        : _encoder(enc) {}
    const char *name() const override { return "IMU_MPU_9250"; }
    uint32_t sampleRateHz() const override { return 100; } // 100Hz update rate
    bool begin() override;
    bool sample(TelemetrySample &out) override;

private:
    JsonEncoder &_encoder;

    // Sensor
    MPU9250 _imu;
};