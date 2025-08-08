#pragma once
#include "../interface_telemetry_provider.hpp"
#include "../encoders/json_encoder.hpp"

// lib
#include <Arduino.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_MPU9250.h>

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
    Adafruit_Sensor *_accelerometer, *_gyroscope, *_magnetometer;
    Adafruit_MPU9250 _imu;
    Adafruit_Mahony _filter;

// select either EEPROM or SPI FLASH storage:
#ifdef ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM
    Adafruit_Sensor_Calibration_EEPROM cal;
#else
    Adafruit_Sensor_Calibration_SDFat cal;
#endif

    int init_sensors(void);
};