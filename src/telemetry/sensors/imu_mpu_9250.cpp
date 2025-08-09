#include "imu_mpu_9250.hpp"

#ifdef PERFORMANCE_MONITORING
#include <esp23_fast_timestamp.h>
#endif

bool IMU_MPU9250::begin()
{
    if (!_imu.setup(0x68)) // Wire address
    {
        Serial.println("[IMU_MPU9250] MPU connection failed");
        return false;
    }

    return true;
}

bool IMU_MPU9250::sample(TelemetrySample &out)
{
#ifdef PERFORMANCE_MONITORING
    fasttime::Timestamp start = fasttime::Timestamp::now();
#endif

    if (!_imu.update())
    {
        Serial.println("[IMU_MPU9250] IMU update failed");
        return false;
    }

    _encoder.begin();
    _encoder.add("roll", _imu.getRoll());
    _encoder.add("pitch", _imu.getPitch());
    _encoder.add("yaw", _imu.getYaw());

    const char *payload;
    size_t len;
    if (!_encoder.finalize(payload, len))
    {
        Serial.println("[IMU_MPU_9250] Failed to encode JSON");
        return false;
    }

    out.topic_suffix = "imu";
    out.payload = payload;
    out.payload_length = len;
    out.is_binary = false; // JSON format

#ifdef PERFORMANCE_MONITORING
    Serial.printf("[IMU_MPU_9250] Sampled in %llu us\n", fasttime::elapsed_us(start));
#endif

    return true;
}
