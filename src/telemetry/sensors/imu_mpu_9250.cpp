#include "imu_mpu_9250.hpp"

bool IMU_MPU9250::begin()
{
    if (!_imu.setup(0x68)) // Wire address
    {
        Serial.println("[IMU_MPU9250] MPU connection failed");
        return false;
    }

    return true;
}

TelemetryStatus IMU_MPU9250::sample(TelemetrySample &out)
{
    _last_sample_time = fasttime::Timestamp::now();

    if (!_imu.update())
    {
        Serial.println("[IMU_MPU9250] IMU update failed");
        return TelemetryStatus::ERROR;
    }

    _jw.beginObject();
    _jw.key("roll");
    _jw.value(_imu.getRoll());
    _jw.key("pitch");
    _jw.value(_imu.getPitch());
    _jw.key("yaw");
    _jw.value(_imu.getYaw());

    const uint8_t *payload;
    size_t len;
    if (!_jw.finalize(payload, len))
        return TelemetryStatus::ERROR;

    const char *topic = "imu";

    out.topic_suffix = topic;
    out.payload = payload;
    out.payload_length = len;
    out.meta.timestamp = _last_sample_time;
    out.meta.content_type = TelemetryContentType::JSON; // JSON format
    out.meta.qos = 0;                                   // Default QoS
    out.meta.retain = false;                            // Not retained
    out.meta.full_topic = false;                        // Not full topic

#ifdef PERFORMANCE_MONITORING
    Serial.printf("[IMU_MPU_9250] Sampled in %llu us\n", fasttime::elapsed_us(_last_sample_time));
#endif

    return TelemetryStatus::OK;
}
