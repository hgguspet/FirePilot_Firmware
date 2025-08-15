#include "imu_mpu_9250.hpp"
#include "logging/logger.hpp"

bool IMU_MPU9250::begin()
{
    if (!_imu.setup(0x68)) // Wire address
    {
        LOGE("IMU_MPU9250", "[IMU_MPU9250] MPU connection failed");
        return false;
    }

    return true;
}

TelemetryStatus IMU_MPU9250::sample(TelemetrySample &out)
{
    _last_sample_time = fasttime::Timestamp::now();

    if (!_imu.update())
    {
        LOGE("IMU_MPU9250", "IMU update failed");
        return TelemetryStatus::ERROR;
    }

    // Reset JSON writer
    _jw.reset(_buf, _buf_size);

    // Create json string
    _jw.beginObject();
    _jw.key("roll");
    _jw.value(_imu.getRoll());
    _jw.key("pitch");
    _jw.value(_imu.getPitch());
    _jw.key("yaw");
    _jw.value(_imu.getYaw());
    _jw.endObject();

    const uint8_t *payload;
    size_t len;
    if (!_jw.finalize(payload, len))
    {
        LOGE("IMU_MPU9250", "JSON finalization failed");
        return TelemetryStatus::ERROR;
    }

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
    LOGD("IMU_MPU_9250", "Sampled in %llu us", fasttime::elapsed_us(_last_sample_time));
#endif

    return TelemetryStatus::OK;
}
