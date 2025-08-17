#include "imu_mpu_9250.hpp"
#include "logging/logger.hpp"

bool IMU_MPU9250::begin()
{
    // Guard setup: WHOAMI, config writes, etc.
    if (_i2cMutex)
    {
        xSemaphoreTake(_i2cMutex, portMAX_DELAY);
    }
    const bool ok = _imu.setup(0x68);
    if (_i2cMutex)
    {
        xSemaphoreGive(_i2cMutex);
    }

    if (!ok)
    {
        LOGE("IMU_MPU9250", "MPU connection failed");
        return false;
    }

    // Create sampling task
    const BaseType_t rc = xTaskCreatePinnedToCore(
        &_taskThunk, "IMU_MPU9250", 2048, this, 3, &_taskHandle, tskNO_AFFINITY);
    if (rc != pdPASS)
    {
        LOGE("IMU_MPU9250", "Task create failed");
        return false;
    }
    return true;
}

void IMU_MPU9250::_taskThunk(void *arg)
{
    static_cast<IMU_MPU9250 *>(arg)->_runLoop();
}

void IMU_MPU9250::_runLoop()
{
    TickType_t last = xTaskGetTickCount(); // run now
    while (true)
    {
        // Pace FIRST so failures can’t spin the loop
        const uint32_t hz = _rateHz ? _rateHz : 1;
        const TickType_t period =
            (TickType_t)(((uint64_t)configTICK_RATE_HZ + hz - 1) / hz); // ceil

        vTaskDelayUntil(&last, period);

        // --- Read sensor (I2C protected) ---
        if (_i2cMutex)
        {
            xSemaphoreTake(_i2cMutex, portMAX_DELAY);
        }
        else
        {
            // Warn about I2C mutex not taken
            LOGW("IMU_MPU9250", "I2C mutex not taken");
        }

        const bool ok = _imu.update();

        if (_i2cMutex)
        {
            xSemaphoreGive(_i2cMutex);
        }

        if (!ok)
        {
            LOGE("IMU_MPU9250", "IMU update failed");
            continue;
        }

        // update current buffer
        JsonBufWriter &jw = _jw[_buf_index];
        jw.reset(_buf[_buf_index], sizeof(_buf[_buf_index]));
        // Create json string
        jw.beginObject();
        jw.key("roll");
        jw.value(_imu.getRoll());
        jw.key("pitch");
        jw.value(_imu.getPitch());
        jw.key("yaw");
        jw.value(_imu.getYaw());
        jw.endObject();

        const uint8_t *output;
        size_t length;
        if (!jw.finalize(output, length))
        {
            LOGE("IMU_MPU9250", "JSON finalization failed");
            return;
        }

        TelemetrySample sample{
            .topic_suffix = _topicSuffix,
            .payload = output,
            .payload_length = length,
            .meta = TelemetryMeta{
                .qos = 0,
                .retain = false,
                .content_type = TelemetryContentType::JSON,
                .full_topic = false,
            }};

        // LOGI("IMU_MPU9250", "Publishing telemetry sample, topic %s", _topicSuffix);
        (void)publish(sample, 0); // Non-blocking, drop if queue is full

        // Flip buffer index
        _buf_index = (_buf_index + 1) % sizeof(_buf) / sizeof(_buf[0]);
    }
}