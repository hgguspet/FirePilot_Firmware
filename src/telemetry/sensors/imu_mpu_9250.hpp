#pragma once

/**
 * @brief IMU telemetry provider using MPU9250 sensor
 *
 * This class provides telemetry data from an MPU9250 IMU sensor, including
 * roll, pitch, and yaw measurements. The sensor data is continuously sampled
 * in a dedicated FreeRTOS task and published as JSON telemetry.
 *
 * @warning The MPU9250 sensor requires continuous updates to maintain proper
 * sensor fusion and filtering. Update rates below 10Hz may result in stale
 * or incorrect readings. Recommended minimum rate is 25-40Hz for reliable
 * operation. I2C mutex blocking can also interfere with sensor fusion timing.
 *
 * Key features:
 * - Thread-safe I2C communication with mutex support
 * - Double-buffered JSON encoding to prevent blocking
 * - Configurable sampling rate (default 200Hz)
 * - Non-blocking telemetry publishing
 *
 * JSON output format:
 * @code{.json}
 * {"roll": 0.123, "pitch": -0.456, "yaw": 180.789}
 * @endcode
 */

#include "../itelemetry_provider.hpp"
#include <json_buffer_writer.hpp>

#include <Arduino.h>
#include <MPU9250.h>

class IMU_MPU9250 final : public ITelemetryProvider
{
public:
    /**
     * @brief Construct a new IMU_MPU9250 telemetry provider
     *
     * @param i2cMutex Optional I2C mutex for thread-safe communication (can be nullptr)
     * @param rateHz Sampling rate in Hz (default: 200Hz, minimum recommended: 25Hz)
     * @param topicSuffix MQTT topic suffix for telemetry publishing
     *
     * @warning Using rates below 25Hz may cause sensor fusion issues and stale readings
     */
    explicit IMU_MPU9250(SemaphoreHandle_t i2cMutex = nullptr,
                         uint32_t rateHz = 200,
                         const char *topicSuffix = "telemetry/imu")
        : _i2cMutex(i2cMutex), _rateHz(rateHz), _topicSuffix(topicSuffix) {}

    /**
     * @brief Get the provider name
     * @return Provider name string
     */
    const char *name() const override { return "IMU_MPU_9250"; }

    /**
     * @brief Get the current sampling rate
     * @return Sampling rate in Hz
     */
    uint32_t sampleRateHz() const override { return _rateHz; }

    /**
     * @brief Initialize the IMU sensor and start sampling task
     *
     * Performs sensor initialization, creates the sampling task, and begins
     * continuous telemetry generation.
     *
     * @return true if initialization successful, false otherwise
     * @warning Ensure I2C bus is properly initialized before calling this method
     */
    bool begin() override;

    /**
     * @brief Set the I2C mutex for thread-safe communication
     *
     * @param i2cMutex Mutex handle, or nullptr to disable mutex protection
     * @warning Changing mutex during operation may cause race conditions
     */
    void setI2CMutex(SemaphoreHandle_t i2cMutex) { _i2cMutex = i2cMutex; }

    /**
     * @brief Handle sampling rate change requests
     *
     * @param newRateHz New sampling rate in Hz (minimum 1Hz)
     * @warning Rates below 25Hz may cause sensor fusion issues. The MPU9250
     * requires regular updates to maintain accurate orientation calculations.
     */
    void onSamplingRateChange(uint32_t newRateHz) override
    {
        _rateHz = (newRateHz == 0 ? 1 : newRateHz);
    }

private:
    // Tasking / synchronization
    /**
     * @brief Static task entry point for FreeRTOS
     * @param arg Pointer to IMU_MPU9250 instance
     */
    static void _taskThunk(void *arg);

    /**
     * @brief Main sampling loop running in dedicated task
     *
     * Continuously samples the IMU sensor, generates JSON telemetry,
     * and publishes data at the configured rate.
     */
    void _runLoop();

    TaskHandle_t _taskHandle;    ///< FreeRTOS task handle
    SemaphoreHandle_t _i2cMutex; ///< I2C bus protection mutex

    // Config
    uint32_t _rateHz;         ///< Sampling rate in Hz
    const char *_topicSuffix; ///< MQTT topic suffix

    // Sensor
    MPU9250 _imu; ///< MPU9250 sensor instance

    // Double Buffering
    uint8_t _buf[2][256];    ///< Double buffer for JSON data
    size_t _len[2] = {0, 0}; ///< Buffer lengths
    uint8_t _buf_index = 0;  ///< Current buffer index

    // Json Encoding
    JsonBufWriter _jw[2] = {///< JSON writers for each buffer
                            JsonBufWriter(_buf[0], sizeof(_buf[0])),
                            JsonBufWriter(_buf[1], sizeof(_buf[1]))};
};