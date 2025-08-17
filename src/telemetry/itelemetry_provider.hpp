#pragma once
/**
 * @file itelemetry_provider.hpp
 * @brief Interfaces and data types for producing telemetry samples.
 */

#include <stdint.h>
#include <stddef.h>

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
}

enum class TelemetryContentType : uint8_t
{
    JSON,
    CBOR,
    BINARY,
    TEXT
};

struct TelemetryMeta
{
    uint8_t qos = 0;
    bool retain = false;
    TelemetryContentType content_type = TelemetryContentType::JSON;
    bool full_topic = false;
};

/**
 * @brief Telemetry sample descriptor passed through the queue.
 * NOTE: topic_suffix/payload must point to storage that remains valid
 * until the consumer has used them (provider should double-buffer or similar).
 */
struct TelemetrySample
{
    const char *topic_suffix{nullptr};
    const uint8_t *payload{nullptr};
    size_t payload_length{0};
    TelemetryMeta meta{};
};

class ITelemetryProvider
{
public:
    virtual ~ITelemetryProvider() = default;

    /// @return Short, stable provider name (e.g., "imu", "baro").
    virtual const char *name() const = 0;

    /// @return Wanted sampling rate in Hz.
    virtual uint32_t sampleRateHz() const = 0;

    /// Initialize hardware/resources. Return false on fatal error.
    virtual bool begin() = 0;

    /// Called when e.g. TelemetryService changes sampling rate of this provider.
    virtual void onSamplingRateChange(uint32_t newRateHz) { (void)newRateHz; }

    /// Wire the output queue before tasks start.
    void setOutputQueue(QueueHandle_t qHandle) { _out = qHandle; }

protected:
    /**
     * @brief Publish by value (no heap). Returns false on failure.
     * @param timeoutTicks Use 0 to drop when the queue is full; or a small timeout for backpressure.
     *
     * @warning IMPORTANT: The queue must be created with item_size == sizeof(TelemetrySample).
     * The pointed-to buffers must remain valid until the consumer is done.
     */
    bool publish(const TelemetrySample &sample, TickType_t timeoutTicks = 0)
    {
        if (!_out)
        {
            return false;
        }
        // FreeRTOS will copy sizeof(TelemetrySample) bytes into the queue
        return xQueueSend(_out, &sample, timeoutTicks) == pdTRUE;
    }

    /**
     * @brief Overwrite the single-slot queue (latest-only semantics).
     * Only use if _out refers to a queue created with length==1.
     */
    bool publishOverwrite(const TelemetrySample &sample)
    {
        if (!_out)
            return false;
        return xQueueOverwrite(_out, &sample) == pdTRUE;
    }

private:
    QueueHandle_t _out{nullptr};
};
