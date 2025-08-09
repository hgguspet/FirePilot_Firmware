#pragma once
#include <stdint.h>
#include <stddef.h>
#include <esp23_fast_timestamp.h>

/**
 * @file itelemetry_provider.hpp
 * @brief Interfaces and data types for producing telemetry samples.
 */

enum class TelemetryStatus : uint8_t
{
    OK,
    NO_DATA,
    ERROR
};

enum class TelemetryContentType : uint8_t
{
    JSON,
    CBOR,
    BINARY,
    TEXT
};

struct TelemetryMeta
{
    fasttime::Timestamp timestamp;
    uint8_t qos = 0;
    bool retain = false;
    TelemetryContentType content_type = TelemetryContentType::JSON;
    bool full_topic = false;
};

struct TelemetrySample
{
    const char *topic_suffix;
    const uint8_t *payload;
    size_t payload_length;
    TelemetryMeta meta;
};

class ITelemetryProvider
{
public:
    virtual ~ITelemetryProvider() = default;

    /// @return Short, stable provider name (e.g., "imu", "baro").
    virtual const char *name() const = 0;

    /// @return Target sampling frequency in Hz.
    virtual uint32_t sampleRateHz() const = 0;

    /// Initialize hardware/resources. Return false on fatal error.
    virtual bool begin() = 0;

    /// Fill @p out with the next sample if available.
    virtual TelemetryStatus sample(TelemetrySample &out) = 0;

    /// Called when the scheduler changes the polling rate.
    virtual void onRateChange(uint32_t newRateHz) { (void)newRateHz; }

    /// Absolute time when the next sample is due, or default for ASAP.
    virtual fasttime::Timestamp nextDueTimestamp() const { return {}; }
};
