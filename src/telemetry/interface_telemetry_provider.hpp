#pragma once
#include <stdint.h>
#include <stddef.h>

struct TelemetrySample
{
    const char *topic_suffix;
    const char *payload;
    size_t payload_length;
    bool is_binary; // encoding type
};

class ITelemetryProvider
{
public:
    virtual ~ITelemetryProvider() = default;

    virtual const char *name() const = 0;
    virtual uint32_t sampleRateHz() const = 0;
    virtual bool begin() = 0;
    virtual bool sample(TelemetrySample &out) = 0;
};
