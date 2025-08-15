#pragma once
#include "itelemetry_provider.hpp"
#include "../drivers/iesc_driver.hpp"
#include <json_buffer_writer.hpp>

class EscTelemetryAdapter final : public ITelemetryProvider
{
public:
    explicit EscTelemetryAdapter(IEscDriver &drv,
                                 const char *topicSuffix,
                                 uint32_t rateHz,
                                 uint8_t *buf,
                                 size_t bufCap)
        : drv_(drv), topic_(topicSuffix), rateHz_(rateHz), jw_(buf, bufCap) {}

    const char *name() const override { return "esc"; }
    uint32_t sampleRateHz() const override { return rateHz_; }
    bool begin() override { return true; } // driver is begun elsewhere
    void onRateChange(uint32_t hz) override { rateHz_ = hz; }

    TelemetryStatus sample(TelemetrySample &out) override
    {
        IEscDriver::Telemetry t{};
        if (!drv_.readTelemetry(t) || !t.valid)
            return TelemetryStatus::NO_DATA;

        jw_.reset(nullptr, 0); // ensure state clear
        // Write into the provided buffer (jw_ already owns the caller buffer)
        jw_.beginObject();
        jw_.key("rpm");
        jw_.value((uint32_t)t.rpm);
        jw_.key("tempC");
        jw_.value((uint32_t)t.temperatureC);
        jw_.key("mV");
        jw_.value((uint32_t)t.millivolts);
        jw_.key("mA");
        jw_.value((uint32_t)t.milliamps);
        jw_.endObject();

        const uint8_t *p;
        size_t n;
        if (!jw_.finalize(p, n))
            return TelemetryStatus::ERROR;

        out.topic_suffix = topic_;
        out.payload = p;
        out.payload_length = n;
        out.meta.timestamp = fasttime::Timestamp::now();
        out.meta.content_type = TelemetryContentType::JSON;
        return TelemetryStatus::OK;
    }

private:
    IEscDriver &drv_;
    const char *topic_;
    uint32_t rateHz_;
    JsonBufWriter jw_;
};
