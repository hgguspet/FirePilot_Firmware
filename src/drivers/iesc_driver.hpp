#pragma once
#include <cstdint>

// For telemetry
#include "../telemetry/itelemetry_provider.hpp"
#include "../telemetry/encoders/json_buf_writer.hpp"

struct EscCapabilities
{
    bool bidirTelemetry : 1; // Supports bidirectional telemetry (DShot)
    bool needsCalibrate : 1; // analog/PWM ESCs
    uint16_t maxRateHz;      // Max update rate
};

class IEscDriver
{
public:
    virtual ~IEscDriver() = default;

    // Lifecycle / setup
    virtual bool begin(uint8_t pin, uint16_t rateHz) = 0; // pin is GPIO number
    virtual void end() = 0;
    virtual EscCapabilities caps() const = 0;

    /**
     * Control Path (hot)
     * norm in [0..1]; implementation maps to protocol units
     */
    virtual void writeNormalized(float norm01) = 0;

    /**
     * Optional features
     */
    virtual void arm(bool on) { (void)on; } // default no-op
    virtual void setUpdateRate(uint16_t rateHz) = 0;

    /**
     * Batch / sync frame (prevents jitter across multiple writes)
     */
    virtual void beginFrame() {}
    virtual void endFrame() {}

    /**
     * Optional raw telemetry pull (protocol-native). Return false if none available.
     * Implementations may fill any/all fields.
     */
    struct Telemetry
    {
        bool valid = false;
        uint16_t rpm = 0;
        uint8_t temperatureC = 0;
        uint16_t millivolts = 0;
        uint16_t milliamps = 0;
    };
    virtual bool readTelemetry(Telemetry &out)
    {
        (void)out;
        return false;
    }
};