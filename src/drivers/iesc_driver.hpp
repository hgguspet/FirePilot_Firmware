#pragma once
#include <cstdint>

// Capability flags (bitmask) so app can discover extras at runtime
enum class EscFeature : uint32_t
{
    Telemetry = 1u << 0,    // readTelemetry() returns real data
    DirectionCmd = 1u << 1, // supports runtime direction changes
    Mode3D = 1u << 2,       // supports 3D (center = stop; +/- = fwd/rev)
    Brake = 1u << 3,        // can command active braking
    Beeper = 1u << 4,       // can beep via command
};

inline constexpr EscFeature operator|(EscFeature a, EscFeature b)
{
    return EscFeature(uint32_t(a) | uint32_t(b));
}
inline constexpr bool hasFeature(uint32_t mask, EscFeature f)
{
    return (mask & uint32_t(f)) != 0;
}

struct EscCapabilities
{
    uint32_t features = 0;   // bitmask of EscFeature
    bool needsCalibrate : 1; // analog/PWM ESCs often do
    bool bidirTelemetry : 1; // true if telemetry wire/command supported
    uint16_t maxRateHz;      // conservative max command rate (e.g. 2000 for OS125, 4000 for DShot)
};

class IEscDriver
{
public:
    virtual ~IEscDriver() = default;

    // Lifecycle
    virtual bool begin(uint8_t pin, uint16_t rateHz) = 0; // pin is GPIO number
    virtual void end() = 0;
    virtual EscCapabilities caps() const = 0;

    // Control path (hot). norm in [0..1]
    virtual void writeNormalized(float norm01) = 0;

    // Optional, but common across protocols
    virtual void arm(bool on) { (void)on; } // default no-op
    virtual void setUpdateRate(uint16_t rateHz) = 0;

    // Batch/sync hooks (optional)
    virtual void beginFrame() {}
    virtual void endFrame() {}

    // Telemetry (optional)
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

    // ---- Optional protocol-specific helpers (default no-op) ----
    // DShot: 0..47 special commands. Returns true if accepted.
    virtual bool sendSpecial(uint16_t /*code*/) { return false; }

    // High-level helpers that drivers may implement using protocol commands.
    // Return true if supported & applied.
    virtual bool setDirection(bool /*reversed*/) { return false; } // DShot: 20/21 (+23 save)
    virtual bool set3DMode(bool /*enable*/) { return false; }      // DShot or BLHeli config
    virtual bool setBrake(bool /*enable*/) { return false; }       // if supported
};
