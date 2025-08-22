#pragma once
#include <cstdint>
#include <cmath>

// ===== Feature flags (bitmask) ===============================================
typedef uint32_t DcFeature;

enum class DcFeatures : DcFeature
{
    DirectionPins = 1u << 0,   // driver exposes dir control (H-bridge)
    SignedCommand = 1u << 1,   // accepts [-1..+1] directly
    BrakeCommand = 1u << 2,    // can command active brake
    CoastCommand = 1u << 3,    // can command coast (high-Z / freewheel)
    FreqAdjustable = 1u << 4,  // PWM frequency can be changed at runtime
    CurrentSense = 1u << 5,    // read current (mA)
    VoltageSense = 1u << 6,    // read bus voltage (mV)
    EncoderFeedback = 1u << 7, // can report rpm via encoder/counter
};

inline constexpr DcFeature operator|(DcFeatures a, DcFeatures b)
{
    return DcFeature(DcFeature(a) | DcFeature(b));
}

inline DcFeature &operator|=(DcFeature &a, DcFeatures b)
{
    a |= DcFeature(b);
    return a;
}
inline constexpr bool hasFeature(DcFeature mask, DcFeatures f)
{
    return (mask & DcFeature(f)) != 0;
}

// Conservative maximums for planning
struct DcPwmFreqHz
{
    uint32_t value = 0; // 0 = unspecified
    DcPwmFreqHz() = default;
    explicit DcPwmFreqHz(uint32_t v) : value(v) {}
};

struct DcCapabilities
{
    DcFeature features = 0;        // bitmask of DcFeatures
    DcPwmFreqHz maxFreqHz;         // e.g. 40 kHz (LEDC @ 12-bit), 300 kHz (MCPWM)
    uint8_t maxResolutionBits = 0; // e.g. 12 for LEDC at typical motor freqs
};

// ===== Interface ==============================================================
class IDcDriver
{
public:
    virtual ~IDcDriver() = default;

    // Lifecycle
    //
    // Min config for DC is just a PWM output pin + frequency.
    // Implementations may accept additional pins (dir/brake) via their own .begin(...) overloads
    // or a separate configure()—but this minimal signature must work for one-pin PWM drivers.
    virtual bool begin(uint8_t pwmPin, uint32_t pwmFreqHz) = 0;
    virtual void end() = 0;

    // Capabilities
    virtual DcCapabilities caps() const = 0;

    // Control path
    //
    // 0..1 duty (unsigned). For H-bridge drivers, this means forward (dir pin handled internally).
    // Implementations should clamp safely.
    virtual void writeNormalized(float duty01) = 0;

    // Optional signed command in [-1..+1].
    // Default adapter maps to writeNormalized() if SignedCommand is not supported.
    virtual void writeSigned(float dutySigned)
    {
        // Default fallback: directionless (low-side) => sign is ignored.
        writeNormalized(std::fmin(std::fmax(std::fabs(dutySigned), 0.0f), 1.0f));
    }

    // Safe minimum/maximum duty clamp points (defaults 0 and 1).
    virtual void setOutputLimits(float /*min01*/, float /*max01*/) {}

    // Arm / enable output stage (default no-op).
    // Drivers that gate an EN pin or power stage should honor this.
    virtual void arm(bool /*on*/) {}

    // Adjust PWM frequency at runtime (if supported).
    virtual void setUpdateRate(uint32_t /*pwmFreqHz*/) {}

    // Braking and coasting (optional)
    virtual bool setBrake(bool /*on*/) { return false; } // true if applied
    virtual bool coast() { return false; }               // high-Z/freewheel, true if applied

    // Batch hooks (optional)
    virtual void beginFrame() {}
    virtual void endFrame() {}

    // Telemetry (optional)
    struct Telemetry
    {
        bool valid = false;
        uint16_t rpm = 0;          // if encoder available
        uint16_t temperatureC = 0; // driver/board temp if sensed
        uint16_t millivolts = 0;   // supply/bus voltage
        uint16_t milliamps = 0;    // motor current
    };
    virtual bool readTelemetry(Telemetry &out)
    {
        (void)out;
        return false;
    }
};
