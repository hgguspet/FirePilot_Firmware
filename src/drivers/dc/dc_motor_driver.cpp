#include "dc_motor_driver.hpp"
#include <cmath>

void DcMotorDriver::configurePins(int dirPin, int enPin, int brakePin)
{
    _pin_dir = dirPin;
    _pin_en = enPin;
    _pin_brake = brakePin;
}

bool DcMotorDriver::begin(uint8_t pwmPin, uint32_t pwmFreqHz)
{
    if (_initalized)
        return true;

    if (!ch_.acquire_any())
        return false;

    _pin_pwm = static_cast<int>(pwmPin);
    _freq_hz = (pwmFreqHz == 0) ? 20000u : pwmFreqHz; // default 20 kHz

    // Configure optional pins
    if (_pin_dir >= 0)
    {
        pinMode(_pin_dir, OUTPUT);
        digitalWrite(_pin_dir, _dir_inverted ? HIGH : LOW);
    }
    if (_pin_en >= 0)
    {
        pinMode(_pin_en, OUTPUT);
        digitalWrite(_pin_en, LOW);
    } // disabled until arm()
    if (_pin_brake >= 0)
    {
        pinMode(_pin_brake, OUTPUT);
        digitalWrite(_pin_brake, LOW);
    } // brake off

    // LEDC setup
    if (!ledcalloc::init())
        return false;

    ledc_channel_t ch = ch_.get();
    if (ch < 0 || ch >= LEDC_CHANNEL_MAX)
        return false;

    // Set up PWM channel
    if (ledcSetup(ch, _freq_hz, _res_bits) == 0.0) // returns actual freq; 0 on error
        return false;

    ledcAttachPin(_pin_pwm, ch);

    _max_duty = (1u << _res_bits) - 1u;
    ledcWrite(ch, 0);

    _initalized = true;
    // Stay disarmed until arm(true); keeps output idle
    return true;
}

void DcMotorDriver::end()
{
    if (!_initalized)
        return;

    ledc_channel_t ch = ch_.get();
    if (ch >= 0 && ch < LEDC_CHANNEL_MAX)
    {
        // Drive output safe
        ledcWrite(ch, 0);
        // Detach to release GPIO routing
        ledcDetachPin(_pin_pwm);
    }
    ch_.release();

    // Optionally deassert pins
    if (_pin_en >= 0)
        digitalWrite(_pin_en, LOW);
    if (_pin_brake >= 0)
        digitalWrite(_pin_brake, LOW);

    _initalized = false;
    _armed = false;
    _pin_pwm = -1;
    _freq_hz = 0;
    _max_duty = 0;
}

DcCapabilities DcMotorDriver::caps() const
{
    DcFeature f = 0;
    if (_pin_dir >= 0)
        f |= DcFeatures::DirectionPins;
    if (_pin_brake >= 0)
        f |= DcFeatures::BrakeCommand;
    // We support changing frequency at runtime via ledcSetup
    f |= DcFeatures::FreqAdjustable;

    // Typical safe limits for LEDC @ ~20 kHz
    DcCapabilities c{};
    c.features = f;
    c.maxFreqHz = DcPwmFreqHz(40000); // practical upper bound for many setups
    c.maxResolutionBits = _res_bits;
    return c;
}

void DcMotorDriver::setOutputLimits(float min01, float max01)
{
    if (std::isnan(min01) || std::isnan(max01))
        return;
    if (min01 > max01)
        std::swap(min01, max01);
    _out_min = fmaxf(0.0f, fminf(min01, 1.0f));
    _out_max = fmaxf(0.0f, fminf(max01, 1.0f));
}

void DcMotorDriver::arm(bool on)
{
    _armed = on;
    if (_pin_en >= 0)
        digitalWrite(_pin_en, on ? HIGH : LOW);

    // When disarming, force duty=0
    if (!on)
    {
        ledc_channel_t ch = ch_.get();
        if (ch >= 0 && ch < LEDC_CHANNEL_MAX)
        {
            ledcWrite(ch, 0);
        }
    }
}

void DcMotorDriver::setUpdateRate(uint32_t pwmFreqHz)
{
    if (!_initalized || pwmFreqHz == 0)
        return;

    _freq_hz = pwmFreqHz;
    ledc_channel_t ch = ch_.get();
    if (ch < 0 || ch >= LEDC_CHANNEL_MAX)
        return;

    // Re-setup channel to new frequency with same resolution
    if (ledcSetup(ch, _freq_hz, _res_bits) == 0.0)
    {
        // failed: keep old freq
        return;
    }
    _max_duty = (1u << _res_bits) - 1u;
    // Keep current duty ratio by reading back (Arduino core lacks read; we could track last duty)
    // For simplicity, leave duty as-is (hardware keeps it).
}

void DcMotorDriver::ensureAttached_()
{
    // Arduino core ledcAttachPin is idempotent for the same mapping; guard anyway.
    // (No-op here; kept for parity with other backends.)
}

void DcMotorDriver::applyDutyRaw_(uint32_t duty)
{
    if (!_initalized)
        return;
    ledc_channel_t ch = ch_.get();
    if (ch < 0 || ch >= LEDC_CHANNEL_MAX)
        return;

    // Clamp and write
    if (duty > _max_duty)
        duty = _max_duty;
    ledcWrite(ch, duty);
}

void DcMotorDriver::writeNormalized(float duty01)
{
    if (!_initalized)
        return;

    // Disarmed => force output idle
    if (!_armed)
    {
        applyDutyRaw_(0);
        return;
    }

    if (std::isnan(duty01))
        duty01 = 0.0f;
    duty01 = fminf(fmaxf(duty01, _out_min), _out_max);

    // Fast paths for 0% / 100%
    if (duty01 <= 0.0f + 1e-6f)
    {
        applyDutyRaw_(0);
        return;
    }
    if (duty01 >= 1.0f - 1e-6f)
    {
        applyDutyRaw_(_max_duty);
        return;
    }

    uint32_t duty = static_cast<uint32_t>(lrintf(duty01 * float(_max_duty)));
    applyDutyRaw_(duty);
}

void DcMotorDriver::updateDirFromSigned_(float dutySigned)
{
    if (_pin_dir < 0)
        return;
    bool forward = (dutySigned >= 0.0f);
    bool level = _dir_inverted ? !forward : forward;
    digitalWrite(_pin_dir, level ? HIGH : LOW);
}

void DcMotorDriver::writeSigned(float dutySigned)
{
    if (!_initalized)
        return;

    // Map sign to direction (if present), magnitude to duty
    float mag = std::fabs(dutySigned);
    if (mag > 1.0f)
        mag = 1.0f;

    if (_pin_dir >= 0)
    {
        updateDirFromSigned_(dutySigned);
        writeNormalized(mag);
    }
    else
    {
        // No direction pin -> behave like unsigned driver
        writeNormalized(mag);
    }
}

bool DcMotorDriver::setBrake(bool on)
{
    if (_pin_brake < 0)
        return false;
    digitalWrite(_pin_brake, on ? HIGH : LOW);
    if (on)
        applyDutyRaw_(0);
    return true;
}

bool DcMotorDriver::coast()
{
    // For a low-side MOSFET stage, "coast" is typically just duty=0
    // without asserting brake. If an EN pin exists, disable it.
    if (!_initalized)
        return false;

    if (_pin_en >= 0)
    {
        digitalWrite(_pin_en, LOW);
    }
    applyDutyRaw_(0);
    return true;
}