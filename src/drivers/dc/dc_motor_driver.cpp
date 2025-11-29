#include "dc_motor_driver.hpp"
#include <cmath>

void DcMotorDriver::configurePins(int dirPin, int enPin, int brakePin)
{
    _pin_dir = dirPin;
    _pin_en = enPin;
    _pin_brake = brakePin;

    _dir_mode = (_pin_dir >= 0) ? DirMode::SingleDir : _dir_mode;
}

void DcMotorDriver::configureDualInputs(int in1Pin, int in2Pin, int enPin)
{
    _pin_in1 = in1Pin;
    _pin_in2 = in2Pin;
    _pin_en = enPin;
    _dir_mode = (_pin_in1 >= 0 && _pin_in2 >= 0) ? DirMode::DualInputs : _dir_mode;
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
    if (_dir_mode == DirMode::SingleDir && _pin_dir >= 0)
    {
        pinMode(_pin_dir, OUTPUT);
        digitalWrite(_pin_dir, _dir_inverted ? HIGH : LOW);
    }
    if (_dir_mode == DirMode::DualInputs)
    {
        if (_pin_in1 >= 0)
        {
            pinMode(_pin_in1, OUTPUT);
            digitalWrite(_pin_in1, LOW);
        }
        if (_pin_in2 >= 0)
        {
            pinMode(_pin_in2, OUTPUT);
            digitalWrite(_pin_in2, LOW);
        }
    }
    if (_pin_en >= 0)
    {
        pinMode(_pin_en, OUTPUT);
        digitalWrite(_pin_en, LOW); // disabled until arm()
    }
    if (_pin_brake >= 0)
    {
        pinMode(_pin_brake, OUTPUT);
        digitalWrite(_pin_brake, LOW); // brake off
    }

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
        ledcWrite(ch, 0);
        ledcDetachPin(_pin_pwm);
    }
    ch_.release();

    // Safe all pins
    if (_pin_en >= 0)
        digitalWrite(_pin_en, LOW);
    if (_pin_brake >= 0)
        digitalWrite(_pin_brake, LOW);
    if (_dir_mode == DirMode::SingleDir && _pin_dir >= 0)
        digitalWrite(_pin_dir, LOW);
    if (_dir_mode == DirMode::DualInputs)
    {
        if (_pin_in1 >= 0)
            digitalWrite(_pin_in1, LOW);
        if (_pin_in2 >= 0)
            digitalWrite(_pin_in2, LOW);
    }

    _initalized = false;
    _armed = false;
    _pin_pwm = -1;
    _freq_hz = 0;
    _max_duty = 0;
}

DcCapabilities DcMotorDriver::caps() const
{
    DcFeature f = 0;
    if (_dir_mode == DirMode::SingleDir || _dir_mode == DirMode::DualInputs)
        f |= DcFeatures::DirectionPins;
    if (_pin_brake >= 0 || _dir_mode == DirMode::DualInputs)
        f |= DcFeatures::BrakeCommand; // dual inputs can active-brake
    f |= DcFeatures::FreqAdjustable;

    DcCapabilities c{};
    c.features = f;
    c.maxFreqHz = DcPwmFreqHz(40000);
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

    if (!on)
    {
        // Idle/safe outputs
        if (_dir_mode == DirMode::DualInputs)
            driveDualInputs_(false, false); // coast
        applyDutyRaw_(0);
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

    if (ledcSetup(ch, _freq_hz, _res_bits) == 0.0)
        return;

    _max_duty = (1u << _res_bits) - 1u;
}

void DcMotorDriver::ensureAttached_()
{
    // kept for parity with other backends (no-op)
}

void DcMotorDriver::applyDutyRaw_(uint32_t duty)
{
    if (!_initalized)
        return;
    ledc_channel_t ch = ch_.get();
    if (ch < 0 || ch >= LEDC_CHANNEL_MAX)
        return;

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

void DcMotorDriver::driveDualInputs_(bool in1, bool in2)
{
    if (_dir_mode != DirMode::DualInputs)
        return;
    if (_pin_in1 >= 0)
        digitalWrite(_pin_in1, in1 ? HIGH : LOW);
    if (_pin_in2 >= 0)
        digitalWrite(_pin_in2, in2 ? HIGH : LOW);
}

void DcMotorDriver::updateDirFromSigned_(float dutySigned)
{
    switch (_dir_mode)
    {
    case DirMode::SingleDir:
    {
        if (_pin_dir < 0)
            return;
        bool forward = (dutySigned >= 0.0f);
        bool level = _dir_inverted ? !forward : forward;
        digitalWrite(_pin_dir, level ? HIGH : LOW);
        break;
    }
    case DirMode::DualInputs:
    {
        // For dual inputs, encode direction via IN1/IN2
        bool forward = (dutySigned >= 0.0f);
        if (_dir_inverted)
            forward = !forward;
        if (forward)
            driveDualInputs_(true, false); // IN1=H, IN2=L
        else
            driveDualInputs_(false, true); // IN1=L, IN2=H
        break;
    }
    case DirMode::None:
    default:
        // nothing to do
        break;
    }
}

void DcMotorDriver::writeSigned(float dutySigned)
{
    if (!_initalized)
        return;

    float mag = std::fabs(dutySigned);
    if (mag > 1.0f)
        mag = 1.0f;

    if (_dir_mode == DirMode::SingleDir || _dir_mode == DirMode::DualInputs)
    {
        updateDirFromSigned_(dutySigned);
        writeNormalized(mag);
    }
    else
    {
        // No direction pins -> behave like unsigned driver
        writeNormalized(mag);
    }
}

bool DcMotorDriver::setBrake(bool on)
{
    // Prefer dedicated brake pin if present
    if (_pin_brake >= 0)
    {
        digitalWrite(_pin_brake, on ? HIGH : LOW);
        if (on)
            applyDutyRaw_(0);
        return true;
    }

    // Dual input bridges can do active brake (IN1=IN2=HIGH)
    if (_dir_mode == DirMode::DualInputs)
    {
        if (on)
        {
            driveDualInputs_(true, true); // fast brake
            applyDutyRaw_(0);
        }
        else
        {
            // Release brake -> default to coast until next command
            driveDualInputs_(false, false);
        }
        return true;
    }

    // Single-dir only cannot actively brake without extra hardware
    return false;
}

bool DcMotorDriver::coast()
{
    if (!_initalized)
        return false;

    if (_pin_en >= 0)
        digitalWrite(_pin_en, LOW);
    applyDutyRaw_(0);

    // Dual input freewheel: IN1=IN2=LOW
    if (_dir_mode == DirMode::DualInputs)
        driveDualInputs_(false, false);

    return true;
}
