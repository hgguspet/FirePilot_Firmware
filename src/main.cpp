#include "logging/logger.hpp"
#include "logging/pinger.hpp"
#include "logging/sinks/serial_sink.hpp"
#include "logging/sinks/mqtt_sink.hpp"

#include "services/mqtt_service.hpp"
#include "services/telemetry_service.hpp"
#include "telemetry/sensors/imu_mpu_9250.hpp"

#include "drivers/esc/pwm.hpp"
#include "drivers/dc/dc_motor_driver.hpp"

#include "secrets.hpp"
#include <cmath>
#include <algorithm>

// ===== Config =================================================================
static const char *DEVICE_ID = "guspet24";
static const char *SERVO_TOPIC = "servo";
static const char *MOTOR_TOPIC = "motor";

static const uint8_t SERVO_PIN = 32;
static const uint8_t MOTOR_IN_1 = 33;
static const uint8_t MOTOR_IN_2 = 25;

static constexpr uint32_t IMU_RATE = 100; // Hz (lower rates may cause problems)
static constexpr size_t TELEMETRY_QUEUE_LEN = 64;
// ==============================================================================

// ===== Hardware ===============================================================
static DcMotorDriver Motor;
static PwmDriver Servo;

static volatile float MotorTarget = 0.0f;
static volatile float ServoTarget = 0.5f;

static IMU_MPU9250 imu(
    /*i2cMutex*/ nullptr, /*rateHz*/ IMU_RATE,
    /*topicSuffix*/ "telemetry/imu");
//  ==============================================================================

static void onMotorUpdate(MqttService::Message msg)
{
  // copy the payload to a null-terminated buffer, to ensure mqtt can't mess with it
  char buf[16];
  strncpy(buf, reinterpret_cast<const char *>(msg.payload), std::min<size_t>(15, msg.len));
  buf[std::min<size_t>(15, msg.len)] = '\0';

  LOGI("MOTOR", "Received motor update: %s", buf);
  float val = atof(buf);

  // Validation
  if (isnan(val) || val < -1.0f || val > 1.0f)
  {
    LOGW("MOTOR", "Invalid motor value: %f", val);
    return;
  }
  LOGI("MOTOR", "Motor target: %f", val);
  MotorTarget = val;
}

static void onServoUpdate(MqttService::Message msg)
{
  char buf[16];
  strncpy(buf, reinterpret_cast<const char *>(msg.payload), std::min<size_t>(15, msg.len));
  buf[std::min<size_t>(15, msg.len)] = '\0';

  float val = atof(buf);

  // Validation
  if (isnan(val) || val < 0.0f || val > 1.0f)
  {
    LOGW("SERVO", "Invalid servo value: %f", val);
    return;
  }
  LOGI("SERVO", "Servo target: %f", val);
  ServoTarget = val;
}

void setup()
{
  // ===== Setup Logger (Should always be first) ================================
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }

  auto &mqtt = MqttService::MqttService::instance();
  auto &log = Logger::instance();
  auto &pinger = Pinger::instance(/*1Hz*/ 1000);
  log.init(256);
  log.setMinLevel(LogLevel::Debug);

  static SerialSink serial_sink(&Serial);
  static MqttSink mqtt_sink(mqtt);

  // Logging
  log.addSink(&serial_sink);
  log.addSink(&mqtt_sink);

  // pinger to prove active connection
  pinger.addSink(&mqtt_sink);
  pinger.begin();

  // ===== Setup MQTT Client ====================================================
  mqtt.setServer(secrets::mqtt_broker, secrets::mqtt_port);
  LOGI("BOOT", "Starting, ip=%s", WiFi.localIP().toString().c_str());
  mqtt.begin(secrets::wifi_ssid, secrets::wifi_password, DEVICE_ID);
  mqtt.subscribeRel(MOTOR_TOPIC, /*QoS*/ MqttService::QoS::ExactlyOnce, onMotorUpdate);
  mqtt.subscribeRel(SERVO_TOPIC, /*QoS*/ MqttService::QoS::ExactlyOnce, onServoUpdate);

  // ===== Hardware interface initialization ====================================
  Wire.begin(); // Initialize I2C bus

  // ===== Setup Telemetry Service ==============================================
  auto &telem = TelemetryService::instance();
  telem.begin(
      DEVICE_ID, /*queueLen=*/TELEMETRY_QUEUE_LEN,
      /*txPrio=*/5, /*txStackWords=*/4096, /*txCore=*/tskNO_AFFINITY);
  imu.setI2CMutex(telem.i2cMutex());
  telem.addProvider(&imu);

  // ===== Setup Motor & Servo ===================================================
  Motor.arm(true);
  Motor.configureDualInputs(MOTOR_IN_1, MOTOR_IN_2, /*en*/ -1);
  if (!Motor.begin(/*tied on L298N */ -1, /*good enough */ 50))
  {
    for (;;)
    {
      LOGC("MOTOR", "Failed to initialize motor");
    }
  }
  LOGI("MOTOR", "Motor driver initialized on pins %d, %d", MOTOR_IN_1, MOTOR_IN_2);

  Servo.setMinPulseUs(544);
  Servo.setMaxPulseUs(2400);
  Servo.setZeroThrottleValue(0.5f); // Be careful if using this on other things!
  Servo.arm(true);
  if (!Servo.begin(SERVO_PIN, /*Typical servo frequency*/ 50))
  {
    for (;;)
    {
      LOGC("SERVO", "Failed to initialize servo");
    }
  }
  LOGI("SERVO", "Servo driver initialized on pin %d", SERVO_PIN);
}

void loop()
{
  if (MotorTarget == 0.0f) // feels like coasting should happen automatically
                           // but since this works and FirePilot isn't really designed to drive dc motors especially without
                           // pwm, this is fine for now
  {
    Motor.coast();
  }
  else
  {
    Motor.writeSigned(MotorTarget); // signed for direction control
  }
  Servo.writeNormalized(ServoTarget);
  delayMicroseconds(500); // ~2kHz
}