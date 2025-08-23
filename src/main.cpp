#include "logging/logger.hpp"
#include "logging/sinks/serial_sink.hpp"
#include "logging/sinks/mqtt_sink.hpp"

#include "services/mqtt_service.hpp"
#include "services/telemetry_service.hpp"
#include "telemetry/sensors/imu_mpu_9250.hpp"

#include "drivers/esc/pwm.hpp"
#include "drivers/dc/dc_motor_driver.hpp"

#include "secrets.hpp"
#include <cmath>

// ===== Config =================================================================
static const char *DEVICE_ID = "Drone";
static const char *LOG_TOPIC = "log";
static const char *SERVO_TOPIC = "servo";
static const char *MOTOR_TOPIC = "motor";

static constexpr uint32_t IMU_RATE = 100;         // Hz
static constexpr size_t TELEMETRY_QUEUE_LEN = 64; // Telemetry queue depth

static const uint8_t SERVO_PIN = 32;
static const uint8_t MOTOR_PIN = 25;
// ==============================================================================

// ===== Hardware ===============================================================
static DcMotorDriver Motor;
static PwmDriver Servo;
static volatile float MotorTarget = 0.0f;
static volatile float ServoTarget = 0.5f;
// static IMU_MPU9250 imu(
//     /*i2cMutex*/ nullptr, /*rateHz*/ IMU_RATE,
//     /*topicSuffix*/ "telemetry/imu");
//  ==============================================================================

static void onMotorUpdate(MqttService::Message msg)
{
  float val = atof(reinterpret_cast<const char *>(msg.payload));

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
  float val = atof(reinterpret_cast<const char *>(msg.payload));

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
  log.init(256);
  log.setMinLevel(LogLevel::Info);

  static SerialSink serial_sink(&Serial);
  log.addSink(&serial_sink);

  // Set MQTT server BEFORE Wi-Fi connect to avoid a first-connect miss
  mqtt.setServer(secrets::mqtt_broker, secrets::mqtt_port);

  // Add MQTT sink
  static MqttSink mqtt_sink(mqtt, LOG_TOPIC);
  log.addSink(&mqtt_sink);

  LOGI("BOOT", "Starting, ip=%s", WiFi.localIP().toString().c_str());

  // ===== Setup MQTT Client ====================================================
  mqtt.begin(secrets::wifi_ssid, secrets::wifi_password, DEVICE_ID);
  mqtt.subscribeRel(MOTOR_TOPIC, /*QoS*/ MqttService::QoS::ExactlyOnce, onMotorUpdate);
  mqtt.subscribeRel(SERVO_TOPIC, /*QoS*/ MqttService::QoS::ExactlyOnce, onServoUpdate);

  // ===== Hardware interface initialization ====================================
  Wire.begin(); // Initialize I2C bus

  // ===== Setup Telemetry Service ==============================================W
  // auto &telem = TelemetryService::instance();
  // telem.begin(
  //    DEVICE_ID, /*queueLen=*/TELEMETRY_QUEUE_LEN,
  //    /*txPrio=*/5, /*txStackWords=*/4096, /*txCore=*/tskNO_AFFINITY);
  // imu.setI2CMutex(telem.i2cMutex());
  // telem.addProvider(&imu);

  // ===== Setup Motor Driver ================================================
  Motor.arm(true);
  if (!Motor.begin(MOTOR_PIN, /*good enough */ 50))
  {
    for (;;)
    {
      LOGC("MOTOR", "Failed to initialize motor");
    }
  }
  LOGI("MOTOR", "Motor driver initialized on pin %d", MOTOR_PIN);

  Servo.setMinPulseUs(1000);
  Servo.setMaxPulseUs(2000);
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
  Motor.writeNormalized(MotorTarget);
  Servo.writeNormalized(ServoTarget);
  delayMicroseconds(500); // ~2kHz
}