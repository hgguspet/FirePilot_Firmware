#include "logging/logger.hpp"
#include "logging/sinks/serial_sink.hpp"
#include "logging/sinks/mqtt_sink.hpp"

#include "services/mqtt_service.hpp"
#include "services/telemetry_service.hpp"
#include "telemetry/sensors/imu_mpu_9250.hpp"

#include "drivers/esc/pwm.hpp"

#include "secrets.hpp"
#include <cmath>

// ===== Config =================================================================
static const char *DEVICE_ID = "Drone";
static const char *LOG_TOPIC = "log";
static const char *SERVO_TOPIC = "Drone/servo";

static constexpr uint32_t IMU_RATE = 100;         // Hz
static constexpr size_t TELEMETRY_QUEUE_LEN = 64; // Telemetry queue depth

static const uint8_t SERVO_PIN = 32;
// ==============================================================================

// ===== Hardware ===============================================================
PwmDriver Servo;
static float ServoTarget;

static IMU_MPU9250 imu(/*i2cMutex*/ nullptr, /*rateHz*/ IMU_RATE,
                       /*topicSuffix*/ "telemetry/imu");
// ==============================================================================

static void onServoUpdate(Message msg)
{
  // Try reading value as float
  float target = atof((const char *)msg.payload);
  if (isnan(target))
  {
    LOGI("Servo", "Received Invalid Target");
    return;
  }
  target = std::clamp(target, 0.f, 1.f);
  LOGI("Servo", "Received Target: %.2f", target);
  ServoTarget = target;
}

void setup()
{
  // ===== Setup Logger (Should always be first) ================================
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }

  auto &mqtt = MqttService::instance();
  auto &log = Logger::instance();
  log.init(256);
  log.setLevel(LogLevel::Info);

  static SerialSink serial_sink(&Serial);
  log.addSink(&serial_sink);

  // Set MQTT server BEFORE Wi-Fi connect to avoid a first-connect miss
  mqtt.setServer(secrets::mqtt_broker, secrets::mqtt_port);

  // Add MQTT sink (optional device id tag)
  static MqttSink mqtt_sink(mqtt, LOG_TOPIC, DEVICE_ID);
  log.addSink(&mqtt_sink);

  LOGI("BOOT", "Starting, ip=%s", WiFi.localIP().toString().c_str());

  // ===== Setup MQTT Client ====================================================
  mqtt.begin(secrets::wifi_ssid, secrets::wifi_password);

  // Subscribe to ARM topic with per-topic callback
  mqtt.subscribe(SERVO_TOPIC, /*QoS*/ 0, onServoUpdate);

  // ===== Hardware interface initialization ====================================
  Wire.begin(); // Initialize I2C bus

  // ===== Setup Telemetry Service ==============================================W
  auto &telem = TelemetryService::instance();
  telem.begin(
      DEVICE_ID, /*queueLen=*/TELEMETRY_QUEUE_LEN,
      /*txPrio=*/5, /*txStackWords=*/4096, /*txCore=*/tskNO_AFFINITY);
  imu.setI2CMutex(telem.i2cMutex());
  telem.addProvider(&imu);

  // ===== Esc & Servo Setup ====================================================
  Servo.setMinPulseUs(544);
  Servo.setMaxPulseUs(2400);
  // Servo.setZeroThrottleValue(0.5f); // Centered servo
  ServoTarget = 0.5f;
  Servo.arm(true);
  if (!Servo.begin(SERVO_PIN, /*~50Hz is typical for servos*/ 50))
  {
    for (;;)
    {
      LOGE("Servo", "Servo setup failed");
      delay(1000);
    }
  }
  LOGI("Servo", "Servo ready on gpio pin: %u", SERVO_PIN);
}

void loop()
{
  Servo.writeNormalized(ServoTarget);
  delayMicroseconds(500); // Fine for most drivers
}
