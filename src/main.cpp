#include "logging/logger.hpp"
#include "logging/sinks/mqtt_sink.hpp"
#include "logging/sinks/serial_sink.hpp"

#include "services/mqtt_service.hpp"
#include "services/telemetry_service.hpp"

#include "telemetry/sensors/imu_mpu_9250.hpp"

#include "secrets.hpp"

#include <Arduino.h>

// Priority hierarchy for drone:
// 25: Hardware interrupts/critical safety
// 20: Flight controller main loop
// 18: IMU sampling
// 15: Motor control PWM updates
// 10: Navigation/GPS
// 5:  Telemetry/logging
// 1:  Housekeeping tasks

// ===== Config =================================================================
static const char *DEVICE_ID = "Drone";
static const char *LOG_TOPIC = "log";

static constexpr uint32_t IMU_RATE = 200;         // Hz
static constexpr size_t TELEMETRY_QUEUE_LEN = 64; // Telemetry queue depth
// ==============================================================================

// ===== Accessed Hardware ======================================================
static IMU_MPU9250 imu(/*i2cMutex*/ nullptr, /*rateHz*/ IMU_RATE,
                       /*topicSuffix*/ "telemetry/imu");

// ==============================================================================
void setup()
{
  // ===== Setup Logger (Should always be first) ================================
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }

  // Bring up Wi-Fi + MQTT
  auto &mqtt = MqttService::instance();

  auto &log = Logger::instance();
  log.init(256);
  log.setLevel(LogLevel::Info);

  static SerialSink serial_sink(&Serial);
  log.addSink(&serial_sink);

  // Add MQTT sink (device id is optional; helps when multiple FCs publish)
  static MqttSink mqtt_sink(mqtt, LOG_TOPIC, DEVICE_ID);
  log.addSink(&mqtt_sink);

  LOGI("BOOT", "MQTT sink ready, ip=%s", WiFi.localIP().toString().c_str());

  // ===== Hardware interface initialization ====================================
  Wire.begin(); // Initialize I2C bus

  // ===== Setup MQTT Client ====================================================
  mqtt.begin(secrets::wifi_ssid, secrets::wifi_password);
  if (secrets::mqtt_broker && secrets::mqtt_port)
  {
    mqtt.setServer(secrets::mqtt_broker, secrets::mqtt_port);
  }
  else
  {
    // use default
    mqtt.setServer(IPAddress(192, 168, 1, 200), 1883);
  }

  // ===== Setup Telemetry Service ==============================================
  auto &svc = TelemetryService::instance();
  svc.begin(DEVICE_ID, /*queueLen=*/TELEMETRY_QUEUE_LEN,
            /*txPrio=*/5, /*txStackWords=*/4096, /*txCore=*/tskNO_AFFINITY);

  // Hand shared I2C mutex to providers that use it
  imu.setI2CMutex(svc.i2cMutex());

  // Register IMU provider
  svc.addProvider(&imu);
}

void loop()
{
  // Nothing required here unless you need to pump WiFi/MQTT client, etc.
  // delay to keep the watchdog happy if absolutely idle
  delay(1000);
}
