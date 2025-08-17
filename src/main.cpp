#include "logging/logger.hpp"
#include "logging/sinks/mqtt_sink.hpp"
#include "logging/sinks/serial_sink.hpp"

#include "services/mqtt_service.hpp"
#include "services/telemetry_service.hpp"

#include "telemetry/sensors/imu_mpu_9250.hpp"

#include "secrets.hpp"

#include <Arduino.h>

// ===== Accessed Hardware ======================================================
static IMU_MPU9250 imu;
// ==============================================================================

// ===== Config =================================================================
static const char *deviceId = "Drone";
static const char *logTopic = "log";
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
  static MqttSink mqtt_sink(mqtt, logTopic, deviceId);
  log.addSink(&mqtt_sink);

  LOGI("BOOT", "MQTT sink ready, ip=%s", WiFi.localIP().toString().c_str());

  // ===== Hardware interface initialization ====================================
  Wire.begin(); // Initialize I2C bus

  // ===== Setup MQTT Client ====================================================
  mqtt.begin(nullptr, nullptr);
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
  auto &telem = TelemetryService::instance();
  telem.addProvider(&imu);
  telem.begin(deviceId, 100); // 100Hz tick rate
}

void loop() {}
