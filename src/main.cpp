#include "services/mqtt_service.hpp"
#include "logging/logger.hpp"
#include "logging/sinks/mqtt_sink.hpp"
#include "logging/sinks/serial_sink.hpp"

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }

  // Bring up Wi-Fi + MQTT
  auto &mqtt = MqttService::instance();
  mqtt.begin(nullptr, nullptr, IPAddress(192, 168, 1, 10), 1883);
  mqtt.setServer(IPAddress(192, 168, 1, 200), 1883);

  auto &log = Logger::instance();
  log.init(256);
  log.setLevel(LogLevel::Info);

  static SerialSink serial_sink(&Serial);
  log.addSink(&serial_sink);

  // Add MQTT sink (device id is optional; helps when multiple FCs publish)
  static MqttSink mqtt_sink(mqtt, "log", "Drone");
  log.addSink(&mqtt_sink);

  LOGI("BOOT", "MQTT sink ready, ip=%s", WiFi.localIP().toString().c_str());
}

void loop()
{
  static uint32_t last = 0;
  if (millis() - last > 2000)
  {
    last = millis();
    LOGI("HEART", "alive ms=%lu", (unsigned long)millis());
  }
}
