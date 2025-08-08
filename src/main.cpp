#include <Arduino.h>
#include "services/mqtt_service.hpp"
#include "services/telemetry_service.hpp"
#include "telemetry/sensors/imu_mpu_9250.hpp"
#include "telemetry/encoders/json_encoder.hpp"

static JsonEncoder jsonEnc(256);
static IMU_MPU9250 imu(jsonEnc);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(100);
  delay(1000); // Give time for Serial to stabilize

  // MQTT setup
  MqttService::instance().setServer(IPAddress(192, 168, 1, 200), 1883);
  MqttService::instance().begin(nullptr, nullptr);

  MqttService::instance().onMessage([](const char *topic, const uint8_t *data, size_t len,
                                       const AsyncMqttClientMessageProperties &props)
                                    { Serial.printf("RX %s (%u bytes, qos=%u)\n", topic, (unsigned)len, props.qos); });

  // Telemetry setup
  auto &telem = TelemetryService::instance();
  telem.addProvider(&imu);
  telem.begin("Drone", 100); // telemetry @ 100Hz
}

void loop()
{
  // MqttService::instance().publish("Drone/test/topic", "Hello, world!");
  // delay(1000);
}