#include <Arduino.h>
#include <Wire.h>
#include "services/mqtt_service.hpp"
// #include "services/telemetry_service.hpp"
// #include "telemetry/sensors/imu_mpu_9250.hpp"
#include "drivers/iesc_driver.hpp"
#include "drivers/esc/d_shot_600.hpp"

static DShot600Driver esc1;
// static IMU_MPU9250 imu; // Global IMU instance

// ---------- config ----------
static const char *deviceId = "Drone";
static const char *escTopic = "Drone/manual/esc";
static const int ESC_PIN = 32;    // change if you want another pin
static const int ESC_RATE = 2000; // 2 kHz DShot600
// ----------------------------

static volatile float g_targetNorm = 0.0f;

static String payloadToString(const uint8_t *data, size_t len)
{
  String s;
  s.reserve(len + 1);
  for (size_t i = 0; i < len; ++i)
    s += (char)data[i];
  s.trim();
  return s;
}

static bool tryParseNorm01(const String &in, float &out)
{
  String s = in;
  s.trim();
  s.replace(',', '.');
  int first = -1, last = -1;
  for (int i = 0; i < (int)s.length(); ++i)
  {
    char c = s[i];
    if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
    {
      first = i;
      break;
    }
  }
  if (first >= 0)
  {
    last = first;
    while (last < (int)s.length())
    {
      char c = s[last];
      if ((c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' || c == '-' || c == '+')
      {
        ++last;
      }
      else
        break;
    }
    String num = s.substring(first, last);
    float v = num.toFloat();
    if (!isnan(v))
    {
      if (v < 0.f)
        v = 0.f;
      if (v > 1.f)
        v = 1.f;
      out = v;
      return true;
    }
  }
  return false;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Wire.begin();
  Wire.setClock(400000);

  delay(300);

  auto &mqtt = MqttService::instance();
  mqtt.setServer(IPAddress(192, 168, 1, 200), 1883);
  mqtt.begin(nullptr, nullptr);
  mqtt.subscribe(escTopic);

  mqtt.onMessage([](const char *topic,
                    const uint8_t *data,
                    size_t len,
                    const AsyncMqttClientMessageProperties &props)
                 {
    (void)props;
    if (strcmp(topic, escTopic) != 0) return;

    String msg = payloadToString(data, len);
    Serial.printf("MQTT <%s>: \"%s\"\n", topic, msg.c_str());

    if (msg.equalsIgnoreCase("arm"))    { esc1.arm(true);  Serial.println("ESC: armed"); g_targetNorm = 0.0f; return; }
    if (msg.equalsIgnoreCase("disarm")) { esc1.arm(false); Serial.println("ESC: disarmed"); g_targetNorm = 0.0f; return; }

    float v;
    if (tryParseNorm01(msg, v)) {
      g_targetNorm = v;
      Serial.printf("ESC: throttle=%.3f\n", (double)v);
    } else {
      Serial.println("ESC: ignored payload");
    } });

  // Telemetry disabled:
  // auto &telem = TelemetryService::instance();
  // telem.addProvider(&imu);
  // telem.begin(deviceId, 100);

  if (!esc1.begin(ESC_PIN, ESC_RATE))
  {
    Serial.println("ESC init failed");
    for (;;)
      delay(1000);
  }
  esc1.arm(true);

  uint32_t t0 = millis();
  while (millis() - t0 < 2000)
  {
    esc1.writeNormalized(0.0f);
    yield();
  }

  Serial.printf("ESC ready on GPIO %d @ %d Hz. MQTT topic: %s\n", ESC_PIN, ESC_RATE, escTopic);
}

void loop()
{
  float target = g_targetNorm;
  esc1.writeNormalized(target);

  IEscDriver::Telemetry tlm;
  esc1.readTelemetry(tlm);

  Serial.printf("ESC telemetry: %d %d %d %d\n", tlm.rpm, tlm.milliamps, tlm.millivolts, tlm.temperatureC);

  delayMicroseconds(500); // ≈ 2 kHz DShot command rate
  // or: delayMicroseconds(250); // ≈ 4 kHz
}
