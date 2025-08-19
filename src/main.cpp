#include "logging/logger.hpp"
#include "logging/sinks/mqtt_sink.hpp"
#include "logging/sinks/serial_sink.hpp"

#include "services/mqtt_service.hpp"
#include "services/telemetry_service.hpp"

#include "telemetry/sensors/imu_mpu_9250.hpp"

#include "drivers/esc/d_shot_600.hpp"

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
static const char *ESC_TOPIC = "Drone/manual/esc";
static const int ESC_PIN = 25;
static const int ESC_RATE = 2000;

static constexpr uint32_t IMU_RATE = 100;         // Hz
static constexpr size_t TELEMETRY_QUEUE_LEN = 64; // Telemetry queue depth
// ==============================================================================

// ===== Accessed Hardware ======================================================
static IMU_MPU9250 imu(/*i2cMutex*/ nullptr, /*rateHz*/ IMU_RATE,
                       /*topicSuffix*/ "telemetry/imu");

static DShot600Driver esc1;

// ==============================================================================

static volatile float g_targetNorm = 0.0f;

static String payloadToString(const uint8_t *data, size_t len)
{
  String s;
  s.reserve(len + 1); // +1 for null terminator
  for (size_t i = 0; i < len; ++i)
  {
    s += static_cast<char>(data[i]);
  }
  s.trim();
  return s;
}

static bool tryParseNorm01(const String &in, float &out)
{
  String s = in; // copy
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
      {
        break;
      }
    }
    String num = s.substring(first, last);
    float v = num.toFloat();
    if (!isnan(v))
    {
      if (v < 0.f)
      {
        v = 0.f;
      }
      if (v > 1.f)
      {
        v = 1.f;
      }
      out = v;
      return true;
    }
  }
  return false;
}

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

  // Subscribe to topics
  mqtt.subscribe(ESC_TOPIC);
  mqtt.onMessage([](const char *topic,
                    const uint8_t *data,
                    size_t len,
                    const AsyncMqttClientMessageProperties &props)
                 {
    (void)props;
    if (strcmp(topic, ESC_TOPIC) != 0) return;
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

  // ===== Setup Telemetry Service ==============================================
  auto &svc = TelemetryService::instance();
  svc.begin(DEVICE_ID, /*queueLen=*/TELEMETRY_QUEUE_LEN,
            /*txPrio=*/5, /*txStackWords=*/4096, /*txCore=*/tskNO_AFFINITY);

  // Hand shared I2C mutex to providers that use it
  imu.setI2CMutex(svc.i2cMutex());

  // Register IMU provider
  svc.addProvider(&imu);

  // ===== Esc Setup ==============================================================
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
  Serial.printf("ESC ready on GPIO %d @ %d Hz. MQTT topic: %s\n", ESC_PIN, ESC_RATE, ESC_TOPIC);
}

void loop()
{
  // Nothing required here unless you need to pump WiFi/MQTT client, etc.
  // delay to keep the watchdog happy if absolutely idle
  float target = g_targetNorm;
  esc1.writeNormalized(target);
  delayMicroseconds(500); // ≈ 2 kHz DShot command rate
}
