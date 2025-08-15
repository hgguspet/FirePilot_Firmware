#include "mqtt_service.hpp"
#include "secrets.hpp"
#include "logging/logger.hpp"

// ===== Singleton =====
MqttService &MqttService::instance()
{
    static MqttService s;
    return s;
}

MqttService::MqttService()
{
    // Create one-shot timers (2s delay) for reconnects
    mqttReconnectTimer_ = xTimerCreate("mqttTimer",
                                       pdMS_TO_TICKS(2000),
                                       pdFALSE,
                                       this,
                                       &MqttService::mqttTimerCbStatic);
    wifiReconnectTimer_ = xTimerCreate("wifiTimer",
                                       pdMS_TO_TICKS(2000),
                                       pdFALSE,
                                       this,
                                       &MqttService::wifiTimerCbStatic);

    // Hook WiFi events
    WiFi.onEvent(&MqttService::wifiEventStatic);

    // Hook MQTT events
    mqttClient_.onConnect([this](bool sp)
                          { this->onMqttConnect(sp); });
    mqttClient_.onDisconnect([this](AsyncMqttClientDisconnectReason reason)
                             { this->onMqttDisconnect(reason); });
    mqttClient_.onSubscribe([this](uint16_t id, uint8_t qos)
                            { this->onMqttSubscribe(id, qos); });
    mqttClient_.onUnsubscribe([this](uint16_t id)
                              { this->onMqttUnsubscribe(id); });
    mqttClient_.onMessage([this](char *topic, char *payload,
                                 AsyncMqttClientMessageProperties props,
                                 size_t len, size_t index, size_t total)
                          { this->onMqttMessage(topic, payload, props, len, index, total); });
    mqttClient_.onPublish([this](uint16_t id)
                          { this->onMqttPublish(id); });
}

// ===== Public API =====
void MqttService::setServer(const IPAddress &host, uint16_t port)
{
    mqttHost_ = host;
    mqttPort_ = port;
    mqttClient_.setServer(mqttHost_, mqttPort_);
}

void MqttService::begin(const char *wifiSsid, const char *wifiPassword,
                        const IPAddress &host, uint16_t port)
{
    ssid_ = wifiSsid ? wifiSsid : secrets::wifi_ssid;
    pass_ = wifiPassword ? wifiPassword : secrets::wifi_password;

    if (host != IPAddress() && port != 0)
    {
        setServer(host, port);
    }

    LOGI("MqttService", "Starting Wi-Fi…");
    connectWifi();
}

void MqttService::connectWifi()
{
    if (WiFi.isConnected())
        return;
    LOGI("MqttService", "Connecting to Wi-Fi SSID '%s'…", ssid_ ? ssid_ : "(null)");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid_, pass_);
}

void MqttService::connectMqtt()
{
    if (mqttHost_ == IPAddress() || mqttPort_ == 0)
    {
        LOGE("MqttService", "MQTT server not set. Call setServer() or pass host/port to begin().");
        return;
    }
    if (mqttClient_.connected())
        return;
    LOGI("MqttService", "Connecting to MQTT %s:%u…", mqttHost_.toString().c_str(), mqttPort_);
    mqttClient_.connect();
}

void MqttService::disconnectMqtt()
{
    if (mqttClient_.connected())
    {
        LOGI("MqttService", "Disconnecting MQTT…");
        mqttClient_.disconnect();
    }
}

bool MqttService::publish(const char *topic, const char *payload,
                          uint8_t qos, bool retain)
{
    if (!mqttClient_.connected())
        return false;
    uint16_t id = mqttClient_.publish(topic, qos, retain, payload);
    return id != 0;
}

bool MqttService::publish(const char *topic, const char *payload, size_t len,
                          uint8_t qos, bool retain)
{
    if (!mqttClient_.connected())
        return false;
    uint16_t id = mqttClient_.publish(topic, qos, retain, payload, len);
    return id != 0;
}

bool MqttService::subscribe(const char *topic, uint8_t qos)
{
    // Keep a persistent copy for future resubscribe
    _subs.push_back(Sub{topic, qos});
    if (!mqttClient_.connected())
    {
        LOGW("MqttService", "Queued sub '%s' not yet connected", topic);
        return true; // Queued
    }
    uint16_t id = mqttClient_.subscribe(topic, qos);
    return id != 0;
}

void MqttService::resubscribeAll()
{
    for (const auto &sub : _subs)
    {
        uint16_t id = mqttClient_.subscribe(sub.topic, sub.qos);
        LOGI("MqttService", "(Re)subscribe '%s' qos=%u -> id=%u",
             sub.topic, sub.qos, id);
    }
}

bool MqttService::unsubscribe(const char *topic)
{
    // Check persistent subscriptions
    auto it = std::find_if(_subs.begin(), _subs.end(),
                           [topic](const Sub &sub)
                           { return strcmp(sub.topic, topic) == 0; });
    if (it != _subs.end())
    {
        _subs.erase(it);
        if (mqttClient_.connected())
        {
            uint16_t id = mqttClient_.unsubscribe(topic);
            return id != 0;
        }
        return false; // Not connected
    }
    else
    {
        LOGW("MqttService", "Attempted to unsubscribe from topic that was not subscribed to: %s", topic);
        return false; // Not subscribed (warning / error)
    }
}

void MqttService::onMessage(MessageCallback cb)
{
    appMsgCb_ = std::move(cb);
}

// ===== Static shims =====
void MqttService::wifiEventStatic(WiFiEvent_t event)
{
    instance().handleWifiEvent(event);
}

void MqttService::mqttTimerCbStatic(TimerHandle_t t)
{
    auto *self = static_cast<MqttService *>(pvTimerGetTimerID(t));
    if (self)
        self->connectMqtt();
}

void MqttService::wifiTimerCbStatic(TimerHandle_t t)
{
    auto *self = static_cast<MqttService *>(pvTimerGetTimerID(t));
    if (self)
        self->connectWifi();
}

// ===== Event handlers =====
void MqttService::handleWifiEvent(WiFiEvent_t event)
{
    LOGI("MqttService", "[WiFi-event] %d", event);
    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
        LOGI("MqttService", "[WiFi] Connected. IP: %s", WiFi.localIP().toString().c_str());
        connectMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        LOGW("MqttService", "[WiFi] Disconnected.");
        xTimerStop(mqttReconnectTimer_, 0); // don't race reconnects
        xTimerStart(wifiReconnectTimer_, 0);
        break;
    default:
        break;
    }
}

void MqttService::onMqttConnect(bool sessionPresent)
{
    LOGI("MqttService", "[MQTT] Connected. sessionPresent=%d", sessionPresent);
    resubscribeAll(); // Resubscribe to all topics
}

void MqttService::onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
    LOGI("MqttService", "[MQTT] Disconnected. reason=%d", static_cast<int>(reason));
    if (WiFi.isConnected())
    {
        xTimerStart(mqttReconnectTimer_, 0);
    }
}

void MqttService::onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
    LOGI("MqttService", "[MQTT] Subscribed. id=%u qos=%u", packetId, qos);
}

void MqttService::onMqttUnsubscribe(uint16_t packetId)
{
    LOGI("MqttService", "[MQTT] Unsubscribed. id=%u", packetId);
}

void MqttService::onMqttPublish(uint16_t packetId)
{
    LOGI("MqttService", "[MQTT] Publish ACK. id=%u", packetId);
}

void MqttService::onMqttMessage(char *topic, char *payload,
                                AsyncMqttClientMessageProperties properties,
                                size_t len, size_t index, size_t total)
{
    // payload may not be null-terminated; forward raw bytes
    if (index == 0)
    {
        // first chunk
    }
    if (appMsgCb_)
    {
        appMsgCb_(topic, reinterpret_cast<uint8_t *>(payload), len, properties);
    }
    else
    {
        LOGI("MqttService", "[MQTT] Message topic=%s len=%u qos=%u retain=%u idx=%u total=%u",
             topic, (unsigned)len, properties.qos, properties.retain,
             (unsigned)index, (unsigned)total);
    }
}
