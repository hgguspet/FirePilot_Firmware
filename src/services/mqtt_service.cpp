#include "mqtt_service.hpp"
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
                          { this->onMqttMessage(Message{topic, reinterpret_cast<uint8_t *>(payload), len, props}, index, total); });
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
    // nullptr guard
    if (!wifiSsid || !wifiPassword)
    {
        LOGE("MqttService", "Wi-Fi credentials not provided");
        return;
    }

    ssid_ = wifiSsid;
    pass_ = wifiPassword;

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
    _subs.push_back(Sub{topic, qos, nullptr});
    if (!mqttClient_.connected())
    {
        LOGW("MqttService", "Queued sub '%s' not yet connected", topic);
        return true; // Queued
    }
    uint16_t id = mqttClient_.subscribe(topic, qos);
    return id != 0;
}

bool MqttService::subscribe(const char *topic, uint8_t qos, MessageCallback cb)
{
    _subs.push_back(Sub{topic, qos, std::move(cb)});
    if (!mqttClient_.connected())
    {
        LOGW("MqttService", "Queued sub '%s' with message callback, not yet connected", topic);
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
    auto it = std::find_if(
        _subs.begin(), _subs.end(),
        [topic](const Sub &s)
        { return s.topic == topic; });
    if (it != _subs.end())
    {
        std::string t = it->topic; // keep a copy for the broker call
        _subs.erase(it);
        if (mqttClient_.connected())
        {
            uint16_t id = mqttClient_.unsubscribe(t.c_str());
            return id != 0;
        }
        return false;
    }
    else
    {
        LOGW("MqttService", "Attempted to unsubscribe from non-subscribed topic: %s", topic);
        return false;
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
    LOGI("MqttService", "WiFi-event %d", event);
    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
        LOGI("MqttService", "WiFi Connected. IP: %s", WiFi.localIP().toString().c_str());
        connectMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        LOGW("MqttService", "WiFi Disconnected.");
        xTimerStop(mqttReconnectTimer_, 0); // don't race reconnects
        xTimerStart(wifiReconnectTimer_, 0);
        break;
    default:
        break;
    }
}

void MqttService::onMqttConnect(bool sessionPresent)
{
    LOGI("MqttService", "Connected. sessionPresent=%d", sessionPresent);
    resubscribeAll(); // Resubscribe to all topics
}

void MqttService::onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
    LOGI("MqttService", "Disconnected. reason=%d", static_cast<int>(reason));
    if (WiFi.isConnected())
    {
        xTimerStart(mqttReconnectTimer_, 0);
    }
}

void MqttService::onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
    LOGI("MqttService", "Subscribed. id=%u qos=%u", packetId, qos);
}

void MqttService::onMqttUnsubscribe(uint16_t packetId)
{
    LOGI("MqttService", "Unsubscribed. id=%u", packetId);
}

void MqttService::onMqttPublish(uint16_t packetId)
{
    LOGI("MqttService", "Publish ACK. id=%u", packetId);
}

static bool segEq(const char *a, size_t alen, const char *b, size_t blen)
{
    return alen == blen && memcmp(a, b, alen) == 0;
}

bool MqttService::topicMatches(const std::string &filter, const char *topic)
{
    // MQTT match per spec: filter tokens vs topic tokens
    const char *f = filter.c_str();
    const char *t = topic;

    while (*f && *t)
    {
        // read next token from filter
        const char *fstart = f;
        while (*f && *f != '/')
            ++f;
        size_t flen = size_t(f - fstart);

        // read next token from topic
        const char *tstart = t;
        while (*t && *t != '/')
            ++t;
        size_t tlen = size_t(t - tstart);

        if (flen == 1 && fstart[0] == '+')
        {
            // single-level wildcard: always matches this level
        }
        else if (flen == 1 && fstart[0] == '#')
        {
            // multi-level wildcard: must be last in filter
            return true;
        }
        else if (!segEq(fstart, flen, tstart, tlen))
        {
            return false;
        }

        if (*f == '/')
            ++f;
        if (*t == '/')
            ++t;
    }

    // If filter has trailing '#', it matches remaining topic
    if (*f == '#' && (f == filter.c_str() || *(f - 1) == '/') && *(f + 1) == '\0')
    {
        return true;
    }

    // Both must end at the same time (no leftover levels)
    return *f == '\0' && *t == '\0';
}

void MqttService::onMqttMessage(Message msg, size_t index, size_t total)
{
    // If messages can be chunked and you need the full payload,
    // you'd buffer by (topic,index,total) here. For now we forward chunks.

    bool handled = false;
    for (const auto &sub : _subs)
    {
        if (topicMatches(sub.topic, msg.topic))
        {
            if (sub.cb)
            {
                sub.cb(msg);
                handled = true;
                // You can break on first match, or deliver to all matching filters.
                // break;
            }
        }
    }

    if (!handled)
    {
        if (appMsgCb_)
        {
            appMsgCb_(msg);
        }
        else
        {
            LOGI("MqttService", "Message topic=%s len=%u qos=%u retain=%u idx=%u total=%u",
                 msg.topic, (unsigned)msg.len, msg.props.qos, msg.props.retain,
                 (unsigned)index, (unsigned)total);
        }
    }
}