#include "mqtt_service.hpp"
#include "logging/logger.hpp"

namespace MqttService
{

    // ===== Singleton =====
    MqttService &MqttService::instance()
    {
        static MqttService s;
        return s;
    }

    MqttService::MqttService()
    {
        // Create one-shot timers (2s delay) for reconnects
        _mqttReconnectTimer = xTimerCreate(
            "mqttTimer",
            pdMS_TO_TICKS(2000),
            pdFALSE,
            this,
            &MqttService::mqttTimerCbStatic);
        _wifiReconnectTimer = xTimerCreate(
            "wifiTimer",
            pdMS_TO_TICKS(2000),
            pdFALSE,
            this,
            &MqttService::wifiTimerCbStatic);

        // Hook WiFi events
        WiFi.onEvent(&MqttService::wifiEventStatic);

        // Hook MQTT events
        _mqttClient.onConnect(
            [this](bool sp)
            { this->onMqttConnect(sp); });
        _mqttClient.onDisconnect(
            [this](AsyncMqttClientDisconnectReason reason)
            { this->onMqttDisconnect(reason); });
        _mqttClient.onSubscribe(
            [this](uint16_t id, uint8_t qos)
            { this->onMqttSubscribe(id, (QoS)qos); });
        _mqttClient.onUnsubscribe(
            [this](uint16_t id)
            { this->onMqttUnsubscribe(id); });
        _mqttClient.onMessage(
            [this](
                char *topic, char *payload,
                AsyncMqttClientMessageProperties props,
                size_t len, size_t index, size_t total)
            { this->onMqttMessage(Message{topic, reinterpret_cast<uint8_t *>(payload), len, props}, index, total); });
        _mqttClient.onPublish(
            [this](uint16_t id)
            { this->onMqttPublish(id); });
    }

    // ===== Public API =====
    void MqttService::setServer(const IPAddress &host, uint16_t port)
    {
        _mqttHost = host;
        _mqttPort = port;
        _mqttClient.setServer(_mqttHost, _mqttPort);
    }

    void MqttService::begin(const char *wifiSsid, const char *wifiPassword, const char *deviceId,
                            const IPAddress &host, Port port)
    {
        // nullptr guard
        if (!wifiSsid || !wifiPassword || !deviceId)
        {
            LOGE("MqttService", "Wi-Fi credentials or device ID not provided");
            return;
        }

        _ssid = wifiSsid;
        _pass = wifiPassword;
        _device_id = deviceId;

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
        LOGI("MqttService", "Connecting to Wi-Fi SSID '%s'…", _ssid ? _ssid : "(null)");
        WiFi.mode(WIFI_STA);
        WiFi.begin(_ssid, _pass);
    }

    void MqttService::connectMqtt()
    {
        if (_mqttHost == IPAddress() || _mqttPort == 0)
        {
            LOGE("MqttService", "MQTT server not set. Call setServer() or pass host/port to begin().");
            return;
        }
        if (_mqttClient.connected())
            return;
        LOGI("MqttService", "Connecting to MQTT %s:%u…", _mqttHost.toString().c_str(), _mqttPort);
        _mqttClient.connect();
    }

    void MqttService::disconnectMqtt()
    {
        if (_mqttClient.connected())
        {
            LOGI("MqttService", "Disconnecting MQTT…");
            _mqttClient.disconnect();
        }
    }

    bool MqttService::publish(
        Topic topic, const char *payload,
        QoS qos, bool retain)
    {
        if (!_mqttClient.connected())
            return false;
        uint16_t id = _mqttClient.publish(topic, (uint8_t)qos, retain, payload);
        return id != 0;
    }

    bool MqttService::publish(
        Topic topic, const char *payload, size_t len,
        QoS qos, bool retain)
    {
        if (!_mqttClient.connected())
            return false;
        uint16_t id = _mqttClient.publish(topic, (uint8_t)qos, retain, payload, len);
        return id != 0;
    }

    bool MqttService::subscribe(Topic topic, QoS qos)
    {
        // Keep a persistent copy for future resubscribe
        _subs.push_back(Sub{String(topic), qos, nullptr});
        if (!_mqttClient.connected())
        {
            LOGW("MqttService", "Queued sub '%s' not yet connected", topic);
            return true; // Queued
        }
        uint16_t id = _mqttClient.subscribe(topic, (uint8_t)qos);
        return id != 0;
    }

    bool MqttService::subscribe(Topic topic, QoS qos, MessageCallback cb)
    {
        _subs.push_back(Sub{String(topic), qos, std::move(cb)});
        if (!_mqttClient.connected())
        {
            LOGW("MqttService", "Queued sub '%s' with message callback, not yet connected", topic);
            return true; // Queued
        }
        uint16_t id = _mqttClient.subscribe(topic, (uint8_t)qos);
        return id != 0;
    }

    void MqttService::resubscribeAll()
    {
        for (const auto &sub : _subs)
        {
            uint16_t id = _mqttClient.subscribe(sub.topic.c_str(), (uint8_t)sub.qos);
            LOGI("MqttService", "(Re)subscribe '%s' qos=%u -> id=%u",
                 sub.topic.c_str(), sub.qos, id);
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
            String t = it->topic; // keep a copy for the broker call
            _subs.erase(it);
            if (_mqttClient.connected())
            {
                uint16_t id = _mqttClient.unsubscribe(t.c_str());
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
        _appMsgCb = std::move(cb);
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
            xTimerStop(_mqttReconnectTimer, 0); // don't race reconnects
            xTimerStart(_wifiReconnectTimer, 0);
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
            xTimerStart(_mqttReconnectTimer, 0);
        }
    }

    void MqttService::onMqttSubscribe(uint16_t packetId, QoS qos)
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
            if (topicMatches(sub.topic.c_str(), msg.topic))
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
            if (_appMsgCb)
            {
                _appMsgCb(msg);
            }
            else
            {
                LOGI(
                    "MqttService", "Message topic=%s len=%u qos=%u retain=%u idx=%u total=%u",
                    msg.topic, (unsigned)msg.len, msg.props.qos, msg.props.retain,
                    (unsigned)index, (unsigned)total);
            }
        }
    }
} // Namespace MqttService