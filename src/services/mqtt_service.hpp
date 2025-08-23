#pragma once

#include "logging/logger.hpp"

#include <Arduino.h>
#include <WiFi.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <functional>

namespace MqttService
{
    /**
     * @note when you have a topic of this type, it's always expected to exclude `<deviceId>`
     */
    typedef const char *Topic;

    /**
     * @note Represents a device ID that will automatically be prepended to MQTT topics.
     */
    typedef const char *Device;

    /**
     * @note Represents a port number.
     */
    typedef uint16_t Port;

    enum class QoS
    {
        AtMostOnce = 0,
        AtLeastOnce = 1,
        ExactlyOnce = 2
    };
    struct Message
    {
        const char *topic;
        const uint8_t *payload;
        size_t len;
        AsyncMqttClientMessageProperties props;
    };

    class MqttService
    {
    public:
        using MessageCallback = std::function<void(const Message &msg)>;

        static MqttService &instance();

        // Initialize and start Wi-Fi + MQTT.
        // If you omit host/port, call setServer() before begin().
        void begin(
            const char *wifiSsid, const char *wifiPassword, const char *deviceId,
            const IPAddress &host = IPAddress(), Port port = 0);

        // Set MQTT server before begin() if you don't pass host/port there.
        void setServer(const IPAddress &host, Port port);

        // High-level helpers
        void connectWifi();
        void connectMqtt();
        void disconnectMqtt();

        // MQTT ops (return true if a packet was queued)
        bool publish(
            Topic topic, const char *payload,
            QoS qos = QoS::AtMostOnce, bool retain = false);
        bool publish(
            Topic topic, const char *payload, size_t len,
            QoS qos = QoS::AtMostOnce, bool retain = false);

        bool publishRel(
            Topic topic, const char *payload,
            QoS qos = QoS::AtMostOnce, bool retain = false)
        {
            String topicStr = String(topic) + String(getIfValidDeviceId());
            return publish(
                topicStr.c_str(), payload, qos, retain);
        }
        bool publishRel(
            Topic topic, const char *payload, size_t len,
            QoS qos = QoS::AtMostOnce, bool retain = false)
        {
            String topicStr = String(getIfValidDeviceId()) + "/" + String(topic);
            return publish(
                topicStr.c_str(), payload, len, qos, retain);
        }

        struct Sub
        {
            String topic;
            QoS qos;
            MessageCallback cb;
        };
        bool subscribe(Topic topic, QoS qos);
        bool subscribe(Topic topic, QoS qos, MessageCallback cb);

        bool subscribeRel(Topic topic, QoS qos)
        {
            String topicStr = String(getIfValidDeviceId()) + "/" + String(topic);
            return subscribe(topicStr.c_str(), qos);
        }

        bool subscribeRel(Topic topic, QoS qos, MessageCallback cb)
        {
            String topicStr = String(getIfValidDeviceId()) + "/" + String(topic);
            LOGI("MQTT", "Constructed topic: %s", topicStr.c_str());
            return subscribe(topicStr.c_str(), qos, std::move(cb));
        }

        bool unsubscribe(Topic topic);

        void onMessage(MessageCallback cb);

        // Lightweight state
        bool wifiConnected() const { return WiFi.isConnected(); }
        bool mqttConnected() const { return _mqttClient.connected(); }
        IPAddress localIp() const { return WiFi.localIP(); }

    private:
        MqttService();
        ~MqttService() = default;
        MqttService(const MqttService &) = delete;
        MqttService &operator=(const MqttService &) = delete;

        // Event handlers
        static void wifiEventStatic(WiFiEvent_t event);
        void handleWifiEvent(WiFiEvent_t event);

        void onMqttConnect(bool sessionPresent);
        void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
        void onMqttSubscribe(uint16_t packetId, QoS qos);
        void onMqttUnsubscribe(uint16_t packetId);
        void onMqttMessage(Message msg, size_t index, size_t total);
        void onMqttPublish(uint16_t packetId);

        bool topicMatches(const std::string &filter, Topic topic);

        void resubscribeAll();
        std::vector<Sub> _subs;

        // FreeRTOS timer callbacks
        static void mqttTimerCbStatic(TimerHandle_t);
        static void wifiTimerCbStatic(TimerHandle_t);

    private:
        AsyncMqttClient _mqttClient;
        TimerHandle_t _mqttReconnectTimer{nullptr};
        TimerHandle_t _wifiReconnectTimer{nullptr};

        IPAddress _mqttHost{};
        Port _mqttPort{0};
        const char *_device_id{nullptr};
        const char *_ssid{nullptr};
        const char *_pass{nullptr};

        MessageCallback _appMsgCb{};

        const char *getIfValidDeviceId() const
        {
            if (!_device_id)
            {
                LOGE("MQTT", "Attempting to access uninitialized device ID");
                return nullptr;
            }
            return _device_id;
        }
    };
} // Namespace MqttService