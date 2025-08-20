#pragma once

#include <Arduino.h>
#include <WiFi.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <functional>

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
    void begin(const char *wifiSsid, const char *wifiPassword,
               const IPAddress &host = IPAddress(), uint16_t port = 0);

    // Set MQTT server before begin() if you don't pass host/port there.
    void setServer(const IPAddress &host, uint16_t port);

    // High-level helpers
    void connectWifi();
    void connectMqtt();
    void disconnectMqtt();

    // MQTT ops (return true if a packet was queued)
    bool publish(const char *topic, const char *payload,
                 uint8_t qos = 0, bool retain = false);
    bool publish(const char *topic, const char *payload, size_t len,
                 uint8_t qos = 0, bool retain = false);
    struct Sub
    {
        const char *topic;
        uint8_t qos;
        MessageCallback cb;
    };
    bool subscribe(const char *topic, uint8_t qos);
    bool subscribe(const char *topic, uint8_t qos, MessageCallback cb);
    bool unsubscribe(const char *topic);

    void onMessage(MessageCallback cb);

    // Lightweight state
    bool wifiConnected() const { return WiFi.isConnected(); }
    bool mqttConnected() const { return mqttClient_.connected(); }
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
    void onMqttSubscribe(uint16_t packetId, uint8_t qos);
    void onMqttUnsubscribe(uint16_t packetId);
    void onMqttMessage(Message msg, size_t index, size_t total);
    void onMqttPublish(uint16_t packetId);

    bool topicMatches(const std::string &filter, const char *topic);

    void resubscribeAll();
    std::vector<Sub> _subs;

    // FreeRTOS timer callbacks
    static void mqttTimerCbStatic(TimerHandle_t);
    static void wifiTimerCbStatic(TimerHandle_t);

private:
    AsyncMqttClient mqttClient_;
    TimerHandle_t mqttReconnectTimer_{nullptr};
    TimerHandle_t wifiReconnectTimer_{nullptr};

    IPAddress mqttHost_{};
    uint16_t mqttPort_{0};
    const char *ssid_{nullptr};
    const char *pass_{nullptr};

    MessageCallback appMsgCb_{};
};
