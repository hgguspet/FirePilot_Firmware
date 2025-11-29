#pragma once
#include "logging/ilog_sink.hpp"
#include <stdint.h>
#include <string.h>
extern "C"
{
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "esp_timer.h" // esp_timer_get_time()
}

// ===== Tunables ===============================================================
#ifndef PING_SINK_MAX
#define PING_SINK_MAX 6
#endif
#ifndef PING_TASK_STACK
#define PING_TASK_STACK 4096 // bytes
#endif
#ifndef PING_TASK_PRIO
#define PING_TASK_PRIO 2
#endif
#ifndef PING_TASK_CORE
#define PING_TASK_CORE tskNO_AFFINITY
#endif
#ifndef PING_QUEUE_CAPACITY
#define PING_QUEUE_CAPACITY 16
#endif
#ifndef PING_MSG
#define PING_MSG "1"
#endif

// ==============================================================================
class Pinger
{
public:
    // Singleton access
    static Pinger &instance(unsigned int interval_ms = 1000, const char *topic_rel = "ping")
    {
        static Pinger instance(interval_ms, topic_rel);
        return instance;
    }

    // Delete copy constructor and assignment operator
    Pinger(const Pinger &) = delete;
    Pinger &operator=(const Pinger &) = delete;

private:
    explicit Pinger(unsigned int interval_ms, const char *topic_rel = "ping")
        : _interval_ms(interval_ms), _topic_rel(topic_rel)
    {
        _sinks_mux = portMUX_INITIALIZER_UNLOCKED;
    }
    ~Pinger() { end(); }

public:
    // Register a sink (thread-safe). No duplicates check for speed.
    void addSink(ILogSink *sink)
    {
        if (!sink)
            return;
        portENTER_CRITICAL(&_sinks_mux);
        if (_sink_count < PING_SINK_MAX)
        {
            _sinks[_sink_count++] = sink;
        }
        portEXIT_CRITICAL(&_sinks_mux);
    }

    // Start / restart the pinger.
    void begin(uint16_t queue_capacity = PING_QUEUE_CAPACITY);

    // Stop the pinger and free resources.
    void end();

private:
    // ===== Instance-local queue item (POD) ====================================
    struct QueueItem
    {
        uint32_t ts_us;
        const char *tag; // expected literal / long-lived string
        uint16_t len;
        char msg[8]; // "1" fits; keep small & POD
    };

    // ===== Members =============================================================
    QueueHandle_t _q = nullptr;
    TaskHandle_t _producer_task = nullptr; // Task that generates pings
    TaskHandle_t _consumer_task = nullptr; // Task that processes queue
    ILogSink *_sinks[PING_SINK_MAX] = {};
    uint8_t _sink_count = 0;
    portMUX_TYPE _sinks_mux;
    unsigned int _interval_ms;
    const char *_topic_rel;             // relative tag (default: "ping")
    volatile bool _should_stop = false; // Signal to stop tasks

    // ===== Helpers =============================================================
    static void producerTaskTrampoline(void *arg);
    void producerTask(); // sleeps and generates ping messages

    static void consumerTaskTrampoline(void *arg);
    void consumerTask(); // drains queue and calls sinks
};