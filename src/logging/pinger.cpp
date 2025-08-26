#include "logging/pinger.hpp"
#include "logging/logger.hpp" // for LOGE/LOGW on fail

// ===== begin / end ============================================================
void Pinger::begin(uint16_t queue_capacity)
{
    _should_stop = false;

    // Queue
    if (!_q)
    {
        _q = xQueueCreate(queue_capacity, sizeof(QueueItem));
        if (!_q)
        {
            LOGE("Pinger", "Failed to create queue");
            return;
        }
    }

    // Consumer task (processes the queue)
    if (!_consumer_task)
    {
#if (PING_TASK_CORE == tskNO_AFFINITY)
        if (xTaskCreate(&Pinger::consumerTaskTrampoline, "ping_consumer", PING_TASK_STACK, this, PING_TASK_PRIO, &_consumer_task) != pdPASS)
        {
            LOGE("Pinger", "Failed to create consumer task");
            vQueueDelete(_q);
            _q = nullptr;
            return;
        }
#else
        if (xTaskCreatePinnedToCore(&Pinger::consumerTaskTrampoline, "ping_consumer", PING_TASK_STACK, this, PING_TASK_PRIO, &_consumer_task, PING_TASK_CORE) != pdPASS)
        {
            LOGE("Pinger", "Failed to create consumer task");
            vQueueDelete(_q);
            _q = nullptr;
            return;
        }
#endif
    }

    // Producer task (generates pings at intervals)
    if (!_producer_task)
    {
#if (PING_TASK_CORE == tskNO_AFFINITY)
        if (xTaskCreate(&Pinger::producerTaskTrampoline, "ping_producer", PING_TASK_STACK, this, PING_TASK_PRIO, &_producer_task) != pdPASS)
        {
            LOGE("Pinger", "Failed to create producer task");
            vTaskDelete(_consumer_task);
            _consumer_task = nullptr;
            vQueueDelete(_q);
            _q = nullptr;
            return;
        }
#else
        if (xTaskCreatePinnedToCore(&Pinger::producerTaskTrampoline, "ping_producer", PING_TASK_STACK, this, PING_TASK_PRIO, &_producer_task, PING_TASK_CORE) != pdPASS)
        {
            LOGE("Pinger", "Failed to create producer task");
            vTaskDelete(_consumer_task);
            _consumer_task = nullptr;
            vQueueDelete(_q);
            _q = nullptr;
            return;
        }
#endif
    }

    LOGI("Pinger", "Started with interval %d ms (%d Hz)", _interval_ms, 1000 / _interval_ms);
}

void Pinger::end()
{
    // Signal tasks to stop
    _should_stop = true;

    // Wait a bit for tasks to see the stop signal
    vTaskDelay(pdMS_TO_TICKS(100));

    // Clean up tasks
    if (_producer_task)
    {
        vTaskDelete(_producer_task);
        _producer_task = nullptr;
    }
    if (_consumer_task)
    {
        vTaskDelete(_consumer_task);
        _consumer_task = nullptr;
    }

    // Clean up queue
    if (_q)
    {
        vQueueDelete(_q);
        _q = nullptr;
    }

    LOGI("Pinger", "Stopped");
}

// ===== Producer task -> generates pings at intervals =========================
void Pinger::producerTaskTrampoline(void *arg)
{
    static_cast<Pinger *>(arg)->producerTask();
}

void Pinger::producerTask()
{
    LOGI("Pinger", "Producer task started");

    TickType_t interval_ticks = pdMS_TO_TICKS(_interval_ms);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (!_should_stop)
    {
        // Sleep for the specified interval
        vTaskDelayUntil(&last_wake_time, interval_ticks);

        // Check if we should stop
        if (_should_stop)
            break;

        // Generate ping message
        if (!_q)
        {
            LOGW("Pinger", "Queue not available");
            continue;
        }

        QueueItem qi{};
        qi.ts_us = static_cast<uint32_t>(esp_timer_get_time());
        qi.tag = _topic_rel ? _topic_rel : "ping";

        // Put the tiny payload in-place (no formatting needed)
        constexpr char kPayload[] = PING_MSG; // default "1"
        constexpr size_t kLen = sizeof(kPayload) - 1;
        static_assert(kLen < sizeof(qi.msg), "PING payload too large for QueueItem::msg");
        memcpy(qi.msg, kPayload, kLen);
        qi.len = static_cast<uint16_t>(kLen);

        // Send to queue (don't block if queue is full)
        if (xQueueSend(_q, &qi, 0) != pdPASS)
        {
            LOGW("Pinger", "Queue full, dropping ping");
        }
    }

    LOGI("Pinger", "Producer task ending");
}

// ===== Consumer task -> processes queue and calls sinks ======================
void Pinger::consumerTaskTrampoline(void *arg)
{
    static_cast<Pinger *>(arg)->consumerTask();
}

void Pinger::consumerTask()
{
    LOGI("Pinger", "Consumer task started");

    QueueItem qi{};

    while (!_should_stop)
    {
        // Wait for queue items (with timeout to check stop flag)
        if (xQueueReceive(_q, &qi, pdMS_TO_TICKS(500)) != pdPASS)
        {
            // Timeout - check if we should stop and continue
            continue;
        }

        // Snapshot sinks to minimize time under the lock
        ILogSink *local[PING_SINK_MAX];
        uint8_t cnt;
        portENTER_CRITICAL(&_sinks_mux);
        cnt = _sink_count;
        for (uint8_t i = 0; i < cnt; ++i)
            local[i] = _sinks[i];
        portEXIT_CRITICAL(&_sinks_mux);

        // Skip if no sinks
        if (cnt == 0)
        {
            continue;
        }

        // Build a LogRecord compatible with sink interface
        LogRecord r{};
        r.ts_us = qi.ts_us;
        r.level = LogLevel::None; // uses it's own topic, thus this can be omitted
        r.tag = qi.tag;
        r.from_isr = false;
        r.msg = (qi.len ? qi.msg : nullptr);
        r.msg_len = qi.len;
        r.fmt = nullptr;
        r.va = nullptr;
        r.channel = "ping";

        // Send to all sinks
        for (uint8_t i = 0; i < cnt; ++i)
        {
            local[i]->write(r);
        }
    }

    LOGI("Pinger", "Consumer task ending");
}