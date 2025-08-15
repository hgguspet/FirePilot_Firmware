#include <Arduino.h>
#include <unity.h>
#include "logging/logger.hpp"
#include "../_common/probe_sink.hpp"

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

void test_queue_overflow_drops_messages()
{
    auto &L = Logger::instance();
    L.init(32); // no-op if already inited
    L.setLevel(LogLevel::Info);

    static ProbeSink sink;
    L.addSink(&sink);

    // Warmup to ensure delivery works
    uint32_t before = sink.count;
    Logger::instance().logf(LogLevel::Info, "OVER", "warmup");
    TEST_ASSERT_TRUE_MESSAGE(wait_for_count(sink.count, before + 1, 1500),
                             "Should receive warmup message");

    // Freeze the scheduler so the consumer task can't drain the queue
    vTaskSuspendAll();

    before = sink.count;
    const int burst = 20000; // >> any realistic queue capacity
    for (int i = 0; i < burst; ++i)
    {
        Logger::instance().logf(LogLevel::Info, "BURST", "m%d", i);
        // non-blocking enqueue: drops when full while scheduler is suspended
    }

    xTaskResumeAll();

    // Now let the consumer drain whatever made it into the queue
    delay(300);

    uint32_t received = sink.count - before;
    TEST_ASSERT_TRUE_MESSAGE(received > 0, "Should receive some");
    TEST_ASSERT_TRUE_MESSAGE(received < (uint32_t)burst, "Expected drops; received all?");
}
// ---- Arduino/Unity runner ----
void setUp() {}
void tearDown() {}

void setup()
{
    Serial.begin(115200);
    // Optional settle time
    delay(200);

    UNITY_BEGIN();
    RUN_TEST(test_queue_overflow_drops_messages);
    UNITY_END();
}

void loop() {}
