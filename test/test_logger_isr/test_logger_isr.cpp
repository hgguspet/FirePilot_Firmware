#include <Arduino.h>
#include <unity.h>
#include "logging/logger.hpp"
#include "../_common/probe_sink.hpp"

void setUp() {}
void tearDown() {}

static ProbeSink isr_sink;

void test_isr_log_path()
{
    auto &L = Logger::instance();
    L.init(32);
    L.setLevel(LogLevel::Info);
    L.addSink(&isr_sink);

    uint32_t before = isr_sink.count;

    // Simulate ISR context by calling the ISR-safe API. We’re still in task context,
    // but the logger uses xQueueSendFromISR internally which is fine to call here.
    Logger::instance().logfIsr(LogLevel::Warn, "RMT", "ovf %d", 42);

    TEST_ASSERT_TRUE(wait_for_count(isr_sink.count, before + 1));
    TEST_ASSERT_EQUAL(LogLevel::Warn, isr_sink.last_level);
    TEST_ASSERT_EQUAL_STRING("RMT", isr_sink.last_tag);
    TEST_ASSERT_TRUE(strstr(isr_sink.last_msg, "ovf") != nullptr);
}

void setup()
{
    Serial.begin(115200);
    UNITY_BEGIN();
    RUN_TEST(test_isr_log_path);
    UNITY_END();
}
void loop() {}
