#include <Arduino.h>
#include <unity.h>
#include "logging/logger.hpp"
#include "logging/ilog_sink.hpp"
#include "../_common/probe_sink.hpp"

void setUp() {}
void tearDown() {}

void test_init_and_single_sink_receives_info()
{
    auto &L = Logger::instance();
    L.init(32);
    L.setLevel(LogLevel::Info);

    static ProbeSink sink;
    L.addSink(&sink);

    uint32_t before = sink.count;
    LOGI("BOOT", "hello");
    TEST_ASSERT_TRUE(wait_for_count(sink.count, before + 1));

    TEST_ASSERT_EQUAL(LogLevel::Info, sink.last_level);
    TEST_ASSERT_EQUAL_STRING("BOOT", sink.last_tag);
    TEST_ASSERT_TRUE_MESSAGE(strlen(sink.last_msg) > 0, "message should not be empty");
    TEST_ASSERT_EQUAL_STRING("hello", sink.last_msg);
}

void test_runtime_level_filters_out_lower_levels()
{
    auto &L = Logger::instance();
    L.setLevel(LogLevel::Warn);

    static ProbeSink sink2;
    L.addSink(&sink2);

    uint32_t before = sink2.count;
    LOGD("DBG", "debug"); // should be filtered
    LOGI("INF", "info");  // should be filtered
    delay(50);
    TEST_ASSERT_EQUAL(before, sink2.count); // no new messages

    LOGW("WRN", "warn"); // should pass
    TEST_ASSERT_TRUE(wait_for_count(sink2.count, before + 1));
    TEST_ASSERT_EQUAL(LogLevel::Warn, sink2.last_level);
    TEST_ASSERT_EQUAL_STRING("WRN", sink2.last_tag);
    TEST_ASSERT_EQUAL_STRING("warn", sink2.last_msg);
}

void test_message_truncation()
{
    auto &L = Logger::instance();
    L.init(32);                 // safe if already inited
    L.setLevel(LogLevel::Info); // ensure INFO passes

    static ProbeSink sink; // has a 256B message buffer
    L.addSink(&sink);

    // Build a big input (way larger than any sane cap and larger than ProbeSink)
    static char big[4096];
    for (size_t i = 0; i < sizeof(big) - 1; ++i)
        big[i] = 'A' + (i % 26);
    big[sizeof(big) - 1] = '\0';
    const size_t in_len = strlen(big);

    uint32_t before = sink.count;
    LOGI("TRUNC", "%s", big); // use INFO so it isn't compiled out
    TEST_ASSERT_TRUE_MESSAGE(wait_for_count(sink.count, before + 1, 1500),
                             "logger did not deliver to sink");

    // Assert truncation occurred (output shorter than input),
    // NUL-termination is correct, and sink buffer didn't overflow.
    TEST_ASSERT_TRUE_MESSAGE(sink.last_len < in_len, "Expected truncation");
    TEST_ASSERT_EQUAL_UINT_MESSAGE(sink.last_len, strlen(sink.last_msg),
                                   "Output not NUL-terminated at reported length");
    TEST_ASSERT_TRUE_MESSAGE(sink.last_len <= (sizeof(sink.last_msg) - 1),
                             "Sink buffer overflowed");

    // Sanity: tag and level propagated.
    TEST_ASSERT_EQUAL(LogLevel::Info, sink.last_level);
    TEST_ASSERT_EQUAL_STRING("TRUNC", sink.last_tag);
}

void test_multiple_sinks_receive_same_record()
{
    auto &L = Logger::instance();
    L.setLevel(LogLevel::Info);

    static ProbeSink a;
    static ProbeSink b;
    L.addSink(&a);
    L.addSink(&b);

    uint32_t ca = a.count, cb = b.count;
    LOGI("MULTI", "fanout");
    TEST_ASSERT_TRUE(wait_for_count(a.count, ca + 1));
    TEST_ASSERT_TRUE(wait_for_count(b.count, cb + 1));

    TEST_ASSERT_EQUAL_STRING("MULTI", a.last_tag);
    TEST_ASSERT_EQUAL_STRING("fanout", a.last_msg);
    TEST_ASSERT_EQUAL_STRING("MULTI", b.last_tag);
    TEST_ASSERT_EQUAL_STRING("fanout", b.last_msg);
}

void setup()
{
    Serial.begin(115200);
    UNITY_BEGIN();
    RUN_TEST(test_init_and_single_sink_receives_info);
    RUN_TEST(test_runtime_level_filters_out_lower_levels);
    RUN_TEST(test_message_truncation);
    RUN_TEST(test_multiple_sinks_receive_same_record);
    UNITY_END();
}

void loop() {}
