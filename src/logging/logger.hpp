#pragma once

/**
 * @file logger.hpp
 * @brief Centralized logging system with support for multiple sinks and ISR-safe operation.
 *
 * This file contains the Logger singleton class that provides a thread-safe,
 * queue-based logging system for the FirePilot firmware. The logger supports
 * multiple output sinks and can be safely called from interrupt service routines.
 */

#include "ilog_sink.hpp"
#include <stdint.h>
#include <stdarg.h>

/**
 * @class Logger
 * @brief Singleton logger class providing centralized logging functionality.
 *
 * The Logger class implements a singleton pattern to provide system-wide logging
 * capabilities. It features:
 * - Thread-safe operation with internal queuing
 * - ISR-safe logging methods
 * - Support for multiple output sinks (Serial, MQTT, etc.)
 * - Configurable log levels for filtering
 * - Deferred formatting to minimize blocking time
 *
 * @note This class is designed to be lightweight and non-blocking to avoid
 *       interfering with real-time operations.
 */
class Logger
{
public:
    /**
     * @brief Get the singleton instance of the Logger.
     *
     * @return Reference to the Logger singleton instance.
     */
    static Logger &instance();

    /**
     * @brief Initialize the logging system.
     *
     * Sets up the internal ring buffer and starts the consumer task that
     * processes log messages asynchronously.
     *
     * @param queue_capacity Size of the internal message queue (default: 256)
     *
     * @note This method should be called once during system initialization
     *       before any logging operations.
     */
    void init(uint16_t queue_capacity = 256);

    /**
     * @brief Set the minimum log level for output.
     *
     * Messages below this level will be filtered out and not sent to sinks.
     *
     * @param level Minimum log level to output
     */
    void setLevel(LogLevel level);

    /**
     * @brief Get the current minimum log level.
     *
     * @return Current minimum log level setting
     */
    LogLevel level() const;

    /**
     * @brief Add a log output sink.
     *
     * Registers a new sink to receive log messages. The Logger takes ownership
     * of the sink and will delete it when the logger is destroyed.
     *
     * @param sink Pointer to the sink implementation (ownership transferred)
     *
     * @warning The sink pointer must be valid and the Logger takes ownership.
     *          Do not delete the sink manually after adding it.
     */
    void addSink(ILogSink *sink);

    /**
     * @brief Consumer task main loop.
     *
     * Processes queued log messages and dispatches them to registered sinks.
     * This method runs in a separate task to avoid blocking the producers.
     */
    void consumeTask();

    /**
     * @brief Log a formatted message (non-ISR safe).
     *
     * Primary logging entry point for normal (non-interrupt) contexts.
     * Supports printf-style formatting with variable arguments.
     *
     * @param level Log level for this message
     * @param tag Tag string to identify the message source
     * @param fmt Printf-style format string
     * @param ... Variable arguments for formatting
     *
     * @note This method may allocate memory and should not be called from ISRs.
     *       Use logfIsr() instead for interrupt-safe logging.
     */
    void logf(LogLevel level, const char *tag, const char *fmt, ...);

    /**
     * @brief Log a formatted message (ISR-safe).
     *
     * ISR-safe logging method that avoids memory allocation and blocking
     * operations. May defer formatting to the consumer task for complex
     * format strings.
     *
     * @param level Log level for this message
     * @param tag Tag string to identify the message source
     * @param fmt Printf-style format string
     * @param ... Variable arguments for formatting
     *
     * @note This method is designed to be called from interrupt service
     *       routines and is guaranteed not to block or allocate memory.
     */
    void logfIsr(LogLevel level, const char *tag, const char *fmt, ...);

    /**
     * @brief Internal logging implementation with va_list support.
     *
     * Common implementation for both logf() and logfIsr() that handles
     * variable argument lists directly. This method is primarily intended
     * for internal use and macro implementations.
     *
     * @param level Log level for this message
     * @param tag Tag string to identify the message source
     * @param fmt Printf-style format string
     * @param args Variable argument list
     * @param from_isr True if called from an ISR context
     */
    void vlogf(LogLevel level, const char *tag, const char *fmt, va_list args, bool from_isr);

private:
    /**
     * @brief Private constructor for singleton pattern.
     */
    Logger() = default;

    /**
     * @brief Add a log record to the internal queue.
     *
     * @param r Log record to enqueue (moved)
     */
    void enqueue(LogRecord &&r);
};

/**
 * @def LOG_COMPILE_LEVEL
 * @brief Compile-time log level filtering.
 *
 * Messages below this level are completely removed at compile time.
 * Levels: 0=Debug, 1=Info, 2=Warning, 3=Error, 4=Critical, 5=None
 */
#ifndef LOG_COMPILE_LEVEL
#define LOG_COMPILE_LEVEL 1
#endif

/**
 * @defgroup LoggingMacros Logging Convenience Macros
 * @brief Convenience macros for different log levels with compile-time filtering.
 *
 * These macros provide a convenient interface to the Logger singleton with
 * compile-time filtering support. Messages below LOG_COMPILE_LEVEL are
 * completely removed from the compiled code.
 *
 * @{
 */

/**
 * @brief Log a debug message.
 * @param TAG Source tag for the message
 * @param FMT Printf-style format string
 * @param ... Variable arguments for formatting
 */
#define LOGD(TAG, FMT, ...)                                                    \
    do                                                                         \
    {                                                                          \
        if (LOG_COMPILE_LEVEL <= 0)                                            \
            Logger::instance().logf(LogLevel::Debug, TAG, FMT, ##__VA_ARGS__); \
    } while (0)

/**
 * @brief Log an info message.
 * @param TAG Source tag for the message
 * @param FMT Printf-style format string
 * @param ... Variable arguments for formatting
 */
#define LOGI(TAG, FMT, ...)                                                   \
    do                                                                        \
    {                                                                         \
        if (LOG_COMPILE_LEVEL <= 1)                                           \
            Logger::instance().logf(LogLevel::Info, TAG, FMT, ##__VA_ARGS__); \
    } while (0)

/**
 * @brief Log a warning message.
 * @param TAG Source tag for the message
 * @param FMT Printf-style format string
 * @param ... Variable arguments for formatting
 */
#define LOGW(TAG, FMT, ...)                                                   \
    do                                                                        \
    {                                                                         \
        if (LOG_COMPILE_LEVEL <= 2)                                           \
            Logger::instance().logf(LogLevel::Warn, TAG, FMT, ##__VA_ARGS__); \
    } while (0)

/**
 * @brief Log an error message.
 * @param TAG Source tag for the message
 * @param FMT Printf-style format string
 * @param ... Variable arguments for formatting
 */
#define LOGE(TAG, FMT, ...)                                                    \
    do                                                                         \
    {                                                                          \
        if (LOG_COMPILE_LEVEL <= 3)                                            \
            Logger::instance().logf(LogLevel::Error, TAG, FMT, ##__VA_ARGS__); \
    } while (0)

/**
 * @brief Log a critical message.
 * @param TAG Source tag for the message
 * @param FMT Printf-style format string
 * @param ... Variable arguments for formatting
 */
#define LOGC(TAG, FMT, ...)                                                       \
    do                                                                            \
    {                                                                             \
        if (LOG_COMPILE_LEVEL <= 4)                                               \
            Logger::instance().logf(LogLevel::Critical, TAG, FMT, ##__VA_ARGS__); \
    } while (0)

/** @} */ // End of LoggingMacros group