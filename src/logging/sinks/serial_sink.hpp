#pragma once
#include "../ilog_sink.hpp"
class HardwareSerial;

class SerialSink : public ILogSink
{
public:
    explicit SerialSink(void *serial_like); // e.g. &Serial
    void write(const LogRecord &r) override;

private:
    void *_ser; // stored as opaque to keep header light
};
