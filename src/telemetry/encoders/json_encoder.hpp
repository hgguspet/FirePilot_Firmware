#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

class JsonEncoder
{
public:
    explicit JsonEncoder(size_t capacity)
        : _capacity(capacity), _buffer(new char[capacity]) {}

    // Start new JSON object
    void begin()
    {
        _doc.clear();
    }

    // Add key/value pairs
    template <typename T>
    void add(const char *key, const T &value)
    {
        _doc[key] = value;
    }

    // Finalize and get payload pointer/length
    bool finalize(const char *&outData, size_t &outLen)
    {
        size_t n = serializeJson(_doc, _buffer, _capacity);
        if (n == 0)
        {
            return false;
        }
        outData = reinterpret_cast<const char *>(_buffer);
        outLen = n;
        return true;
    }

private:
    size_t _capacity;
    char *_buffer;
    JsonDocument _doc;
};