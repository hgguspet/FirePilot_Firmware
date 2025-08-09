#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <cstdarg>

/**
 * @brief Minimal streaming JSON writer into a caller-provided buffer.
 * - No heap, no DOM
 * - Works on Arduino
 * - Proper string escaping
 * - Supports nested objects/arrays (up to kMaxDepth)
 *
 * Contract: The produced bytes remain valid in the caller's buffer until reused.
 */
class JsonBufWriter
{
public:
    explicit JsonBufWriter(uint8_t *buf, size_t cap)
        : _buf(buf), _cap(cap), _len(0), _err(false), _depth(0),
          _floatPrec(3), _expectValue(false)
    {
    }

    void reset(uint8_t *buf, size_t cap)
    {
        _buf = buf;
        _cap = cap;
        _len = 0;
        _err = false;
        _depth = 0;
        _expectValue = false;
        _floatPrec = 3;
    }

    void setFloatPrecision(uint8_t digits) { _floatPrec = digits; }

    // Root constructors
    bool beginObject() { return open('{', /*isObj=*/true); }
    bool beginArray() { return open('[', /*isObj=*/false); }

    bool endObject() { return close('}', /*isObj=*/true); }
    bool endArray() { return close(']', /*isObj=*/false); }

    // Object key → call before value() when inside an object
    bool key(const char *k)
    {
        if (_err || !inObject() || !_expectValue)
            return fail();
        if (!commaIfNeeded())
            return false;
        if (!string(k))
            return false;
        return append(':');
    }

    // Values
    bool value(const char *s) { return writeValueString(s); }
    bool value(const char *s, size_t n) { return writeValueStringN(s, n); }
    bool value(bool b) { return writeRaw(b ? "true" : "false", b ? 4u : 5u); }
    bool value(int32_t v) { return writeNum("%ld", (long)v); }
    bool value(uint32_t v) { return writeNum("%lu", (unsigned long)v); }
    bool value(int64_t v) { return writeNum("%lld", (long long)v); }
    bool value(uint64_t v) { return writeNum("%llu", (unsigned long long)v); }
    bool value(float f) { return writeFloat((double)f); }
    bool value(double d) { return writeFloat(d); }
    bool null() { return writeRaw("null", 4); }

    // Raw JSON fragment (caller guarantees validity)
    bool raw(const char *json, size_t n)
    {
        if (!commaIfNeeded())
            return false;
        return writeRaw(json, n);
    }

    // Finish writing; returns pointer and length. Must have closed all scopes.
    bool finalize(const uint8_t *&out, size_t &len)
    {
        if (_err || _depth != 0)
            return false;
        out = _buf;
        len = _len;
        return true;
    }

    // Current error state/length
    bool ok() const { return !_err; }
    size_t size() const { return _len; }

private:
    struct Frame
    {
        bool isObj;
        bool first;
        bool expectValue;
    };
    static constexpr size_t kMaxDepth = 8;

    uint8_t *_buf;
    size_t _cap;
    size_t _len;
    bool _err;
    uint8_t _depth;
    uint8_t _floatPrec;
    bool _expectValue; // only for root when not in any frame
    Frame _stack[kMaxDepth];

    bool inAny() const { return _depth > 0; }
    bool inObject() const { return inAny() && _stack[_depth - 1].isObj; }
    Frame &cur() { return _stack[_depth - 1]; }

    bool open(char ch, bool isObj)
    {
        if (_err || (_depth == 0 && _len != 0))
            return fail(); // single root
        if (!commaIfNeeded())
            return false;
        if (!append(ch))
            return false;
        if (_depth >= kMaxDepth)
            return fail();
        _stack[_depth++] = Frame{isObj, /*first=*/true, /*expectValue=*/!isObj};
        _expectValue = false; // root flag only
        return true;
    }

    bool close(char ch, bool isObj)
    {
        if (_err || !inAny() || cur().isObj != isObj)
            return fail();
        if (!append(ch))
            return false;
        _depth--;
        // After closing, the parent no longer expects a value
        if (inAny())
        {
            cur().expectValue = false;
        }
        else
        {
            _expectValue = false;
        }
        return true;
    }

    bool commaIfNeeded()
    {
        if (_err)
            return false;
        if (inAny())
        {
            if (cur().isObj)
            {
                // In object: key() handles comma & colon; here we’re placing values for arrays only
                if (!cur().expectValue)
                {
                    if (!cur().first && !append(','))
                        return false;
                    cur().first = false;
                    // if object, caller must call key() next; if array, value next
                }
            }
            else
            {
                if (!cur().first && !append(','))
                    return false;
                cur().first = false;
            }
        }
        else
        {
            // root: allow only a single value
            if (_len != 0)
                return fail();
        }
        return true;
    }

    bool writeValueString(const char *s)
    {
        if (!commaIfNeeded())
            return false;
        return string(s);
    }
    bool writeValueStringN(const char *s, size_t n)
    {
        if (!commaIfNeeded())
            return false;
        return stringN(s, n);
    }

    bool writeNum(const char *fmt, ...)
    {
        if (!commaIfNeeded())
            return false;
        va_list ap;
        va_start(ap, fmt);
        bool ok = vsprint(fmt, ap);
        va_end(ap);
        setParentAfterValue();
        return ok;
    }

    bool writeFloat(double d)
    {
        if (!commaIfNeeded())
            return false;
        // Avoid locale issues; snprintf with precision
        char fmt[8];
        // "%.3f" style
        snprintf(fmt, sizeof(fmt), "%%.%df", _floatPrec);
        int n = snprint(fmt, d);
        if (n < 0)
            return fail();
        setParentAfterValue();
        return true;
    }

    bool string(const char *s)
    {
        if (!append('\"'))
            return false;
        for (const unsigned char *p = (const unsigned char *)s; *p; ++p)
        {
            if (!escapeChar(*p))
                return false;
        }
        if (!append('\"'))
            return false;
        setParentAfterValueIfArrayOrRoot();
        return true;
    }

    bool stringN(const char *s, size_t n)
    {
        if (!append('\"'))
            return false;
        const unsigned char *p = (const unsigned char *)s;
        for (size_t i = 0; i < n; ++i)
        {
            if (!escapeChar(p[i]))
                return false;
        }
        if (!append('\"'))
            return false;
        setParentAfterValueIfArrayOrRoot();
        return true;
    }

    bool escapeChar(unsigned char c)
    {
        switch (c)
        {
        case '\"':
            return writeRaw("\\\"", 2);
        case '\\':
            return writeRaw("\\\\", 2);
        case '\b':
            return writeRaw("\\b", 2);
        case '\f':
            return writeRaw("\\f", 2);
        case '\n':
            return writeRaw("\\n", 2);
        case '\r':
            return writeRaw("\\r", 2);
        case '\t':
            return writeRaw("\\t", 2);
        default:
            if (c < 0x20)
            { // control → \u00XX
                char u[6] = {'\\', 'u', '0', '0', 0, 0};
                static const char *hex = "0123456789abcdef";
                u[4] = hex[(c >> 4) & 0xF];
                u[5] = hex[c & 0xF];
                return writeRaw(u, 6);
            }
            return append((char)c);
        }
    }

    bool writeRaw(const char *s, size_t n)
    {
        if (_err || !ensure(n))
            return fail();
        for (size_t i = 0; i < n; ++i)
            _buf[_len++] = (uint8_t)s[i];
        setParentAfterValueIfArrayOrRoot();
        return true;
    }

    bool append(char c)
    {
        if (_err || !ensure(1))
            return fail();
        _buf[_len++] = (uint8_t)c;
        return true;
    }

    bool ensure(size_t n) const { return _len + n <= _cap; }
    bool fail()
    {
        _err = true;
        return false;
    }

    // Printing helpers (no std::string)
    int snprint(const char *fmt, double d)
    {
        if (!_buf)
            return -1;
        int n = snprintf((char *)_buf + _len, _cap > _len ? _cap - _len : 0, fmt, d);
        if (n <= 0)
            return -1;
        if (!ensure((size_t)n))
        {
            _err = true;
            return -1;
        }
        _len += (size_t)n;
        return n;
    }
    bool vsprint(const char *fmt, va_list ap)
    {
        if (!_buf)
            return fail();
        int n = vsnprintf((char *)_buf + _len, _cap > _len ? _cap - _len : 0, fmt, ap);
        if (n < 0)
            return fail();
        if (!ensure((size_t)n))
            return fail();
        _len += (size_t)n;
        return true;
    }

    // State transitions after placing a value
    void setParentAfterValue()
    {
        if (inAny())
            cur().expectValue = cur().isObj; // after a value in object, expect next key
        else
            _expectValue = false;
    }
    void setParentAfterValueIfArrayOrRoot()
    {
        if (inAny())
        {
            if (!cur().isObj)
                cur().expectValue = false; // array element done
        }
        else
        {
            _expectValue = false;
        }
    }
};
