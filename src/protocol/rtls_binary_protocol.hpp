#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace rtls::protocol {

static constexpr uint16_t kFrameMagic = 0x4C52; // "RL", little-endian on the wire.
static constexpr uint8_t kFrameVersion = 1;
static constexpr size_t kFrameHeaderSize = 10;

enum class FrameType : uint8_t {
    CommandAck = 1,
    FirmwareInfo = 2,
    TdoaDistances = 3,
    ConfigList = 4,
    ConfigSnapshot = 5,
    LedState = 6,
    Heartbeat = 16,
    LogMessage = 17,
    TdoaEstimatorStatus = 32,
    TdoaAnchorStats = 33,
    TdoaPositionEstimatorStatus = 34,
    TdoaPositionEstimatorEvents = 35,
};

enum class StatusCode : uint8_t {
    Ok = 0,
    Error = 1,
    NotSupported = 2,
    InvalidMode = 3,
    NotFound = 4,
    InvalidName = 5,
    FileSystemError = 6,
};

template <size_t Capacity>
class BinaryFrameBuilder {
public:
    void Begin(FrameType type, StatusCode status = StatusCode::Ok)
    {
        m_size = 0;
        m_truncated = false;
        AppendU16(kFrameMagic);
        AppendU8(kFrameVersion);
        AppendU8(static_cast<uint8_t>(type));
        AppendU16(0); // payload length, patched by Finish().
        AppendU16(m_sequence++);
        AppendU8(static_cast<uint8_t>(status));
        AppendU8(0); // reserved
    }

    void Finish()
    {
        const uint16_t payloadLength = m_size > kFrameHeaderSize
            ? static_cast<uint16_t>(m_size - kFrameHeaderSize)
            : 0;
        m_buffer[4] = static_cast<uint8_t>(payloadLength & 0xFF);
        m_buffer[5] = static_cast<uint8_t>((payloadLength >> 8) & 0xFF);
    }

    void AppendBool(bool value) { AppendU8(value ? 1 : 0); }

    void AppendU8(uint8_t value)
    {
        AppendBytes(&value, sizeof(value));
    }

    void AppendU16(uint16_t value)
    {
        uint8_t bytes[2] = {
            static_cast<uint8_t>(value & 0xFF),
            static_cast<uint8_t>((value >> 8) & 0xFF),
        };
        AppendBytes(bytes, sizeof(bytes));
    }

    void SetU16(size_t offset, uint16_t value)
    {
        if (offset + 1 >= Capacity) {
            return;
        }
        m_buffer[offset] = static_cast<uint8_t>(value & 0xFF);
        m_buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    }

    void AppendU32(uint32_t value)
    {
        uint8_t bytes[4] = {
            static_cast<uint8_t>(value & 0xFF),
            static_cast<uint8_t>((value >> 8) & 0xFF),
            static_cast<uint8_t>((value >> 16) & 0xFF),
            static_cast<uint8_t>((value >> 24) & 0xFF),
        };
        AppendBytes(bytes, sizeof(bytes));
    }

    void AppendI32(int32_t value)
    {
        AppendU32(static_cast<uint32_t>(value));
    }

    void AppendString(const char* value)
    {
        if (value == nullptr) {
            AppendU16(0);
            return;
        }
        const size_t len = strlen(value);
        const uint16_t wireLen = len > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(len);
        AppendU16(wireLen);
        AppendBytes(reinterpret_cast<const uint8_t*>(value), wireLen);
    }

    void AppendString(const char* value, size_t len)
    {
        if (value == nullptr) {
            AppendU16(0);
            return;
        }
        const uint16_t wireLen = len > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(len);
        AppendU16(wireLen);
        AppendBytes(reinterpret_cast<const uint8_t*>(value), wireLen);
    }

    void AppendBytes(const void* data, size_t len)
    {
        if (data == nullptr || len == 0) {
            return;
        }
        if (m_size + len > Capacity) {
            len = Capacity > m_size ? Capacity - m_size : 0;
            m_truncated = true;
        }
        if (len > 0) {
            memcpy(m_buffer + m_size, data, len);
            m_size += len;
        }
    }

    uint8_t* Data() { return m_buffer; }
    const uint8_t* Data() const { return m_buffer; }
    size_t Size() const { return m_size; }
    bool Truncated() const { return m_truncated; }

private:
    uint8_t m_buffer[Capacity] = {};
    size_t m_size = 0;
    bool m_truncated = false;
    uint16_t m_sequence = 0;
};

inline int32_t MetersToMillimeters(float meters)
{
    const float scaled = meters * 1000.0f;
    return scaled >= 0.0f
        ? static_cast<int32_t>(scaled + 0.5f)
        : static_cast<int32_t>(scaled - 0.5f);
}

} // namespace rtls::protocol
