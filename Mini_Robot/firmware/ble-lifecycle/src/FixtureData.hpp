#pragma once

#include <array>
#include <cstdint>

// Hardware-free bytes shared by the bench firmware and desktop interoperability tests.
namespace ble_fixture {
inline void put16(std::uint8_t* target, std::uint16_t value) {
    target[0] = value & 0xff;
    target[1] = value >> 8;
}
inline void put32(std::uint8_t* target, std::uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) target[i] = (value >> (8 * i)) & 0xff;
}
inline bool parseId(const char* text, std::uint32_t& output) {
    if (!*text) return false;
    std::uint64_t value = 0;
    for (; *text; ++text) {
        if (*text < '0' || *text > '9') return false;
        value = value * 10 + static_cast<unsigned>(*text - '0');
        if (value > UINT32_MAX) return false;
    }
    output = static_cast<std::uint32_t>(value);
    return true;
}
inline std::array<std::uint8_t, 18> robotInfo(std::uint32_t id, bool wrongId, bool wrongVersion) {
    std::array<std::uint8_t, 18> record{1, 1, 1, 7, 0, 0, 0, 0, 1, 0,
                                     0x2c, 1, 0x70, 0x17, 0xf4, 1, 0x64, 0};
    put32(record.data() + 3, wrongId ? id ^ 1U : id);
    if (wrongVersion) record[1] = record[2] = 2;
    return record;
}
inline std::array<std::uint8_t, 18> idleTelemetry(std::uint32_t now, std::uint16_t sequence) {
    std::array<std::uint8_t, 18> frame{1, 0x80};
    put16(frame.data() + 2, sequence);
    put32(frame.data() + 8, now);
    put16(frame.data() + 12, 0xffff); // Battery unknown.
    put16(frame.data() + 14, 0x8000); // Velocity unknown, never measured.
    put16(frame.data() + 16, 0x8000);
    return frame;
}
} // namespace ble_fixture
