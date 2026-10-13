#pragma once

#include "common/RobotId.hpp"

#include <chrono>
#include <optional>
#include <string>
#include <string_view>

namespace micro_robot {

// Operational log entries use wall-clock time so that they can be correlated with
// the outside world. Latency/freshness measurements keep using monotonic Timestamp.
using WallClock = std::chrono::system_clock;
using WallTime = WallClock::time_point;

// Ordered by severity; a logger drops records below its minimum level.
enum class LogLevel { Debug, Info, Warning, Error };

[[nodiscard]] constexpr std::string_view toString(LogLevel level) noexcept
{
    switch (level) {
    case LogLevel::Debug: return "DEBUG";
    case LogLevel::Info: return "INFO";
    case LogLevel::Warning: return "WARNING";
    case LogLevel::Error: return "ERROR";
    }
    return "UNKNOWN";
}

struct LogRecord {
    WallTime time;
    LogLevel level{LogLevel::Info};
    std::string module;  // Producing subsystem, e.g. "app", "camera", "transport".
    std::string message; // Human-readable text, not a structured measurement.
    std::optional<RobotId> robotId;
};

// One line, e.g. "2026-10-12T09:41:07.250Z [ERROR] [transport] robot=3 Link lost".
// Time is UTC with millisecond precision. Line breaks in the message are escaped
// so that every record occupies exactly one line.
[[nodiscard]] std::string formatLogRecord(const LogRecord& record);

} // namespace micro_robot
