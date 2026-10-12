#include "infrastructure/logging/Logger.hpp"

#include <iomanip>
#include <sstream>
#include <utility>

namespace micro_robot {
namespace {

void appendEscaped(std::ostringstream& out, std::string_view text)
{
    for (const char c : text) {
        switch (c) {
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        default: out << c; break;
        }
    }
}

} // namespace

std::string formatLogRecord(const LogRecord& record)
{
    using namespace std::chrono;
    // Calendar arithmetic avoids the non-portable gmtime_r/gmtime_s split.
    const auto millis = floor<milliseconds>(record.time.time_since_epoch());
    const auto day = floor<days>(millis);
    const year_month_day date{sys_days{day}};
    const hh_mm_ss timeOfDay{millis - day};

    std::ostringstream out;
    out << std::setfill('0')
        << std::setw(4) << static_cast<int>(date.year()) << '-'
        << std::setw(2) << static_cast<unsigned>(date.month()) << '-'
        << std::setw(2) << static_cast<unsigned>(date.day()) << 'T'
        << std::setw(2) << timeOfDay.hours().count() << ':'
        << std::setw(2) << timeOfDay.minutes().count() << ':'
        << std::setw(2) << timeOfDay.seconds().count() << '.'
        << std::setw(3) << timeOfDay.subseconds().count() << 'Z'
        << std::setfill(' ')
        << " [" << toString(record.level) << "] [" << record.module << "] ";
    if (record.robotId) out << "robot=" << record.robotId->value() << ' ';
    appendEscaped(out, record.message);
    return out.str();
}

Logger::Logger(LogLevel minimumLevel, ClockFunction clock)
    : clock_(clock ? std::move(clock) : ClockFunction{[] { return WallClock::now(); }}),
      minimumLevel_(minimumLevel)
{
}

void Logger::addSink(std::shared_ptr<ILogSink> sink)
{
    if (!sink) return;
    const std::scoped_lock lock{mutex_};
    sinks_.push_back(std::move(sink));
}

void Logger::setMinimumLevel(LogLevel level) noexcept
{
    minimumLevel_.store(level, std::memory_order_relaxed);
}

LogLevel Logger::minimumLevel() const noexcept
{
    return minimumLevel_.load(std::memory_order_relaxed);
}

bool Logger::isEnabled(LogLevel level) const noexcept
{
    return level >= minimumLevel();
}

void Logger::log(LogLevel level, std::string_view module, std::string_view message,
                 std::optional<RobotId> robotId) noexcept
{
    if (!isEnabled(level)) return;
    try {
        const std::scoped_lock lock{mutex_};
        const LogRecord record{clock_(), level, std::string{module}, std::string{message}, robotId};
        for (const auto& sink : sinks_) {
            try {
                sink->write(record);
            } catch (...) {
                // One failing sink must neither stop the others nor reach the caller.
            }
        }
    } catch (...) {
        // Logging is diagnostic; allocation or clock failures are dropped silently.
    }
}

LogChannel Logger::channel(std::string module)
{
    return LogChannel{*this, std::move(module)};
}

} // namespace micro_robot
