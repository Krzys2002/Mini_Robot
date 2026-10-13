#pragma once

#include "infrastructure/logging/LogRecord.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace micro_robot {

// Destination for log records. write() is called with the logger's lock held, so
// records reach each sink one at a time and in order. Implementations must not log
// through the same logger from write(). Exceptions are caught and discarded by the
// logger, so a failing sink cannot crash its callers.
class ILogSink {
public:
    virtual ~ILogSink() = default;
    virtual void write(const LogRecord& record) = 0;

protected:
    ILogSink() = default;
    ILogSink(const ILogSink&) = default;
    ILogSink& operator=(const ILogSink&) = default;
};

class LogChannel;

// Shared application logger. The composition root owns one instance and injects it
// (or LogChannels derived from it) into modules; there is no global logger.
// Thread-safe: any thread may log, configure the level, or add sinks.
class Logger final {
public:
    using ClockFunction = std::function<WallTime()>;

    // An empty clock function uses WallClock::now(). Tests may inject a fixed clock.
    explicit Logger(LogLevel minimumLevel = LogLevel::Info, ClockFunction clock = {});

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    // Sinks are shared so that their creator may keep inspecting them; null is ignored.
    void addSink(std::shared_ptr<ILogSink> sink);

    void setMinimumLevel(LogLevel level) noexcept;
    [[nodiscard]] LogLevel minimumLevel() const noexcept;
    [[nodiscard]] bool isEnabled(LogLevel level) const noexcept;

    // Never throws; failures inside formatting or sinks are swallowed.
    void log(LogLevel level, std::string_view module, std::string_view message,
             std::optional<RobotId> robotId = std::nullopt) noexcept;

    // The logger must outlive the returned channel.
    [[nodiscard]] LogChannel channel(std::string module);

private:
    ClockFunction clock_;
    std::atomic<LogLevel> minimumLevel_;
    std::mutex mutex_;
    std::vector<std::shared_ptr<ILogSink>> sinks_;
};

// Lightweight, copyable per-module handle: fixes the module name of each record.
// Non-owning; the referenced Logger must outlive every channel.
class LogChannel final {
public:
    LogChannel(Logger& logger, std::string module) : logger_(&logger), module_(std::move(module)) {}

    [[nodiscard]] const std::string& module() const noexcept { return module_; }

    void debug(std::string_view message, std::optional<RobotId> robotId = std::nullopt) const noexcept
    {
        logger_->log(LogLevel::Debug, module_, message, robotId);
    }
    void info(std::string_view message, std::optional<RobotId> robotId = std::nullopt) const noexcept
    {
        logger_->log(LogLevel::Info, module_, message, robotId);
    }
    void warning(std::string_view message, std::optional<RobotId> robotId = std::nullopt) const noexcept
    {
        logger_->log(LogLevel::Warning, module_, message, robotId);
    }
    void error(std::string_view message, std::optional<RobotId> robotId = std::nullopt) const noexcept
    {
        logger_->log(LogLevel::Error, module_, message, robotId);
    }

private:
    Logger* logger_;
    std::string module_;
};

} // namespace micro_robot
