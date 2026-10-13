#pragma once

#include "infrastructure/logging/Logger.hpp"

#include <filesystem>
#include <fstream>
#include <ostream>

namespace micro_robot {

// Writes formatted lines to a caller-owned stream (e.g. std::clog), which must
// outlive the sink. Synchronization is provided by the logger.
class StreamLogSink final : public ILogSink {
public:
    explicit StreamLogSink(std::ostream& stream) : stream_(stream) {}
    void write(const LogRecord& record) override;

private:
    std::ostream& stream_;
};

// Appends formatted lines to a file, creating missing parent directories. Each
// record is flushed so that the log survives a crash. Opening never throws: check
// isOpen() and fall back to other sinks. There is no rotation; the file grows
// until removed externally.
class FileLogSink final : public ILogSink {
public:
    explicit FileLogSink(std::filesystem::path path);

    [[nodiscard]] bool isOpen() const noexcept { return file_.is_open(); }
    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }

    void write(const LogRecord& record) override;

private:
    std::filesystem::path path_;
    std::ofstream file_;
};

} // namespace micro_robot
