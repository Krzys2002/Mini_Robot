#include "infrastructure/logging/LogSinks.hpp"

#include <system_error>
#include <utility>

namespace micro_robot {

void StreamLogSink::write(const LogRecord& record)
{
    stream_ << formatLogRecord(record) << '\n';
    stream_.flush();
}

FileLogSink::FileLogSink(std::filesystem::path path) : path_(std::move(path))
{
    try {
        if (path_.has_parent_path()) {
            std::error_code ignored;
            std::filesystem::create_directories(path_.parent_path(), ignored);
        }
        file_.open(path_, std::ios::out | std::ios::app);
    } catch (...) {
        // Leave the sink closed; callers detect this through isOpen().
    }
}

void FileLogSink::write(const LogRecord& record)
{
    if (!file_.is_open()) return;
    file_ << formatLogRecord(record) << '\n';
    file_.flush();
}

} // namespace micro_robot
