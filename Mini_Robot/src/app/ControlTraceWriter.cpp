#include "app/ControlTraceWriter.hpp"

#include <system_error>
#include <utility>

namespace micro_robot {
ControlTraceWriter::ControlTraceWriter(LogChannel log, std::filesystem::path runDirectory)
    : log_(std::move(log)), runDirectory_(std::move(runDirectory)), worker_([this] {
    std::ofstream runFile;
    std::filesystem::path activePath;
    for (;;) {
        Item item;
        {
            std::unique_lock lock(mutex_);
            ready_.wait(lock, [this] { return closing_ || !pending_.empty(); });
            if (pending_.empty()) return;
            item = std::move(pending_.front());
            pending_.pop_front();
        }
        if (item.kind == Item::Kind::BeginRun) {
            if (runFile.is_open()) runFile.close();
            activePath = item.path;
            std::error_code error;
            if (activePath.has_parent_path())
                std::filesystem::create_directories(activePath.parent_path(), error);
            if (!error) runFile.open(activePath, std::ios::out | std::ios::trunc);
            if (!runFile.is_open()) {
                log_.error("Cannot open control experiment export: " + activePath.string());
                std::lock_guard lock(mutex_);
                runExportState_ = ControlRunExportState::Failed;
                continue;
            }
            runFile << item.line << '\n';
            runFile.flush();
            if (!runFile) {
                log_.error("Cannot write control experiment header: " + activePath.string());
                runFile.close();
                std::lock_guard lock(mutex_);
                runExportState_ = ControlRunExportState::Failed;
            } else {
                std::lock_guard lock(mutex_);
                runExportState_ = ControlRunExportState::Open;
            }
            continue;
        }
        if (item.kind == Item::Kind::FinishRun) {
            if (runFile.is_open()) {
                runFile << item.line << '\n';
                runFile.flush();
                const bool written = static_cast<bool>(runFile);
                if (!written) log_.error("Cannot finish control experiment export: " + activePath.string());
                runFile.close();
                std::lock_guard lock(mutex_);
                runExportState_ = written ? ControlRunExportState::Complete : ControlRunExportState::Failed;
            }
            activePath.clear();
            continue;
        }

        if (item.kind == Item::Kind::Trace) log_.info(item.line);
        if (item.kind == Item::Kind::RunSample) {
            if (runFile.is_open()) {
                runFile << item.line << '\n';
                runFile.flush();
                if (!runFile) {
                    log_.error("Cannot append control experiment export: " + activePath.string());
                    std::lock_guard lock(mutex_);
                    runExportState_ = ControlRunExportState::Failed;
                }
            } else {
                std::lock_guard lock(mutex_);
                ++dropped_;
            }
        }
    }
}) {}
ControlTraceWriter::~ControlTraceWriter()
{
    { std::lock_guard lock(mutex_); closing_ = true; }
    ready_.notify_one();
    worker_.join();
}
bool ControlTraceWriter::enqueue(std::string line)
{
    {
        std::lock_guard lock(mutex_);
        const auto limit = runQueuedActive_ ? capacity - 2 : capacity;
        if (closing_ || line.size() > maximumLineBytes || pending_.size() >= limit) {
            ++dropped_;
            return false;
        }
        pending_.push_back({Item::Kind::Trace, {}, std::move(line)});
    }
    ready_.notify_one();
    return true;
}
bool ControlTraceWriter::enqueueSample(std::string line)
{
    {
        std::lock_guard lock(mutex_);
        if (closing_ || !runQueuedActive_ || runExportState_ == ControlRunExportState::Failed
            || line.size() > maximumLineBytes
            || pending_.size() >= capacity - 2) {
            ++dropped_;
            return false;
        }
        pending_.push_back({Item::Kind::RunSample, {}, std::move(line)});
    }
    ready_.notify_one();
    return true;
}
std::optional<std::filesystem::path> ControlTraceWriter::beginRun(std::string fileName, std::string header)
{
    const std::filesystem::path name{fileName};
    if (runDirectory_.empty() || name.empty() || name.has_parent_path() || name.filename() != name
        || header.size() > maximumLineBytes)
        return {};
    const auto path = runDirectory_ / name;
    {
        std::lock_guard lock(mutex_);
        if (closing_ || runQueuedActive_ || pending_.size() >= capacity - 1) {
            ++dropped_;
            return {};
        }
        pending_.push_back({Item::Kind::BeginRun, path, std::move(header)});
        runQueuedActive_ = true;
        runExportState_ = ControlRunExportState::Opening;
    }
    ready_.notify_one();
    return path;
}
bool ControlTraceWriter::finishRun(std::string footer)
{
    {
        std::lock_guard lock(mutex_);
        if (closing_ || !runQueuedActive_ || footer.size() > maximumLineBytes) {
            ++dropped_;
            return false;
        }
        pending_.push_back({Item::Kind::FinishRun, {}, std::move(footer)});
        runQueuedActive_ = false;
    }
    ready_.notify_one();
    return true;
}
std::size_t ControlTraceWriter::dropped() const { std::lock_guard lock(mutex_); return dropped_; }
ControlRunExportState ControlTraceWriter::runExportState() const
{
    std::lock_guard lock(mutex_);
    return runExportState_;
}
} // namespace micro_robot
