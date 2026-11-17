#include "app/ControlTraceWriter.hpp"

namespace micro_robot {
ControlTraceWriter::ControlTraceWriter(LogChannel log) : log_(std::move(log)), worker_([this] {
    for (;;) {
        std::string line;
        {
            std::unique_lock lock(mutex_);
            ready_.wait(lock, [this] { return closing_ || !pending_.empty(); });
            if (pending_.empty()) return;
            line = std::move(pending_.front());
            pending_.pop_front();
        }
        log_.info(line);
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
        if (closing_ || line.size() > maximumLineBytes || pending_.size() >= capacity) { ++dropped_; return false; }
        pending_.push_back(std::move(line));
    }
    ready_.notify_one();
    return true;
}
std::size_t ControlTraceWriter::dropped() const { std::lock_guard lock(mutex_); return dropped_; }
} // namespace micro_robot
