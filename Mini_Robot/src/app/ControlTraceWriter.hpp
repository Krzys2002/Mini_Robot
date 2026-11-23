#pragma once
#include "infrastructure/logging/Logger.hpp"
#include <condition_variable>
#include <deque>
#include <thread>

namespace micro_robot {
// Bounded asynchronous handoff: no sink/file I/O on the caller. Logger outlives
// this writer. Destroy after the GUI event loop exits: destruction drains/joins.
class ControlTraceWriter final {
public:
    static constexpr std::size_t capacity = 256;
    static constexpr std::size_t maximumLineBytes = 8192;
    explicit ControlTraceWriter(LogChannel log);
    ~ControlTraceWriter();
    [[nodiscard]] bool enqueue(std::string line);
    [[nodiscard]] std::size_t dropped() const;
private:
    LogChannel log_;
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<std::string> pending_;
    std::size_t dropped_{};
    bool closing_{};
    std::thread worker_;
};
} // namespace micro_robot
