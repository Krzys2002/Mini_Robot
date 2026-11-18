#pragma once
#include "infrastructure/logging/Logger.hpp"
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <optional>
#include <deque>
#include <thread>

namespace micro_robot {
enum class ControlRunExportState { Idle, Opening, Open, Complete, Failed };
// Bounded asynchronous handoff: no sink/file I/O on the caller. Logger outlives
// this writer. Destroy after the GUI event loop exits: destruction drains/joins.
class ControlTraceWriter final {
public:
    static constexpr std::size_t capacity = 256;
    static constexpr std::size_t maximumLineBytes = 8192;
    explicit ControlTraceWriter(LogChannel log, std::filesystem::path runDirectory = {});
    ~ControlTraceWriter();
    [[nodiscard]] bool enqueue(std::string line);
    [[nodiscard]] bool enqueueSample(std::string line);
    // A run is a JSON Lines file. Begin/end controls reserve queue slots so
    // sample pressure cannot silently omit the final completeness marker.
    [[nodiscard]] std::optional<std::filesystem::path> beginRun(std::string fileName, std::string header);
    [[nodiscard]] bool finishRun(std::string footer);
    [[nodiscard]] ControlRunExportState runExportState() const;
    [[nodiscard]] std::size_t dropped() const;
private:
    struct Item {
        enum class Kind { Trace, RunSample, BeginRun, FinishRun } kind{Kind::Trace};
        std::filesystem::path path;
        std::string line;
    };
    LogChannel log_;
    std::filesystem::path runDirectory_;
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<Item> pending_;
    std::size_t dropped_{};
    ControlRunExportState runExportState_{ControlRunExportState::Idle};
    bool runQueuedActive_{};
    bool closing_{};
    std::thread worker_;
};
} // namespace micro_robot
