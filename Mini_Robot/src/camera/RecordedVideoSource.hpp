#pragma once

#include "camera/ICameraSource.hpp"

#include <chrono>
#include <filesystem>
#include <memory>

namespace micro_robot {

struct RecordedVideoConfig {
    std::filesystem::path file;
    // Fixed virtual cadence, independent of decoder/media clock and wall time.
    Duration frameInterval{std::chrono::milliseconds{40}};
    bool loop{};
};

// Thread-affine, explicitly clock-driven replay adapter. Composition advances
// it on an acquisition worker (or test thread), never the GUI thread: advanceTo
// may block on local file I/O/decoding. All calls/events stay on that same thread.
// Consumers need only ICameraSource; the driver alone knows advanceTo.
class RecordedVideoSource final : public ICameraSource {
public:
    explicit RecordedVideoSource(RecordedVideoConfig config);
    ~RecordedVideoSource() override;
    RecordedVideoSource(const RecordedVideoSource&) = delete;
    RecordedVideoSource& operator=(const RecordedVideoSource&) = delete;
    RecordedVideoSource(RecordedVideoSource&&) = delete;
    RecordedVideoSource& operator=(RecordedVideoSource&&) = delete;

    void setEventSink(std::weak_ptr<ICameraEvents> sink) override;
    [[nodiscard]] CameraInfo info() const override;
    [[nodiscard]] CameraState state() const override;
    void start() override;
    void stop() override;

    // Deferred requests complete here. Time must be nondecreasing; recursive
    // advancement throws. At most one frame is decoded/published per call, with
    // no frame queue and no automatic skipping. Late calls consume sequential
    // frames at their scheduled virtual timestamps, not at the supplied now.
    // Start after stop/EOF/failure reopens at frame zero; sequence and timestamp
    // remain increasing for the object's lifetime. Looping also preserves both.
    void advanceTo(Timestamp now);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace micro_robot
