#include "simulation/FakeCameraSource.hpp"

#include <stdexcept>
#include <utility>

namespace micro_robot {

FakeCameraSource::FakeCameraSource(FakeCameraConfig config) : config_(std::move(config))
{
    if (config_.frameInterval <= Duration::zero()) {
        throw std::invalid_argument("Fake camera frame interval must be positive");
    }
}

void FakeCameraSource::setEventSink(std::weak_ptr<ICameraEvents> sink)
{
    sink_ = std::move(sink);
}

CameraInfo FakeCameraSource::info() const { return config_.info; }
CameraState FakeCameraSource::state() const { return state_; }
void FakeCameraSource::start() { desiredActive_ = true; cancelledStart_ = false; }
void FakeCameraSource::stop()
{
    cancelledStart_ = cancelledStart_ || (desiredActive_ && state_ == CameraState::Stopped);
    desiredActive_ = false;
}

void FakeCameraSource::advanceTo(Timestamp now)
{
    if (advancing_) {
        throw std::logic_error("Recursive fake camera advancement");
    }
    if (lastAdvance_ && now < *lastAdvance_) {
        throw std::invalid_argument("Fake camera time cannot go backward");
    }
    // Reset even if allocation fails. Event sinks must not throw by contract.
    struct AdvancementGuard {
        bool& active;
        explicit AdvancementGuard(bool& value) : active(value) { active = true; }
        ~AdvancementGuard() { active = false; }
    } guard{advancing_};
    lastAdvance_ = now;

    if (desiredActive_ && state_ == CameraState::Stopped) {
        state_ = CameraState::Streaming;
        startedAt_ = now;
        lastFrameTick_.reset();
        sequenceBase_ = nextSequence_;
        if (auto sink = sink_.lock()) {
            sink->onCameraStateChanged(state_, now);
        }
    }
    // A state callback may request stop; do not deliver a frame afterwards.
    if (!desiredActive_) {
        if (state_ != CameraState::Stopped || cancelledStart_) {
            cancelledStart_ = false;
            state_ = CameraState::Stopped;
            if (auto sink = sink_.lock()) {
                sink->onCameraStateChanged(state_, now);
            }
        }
        return;
    }

    const auto tick = static_cast<std::uint64_t>((now - startedAt_) / config_.frameInterval);
    if (lastFrameTick_ && tick == *lastFrameTick_) {
        return;
    }
    const auto sequence = sequenceBase_ + tick;
    auto sink = sink_.lock();
    if (sink) {
        std::vector<std::byte> pixels(8 * 6);
        for (std::size_t index = 0; index < pixels.size(); ++index) {
            pixels[index] = static_cast<std::byte>((config_.patternSeed + sequence + index) % 256);
        }
        auto frame = std::make_shared<const Frame>(Frame{
            sequence, now, 8, 6, 8, PixelFormat::Gray8,
            std::make_shared<const std::vector<std::byte>>(std::move(pixels))
        });
        lastFrameTick_ = tick;
        nextSequence_ = sequence + 1;
        sink->onFrame(std::move(frame));
    } else {
        lastFrameTick_ = tick;
        nextSequence_ = sequence + 1;
    }
}

} // namespace micro_robot
