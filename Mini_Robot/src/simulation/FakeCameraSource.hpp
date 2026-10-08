#pragma once

#include "camera/ICameraSource.hpp"

#include <chrono>
#include <cstdint>
#include <optional>

namespace micro_robot {

struct FakeCameraConfig {
    CameraInfo info{"fake-camera", "Fake camera"};
    Duration frameInterval{std::chrono::milliseconds{33}};
    std::uint8_t patternSeed{};
};

// Deterministic, thread-affine source driven by composition/test code. Requests
// dispatch no events until advanceTo(); production consumers use ICameraSource.
class FakeCameraSource final : public ICameraSource {
public:
    explicit FakeCameraSource(FakeCameraConfig config = {});
    FakeCameraSource(const FakeCameraSource&) = delete;
    FakeCameraSource& operator=(const FakeCameraSource&) = delete;
    FakeCameraSource(FakeCameraSource&&) = delete;
    FakeCameraSource& operator=(FakeCameraSource&&) = delete;

    void setEventSink(std::weak_ptr<ICameraEvents> sink) override;
    [[nodiscard]] CameraInfo info() const override;
    [[nodiscard]] CameraState state() const override;
    void start() override;
    void stop() override;

    // Time may stay equal or increase. Backward time/recursive advancement throws.
    // Emits at most one 8x6 Gray8 frame, skipping missed frame intervals. Stop/start
    // restarts cadence while sequence numbers remain increasing.
    void advanceTo(Timestamp now);

private:
    FakeCameraConfig config_;
    std::weak_ptr<ICameraEvents> sink_;
    CameraState state_{CameraState::Stopped};
    bool desiredActive_{};
    bool advancing_{};
    std::optional<Timestamp> lastAdvance_;
    Timestamp startedAt_{};
    std::optional<std::uint64_t> lastFrameTick_;
    std::uint64_t sequenceBase_{};
    std::uint64_t nextSequence_{};
};

} // namespace micro_robot
