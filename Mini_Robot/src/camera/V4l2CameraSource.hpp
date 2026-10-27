#pragma once

#include "camera/ICameraSourceFactory.hpp"

#include <QObject>
#include <QThread>

#include <cstdint>
#include <memory>

namespace micro_robot {

// OpenCV capture adapter using the V4L2 backend (cv::CAP_V4L2) on Linux. The device
// ID is the node path reported by V4l2CameraDeviceDiscovery. Create and use on a
// Qt event-loop thread; public calls and callbacks stay on that thread. Opening,
// resolution negotiation, capture, and pixel conversion run on a dedicated worker
// that polls for frames in short slices, so stop requests are served promptly;
// frame delivery has a single latest-frame slot. Creation does not open the device.
//
// Start failures are asynchronous and end in Fault: a missing node reports
// Unavailable, an access error PermissionDenied, and other open errors
// BackendFailure. While starting or streaming, node removal reports Unavailable and
// Disconnected; read errors, invalid frames, or a three-second absence of frames
// report Fault. All are recoverable: Start reopens the device.
class V4l2CameraSource final : public QObject, public ICameraSource {
public:
    V4l2CameraSource(CameraDevice device, CameraCaptureSettings settings);
    ~V4l2CameraSource() override;
    V4l2CameraSource(const V4l2CameraSource&) = delete;
    V4l2CameraSource& operator=(const V4l2CameraSource&) = delete;

    void setEventSink(std::weak_ptr<ICameraEvents> sink) override;
    [[nodiscard]] CameraInfo info() const override;
    [[nodiscard]] CameraState state() const override;
    void start() override;
    void stop() override;

private:
    class Backend;
    void receiveState(std::uint64_t generation, CameraState state);
    void receiveError(std::uint64_t generation, const CameraError& error);

    CameraDevice device_;
    std::optional<std::string> calibrationId_;
    std::weak_ptr<ICameraEvents> sink_;
    QThread worker_;
    Backend* backend_{}; // Worker thread deletes it when that thread finishes.
    CameraState state_{CameraState::Stopped};
    std::uint64_t generation_{};
    bool startRequested_{};
    bool stopRequested_{};
};

// Creates V4l2CameraSource objects that request the configured capture settings.
// Supports any absolute node path; whether it is a usable camera is reported by
// the source when started.
class V4l2CameraSourceFactory final : public ICameraSourceFactory {
public:
    explicit V4l2CameraSourceFactory(CameraCaptureSettings settings = {});
    [[nodiscard]] bool supports(const CameraDevice& device) const override;
    [[nodiscard]] std::unique_ptr<ICameraSource> create(const CameraDevice& device) override;

private:
    CameraCaptureSettings settings_;
};

} // namespace micro_robot
