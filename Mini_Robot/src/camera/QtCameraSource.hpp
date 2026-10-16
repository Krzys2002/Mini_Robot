#pragma once

#include "camera/ICameraSourceFactory.hpp"

#include <QObject>
#include <QThread>

#include <cstdint>
#include <memory>

namespace micro_robot {

// Qt Multimedia adapter. Public calls and callbacks belong to the creating Qt
// thread. Native capture and pixel conversion run on a dedicated worker; frame
// delivery has a single latest-frame slot. Creation does not open the device.
class QtCameraSource final : public QObject, public ICameraSource {
public:
    explicit QtCameraSource(CameraDevice device);
    ~QtCameraSource() override;
    QtCameraSource(const QtCameraSource&) = delete;
    QtCameraSource& operator=(const QtCameraSource&) = delete;

    void setEventSink(std::weak_ptr<ICameraEvents> sink) override;
    [[nodiscard]] CameraInfo info() const override;
    [[nodiscard]] CameraState state() const override;
    void start() override;
    void stop() override;

private:
    class Backend;
    void beginStart(std::uint64_t generation);
    void receiveState(std::uint64_t generation, CameraState state);
    void receiveError(std::uint64_t generation, const CameraError& error);

    CameraDevice device_;
    std::weak_ptr<ICameraEvents> sink_;
    QThread worker_;
    Backend* backend_{}; // Worker thread deletes it when that thread finishes.
    CameraState state_{CameraState::Stopped};
    std::uint64_t generation_{};
    bool startRequested_{};
    bool stopRequested_{};
};

class QtCameraSourceFactory final : public ICameraSourceFactory {
public:
    [[nodiscard]] bool supports(const CameraDevice& device) const override;
    [[nodiscard]] std::unique_ptr<ICameraSource> create(const CameraDevice& device) override;
};

} // namespace micro_robot
