#pragma once

#include "camera/ICameraSourceFactory.hpp"
#include "simulation/FakeCameraSource.hpp"

#include <cstddef>
#include <memory>
#include <vector>

namespace micro_robot {

// Stand-in capture backend: each created source is a FakeCameraSource whose info
// names the selected device and marks the feed as simulated. No device is opened.
// Thread-affine. Created sources must be destroyed before the factory.
class FakeCameraSourceFactory final : public ICameraSourceFactory {
public:
    // config.info is replaced per device; other settings apply to every source.
    // Throws std::invalid_argument for a non-positive frame interval.
    explicit FakeCameraSourceFactory(FakeCameraConfig config = {});
    FakeCameraSourceFactory(const FakeCameraSourceFactory&) = delete;
    FakeCameraSourceFactory& operator=(const FakeCameraSourceFactory&) = delete;
    FakeCameraSourceFactory(FakeCameraSourceFactory&&) = delete;
    FakeCameraSourceFactory& operator=(FakeCameraSourceFactory&&) = delete;

    // Any device with an ID; the fake needs no backend capability.
    [[nodiscard]] bool supports(const CameraDevice& device) const override;
    [[nodiscard]] std::unique_ptr<ICameraSource> create(const CameraDevice& device) override;

    // Advances every live created source, in creation order, with the
    // FakeCameraSource::advanceTo rules. Composition drives virtual time here.
    void advanceTo(Timestamp now);
    [[nodiscard]] std::size_t liveSourceCount() const noexcept { return live_.size(); }

private:
    class Source;
    FakeCameraConfig config_;
    std::vector<Source*> live_; // Non-owning; each Source unregisters on destruction.
};

} // namespace micro_robot
