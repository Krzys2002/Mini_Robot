#include "simulation/FakeCameraSourceFactory.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

namespace micro_robot {

// Forwards the camera contract and registers itself for factory-driven time.
class FakeCameraSourceFactory::Source final : public ICameraSource {
public:
    Source(FakeCameraSourceFactory& owner, FakeCameraConfig config)
        : owner_(owner), camera_(std::move(config))
    {
        owner_.live_.push_back(this);
    }
    ~Source() override { std::erase(owner_.live_, this); }
    Source(const Source&) = delete;
    Source& operator=(const Source&) = delete;
    Source(Source&&) = delete;
    Source& operator=(Source&&) = delete;

    void setEventSink(std::weak_ptr<ICameraEvents> sink) override { camera_.setEventSink(std::move(sink)); }
    CameraInfo info() const override { return camera_.info(); }
    CameraState state() const override { return camera_.state(); }
    void start() override { camera_.start(); }
    void stop() override { camera_.stop(); }
    void advanceTo(Timestamp now) { camera_.advanceTo(now); }

private:
    FakeCameraSourceFactory& owner_;
    FakeCameraSource camera_;
};

FakeCameraSourceFactory::FakeCameraSourceFactory(FakeCameraConfig config)
    : config_(std::move(config))
{
    // Fail at composition time rather than on the first selection.
    if (config_.frameInterval <= Duration::zero()) {
        throw std::invalid_argument("Fake camera frame interval must be positive");
    }
}

bool FakeCameraSourceFactory::supports(const CameraDevice& device) const { return !device.id.empty(); }

std::unique_ptr<ICameraSource> FakeCameraSourceFactory::create(const CameraDevice& device)
{
    if (!supports(device)) return nullptr;
    auto config = config_;
    const auto name = device.displayName.empty() ? std::string{"Unnamed camera"} : device.displayName;
    config.info = {"simulated:" + device.id, name + " (simulated feed)"};
    return std::make_unique<Source>(*this, std::move(config));
}

void FakeCameraSourceFactory::advanceTo(Timestamp now)
{
    // Iterate a copy and skip sources released meanwhile, so a misbehaving
    // callback that destroys a source cannot invalidate this loop.
    const auto sources = live_;
    for (auto* source : sources) {
        if (std::find(live_.begin(), live_.end(), source) != live_.end()) source->advanceTo(now);
    }
}

} // namespace micro_robot
