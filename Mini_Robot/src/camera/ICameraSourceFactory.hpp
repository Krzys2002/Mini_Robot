#pragma once

#include "camera/ICameraDeviceDiscovery.hpp"
#include "camera/ICameraSource.hpp"

#include <memory>

namespace micro_robot {

// Creates capture sources for discovered devices. Composition chooses the concrete
// backend; consumers select devices without knowing it. Thread-affine: use on the
// thread that will own the created sources.
class ICameraSourceFactory {
public:
    virtual ~ICameraSourceFactory() = default;
    // Pure query; opens no device and requests no permission. A supported device
    // may still fail later, which the created source reports asynchronously.
    [[nodiscard]] virtual bool supports(const CameraDevice& device) const = 0;
    // Returns a new Stopped source with no event sink, or nullptr when the device is
    // unsupported or cannot be represented. The caller owns it and follows the
    // ICameraSource destruction rules. Creation must not start capture.
    [[nodiscard]] virtual std::unique_ptr<ICameraSource> create(const CameraDevice& device) = 0;
};

} // namespace micro_robot
