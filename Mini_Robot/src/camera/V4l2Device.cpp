#include "camera/V4l2Device.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <system_error>

namespace micro_robot {

V4l2NodeProbe probeV4l2Node(const std::string& path)
{
    using Status = V4l2NodeProbe::Status;
    const auto describe = [](int error) { return std::generic_category().message(error); };

    int descriptor{};
    do {
        descriptor = ::open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    } while (descriptor < 0 && errno == EINTR);
    if (descriptor < 0) {
        const int error = errno;
        switch (error) {
        case ENOENT: case ENOTDIR: case ENODEV: case ENXIO:
            return {Status::Missing, {}, describe(error)};
        case EACCES: case EPERM:
            return {Status::AccessDenied, {}, describe(error)};
        case EBUSY:
            return {Status::Busy, {}, describe(error)};
        default:
            return {Status::Failed, {}, describe(error)};
        }
    }

    v4l2_capability capability{};
    int result{};
    do {
        result = ::ioctl(descriptor, VIDIOC_QUERYCAP, &capability);
    } while (result < 0 && errno == EINTR);
    const int queryError = result < 0 ? errno : 0;
    ::close(descriptor);
    if (result < 0) {
        // ENOTTY: an ordinary file or a non-V4L2 character device.
        return {queryError == ENODEV ? Status::Missing : Status::NotCapture, {}, describe(queryError)};
    }

    // device_caps describes this node; capabilities covers the whole physical device.
    const auto caps = (capability.capabilities & V4L2_CAP_DEVICE_CAPS) != 0
        ? capability.device_caps : capability.capabilities;
    const auto* card = reinterpret_cast<const char*>(capability.card);
    std::string name{card, ::strnlen(card, sizeof(capability.card))};
    const bool capture = (caps & (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE)) != 0;
    return {capture ? Status::Capture : Status::NotCapture, std::move(name), {}};
}

} // namespace micro_robot
