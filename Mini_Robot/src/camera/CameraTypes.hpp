#pragma once

#include "common/Time.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace micro_robot {

// Shared live-stream liveness/presentation bound. Qt adapters use precise timers.
inline constexpr auto CameraFrameTimeout = std::chrono::seconds{3};

struct CameraInfo {
    // Stable for the source object's lifetime; identifies the device/replay source,
    // not a frame or native handle. Consumers obtain it from ICameraSource::info().
    std::string sourceId;
    std::string displayName;
};

// Capture size in pixels requested from a live device.
struct CameraResolution {
    std::uint32_t width{};
    std::uint32_t height{};
    friend bool operator==(const CameraResolution&, const CameraResolution&) = default;
};

// Capture parameters a factory applies to every live source it creates. Backends
// negotiate with the device: an unsupported request falls back to a mode the
// device offers, so Frame width/height always report the delivered size.
struct CameraCaptureSettings {
    std::optional<CameraResolution> resolution; // Empty: the device's default mode.
};

enum class CameraState { Stopped, Starting, Streaming, Stopping, Disconnected, Fault };
// Unsigned 8-bit channels in the named order; Rgba8 uses straight (not
// premultiplied) alpha. Image origin is top-left, +u right, +v down.
enum class PixelFormat { Gray8, Rgb8, Rgba8 };

struct Frame {
    std::uint64_t sequence{}; // Increasing per source lifetime; gaps are allowed.
    // Local monotonic capture time (virtual time for deterministic sources).
    // Replay/media/device timestamps must be mapped into this clock domain.
    Timestamp capturedAt{};
    std::uint32_t width{};
    std::uint32_t height{};
    std::size_t strideBytes{};
    PixelFormat pixelFormat{PixelFormat::Rgb8};
    // Packed rows, top to bottom. Storage includes any stride padding.
    // Producers must not mutate or reuse the Frame or its pixel storage after
    // publication, including through mutable aliases. Consumers may retain both
    // independently of the source. No borrowed device/image-library buffers.
    std::shared_ptr<const std::vector<std::byte>> pixels;
};

using FramePtr = std::shared_ptr<const Frame>;

// Checks packed-image metadata/storage only, without allocation or throwing.
// Timestamp/sequence ordering is the source's responsibility. Require storage
// for every full stride, including final-row padding. Division avoids overflow
// when metadata comes from a malformed backend or a file.
[[nodiscard]] inline bool isValidFrame(const Frame& frame) noexcept
{
    std::size_t channels{};
    switch (frame.pixelFormat) {
    case PixelFormat::Gray8: channels = 1; break;
    case PixelFormat::Rgb8: channels = 3; break;
    case PixelFormat::Rgba8: channels = 4; break;
    default: return false;
    }
    return frame.width > 0 && frame.height > 0 && frame.pixels
        && frame.strideBytes / channels >= frame.width
        && frame.pixels->size() / frame.height >= frame.strideBytes;
}

enum class CameraErrorCode { Unavailable, PermissionDenied, InvalidFrame, BackendFailure };

struct CameraError {
    CameraErrorCode code;
    std::string message; // Diagnostic text, not a protocol error or native handle.
    Timestamp occurredAt;
};

} // namespace micro_robot
