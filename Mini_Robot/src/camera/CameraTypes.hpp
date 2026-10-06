#pragma once

#include "common/Time.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace micro_robot {

struct CameraInfo {
    std::string sourceId; // Opaque source identity, independent of Qt/device handles.
    std::string displayName;
};

enum class CameraState { Stopped, Starting, Streaming, Stopping, Disconnected, Fault };
enum class PixelFormat { Gray8, Rgb8, Rgba8 };

struct Frame {
    std::uint64_t sequence{};
    Timestamp capturedAt{}; // Local monotonic capture time.
    std::uint32_t width{};
    std::uint32_t height{};
    std::size_t strideBytes{};
    PixelFormat pixelFormat{PixelFormat::Rgb8};
    // Packed rows, top to bottom. Storage includes any stride padding.
    // Producers must not mutate or reuse storage while a consumer retains it.
    std::shared_ptr<const std::vector<std::byte>> pixels;
};

using FramePtr = std::shared_ptr<const Frame>;

enum class CameraErrorCode { Unavailable, PermissionDenied, InvalidFrame, BackendFailure };

struct CameraError {
    CameraErrorCode code;
    std::string message; // Diagnostic text, not a protocol error or native handle.
    Timestamp occurredAt;
};

} // namespace micro_robot
