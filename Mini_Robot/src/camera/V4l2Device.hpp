#pragma once

#include <string>

namespace micro_robot {

// Result of opening a V4L2 device node and querying its capabilities. Internal to
// the V4L2 adapter target: discovery and capture classify nodes the same way.
struct V4l2NodeProbe {
    enum class Status {
        Capture,      // Opened; offers single- or multi-planar video capture.
        NotCapture,   // Opened, but not a video capture node (e.g. UVC metadata).
        Missing,      // Node absent or its device has gone.
        AccessDenied, // The process may not open the node for capture.
        Busy,         // The driver refused to open the node now.
        Failed,       // Any other open or capability-query error.
    };
    Status status{Status::Failed};
    std::string card;   // Driver-reported device name when the query succeeded.
    std::string detail; // OS error text when the node could not be queried.
};

// Opens the node read/write and non-blocking, as capture does, queries
// VIDIOC_QUERYCAP, and closes it again. Starts no stream. Blocking is limited to
// the driver's open/close (a USB camera may resume from autosuspend).
[[nodiscard]] V4l2NodeProbe probeV4l2Node(const std::string& path);

} // namespace micro_robot
