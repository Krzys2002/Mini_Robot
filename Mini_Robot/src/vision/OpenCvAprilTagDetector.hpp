#pragma once
#include "vision/MarkerDetection.hpp"

namespace micro_robot {
struct AprilTagDetectorConfig {
    std::uint64_t maximumPixels{64'000'000}; // Includes 8K; configurable resource bound.
    std::uint32_t maximumDimension{16384};
    double minimumPerimeterRate{0.005}; // Relative to max image dimension.
};
// OpenCV-only adapter. Cached tag36h11 dictionary/parameters, no Qt or camera API
// ownership. Construct on any thread, then use exclusively on one worker.
class OpenCvAprilTagDetector final : public IMarkerDetector {
public:
    explicit OpenCvAprilTagDetector(AprilTagDetectorConfig config = {});
    ~OpenCvAprilTagDetector() override;
    OpenCvAprilTagDetector(const OpenCvAprilTagDetector&) = delete;
    OpenCvAprilTagDetector& operator=(const OpenCvAprilTagDetector&) = delete;
    [[nodiscard]] MarkerDetections detect(const Frame& frame) override;
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace micro_robot
