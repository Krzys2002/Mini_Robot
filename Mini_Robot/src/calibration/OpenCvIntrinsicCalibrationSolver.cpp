#include "calibration/OpenCvIntrinsicCalibrationSolver.hpp"

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/calib.hpp>
#include <opencv2/geometry/3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif

#include <cmath>
#include <set>

namespace micro_robot {
IntrinsicCalibrationOutcome OpenCvIntrinsicCalibrationSolver::solve(const IntrinsicCalibrationInput& input,
                                                                   const IntrinsicCalibrationConfig& config)
{
    const auto fail = [](std::string error) { return IntrinsicCalibrationOutcome{{}, std::move(error)}; };
    if (const auto error = validateIntrinsicConfig(config); !error.empty()) return fail(error);
    if (input.sourceId.empty() || input.width == 0 || input.height == 0
        || !supportedCalibrationResolution(input.width, input.height)
        || input.observations.size() < 3 || input.observations.size() > CalibrationMaximumViews)
        return fail("Expected camera identity, supported resolution and 3–24 complete views.");
    try {
        std::vector<cv::Point3f> object;
        for (const auto& p : calibrationObjectPoints(input.target))
            object.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
        std::vector<std::vector<cv::Point2f>> images;
        std::set<std::uint64_t> sequences;
        for (const auto& view : input.observations) {
            if (view.corners.size() != object.size() || !sequences.insert(view.frameSequence).second)
                return fail("Incomplete target or duplicate frame in calibration input.");
            std::vector<cv::Point2f> corners;
            for (const auto& p : view.corners) {
                if (!std::isfinite(p.u) || !std::isfinite(p.v)
                    || p.u < 0 || p.v < 0 || p.u >= input.width || p.v >= input.height)
                    return fail("Calibration corners must be finite and inside the image.");
                corners.emplace_back(static_cast<float>(p.u), static_cast<float>(p.v));
            }
            // A collapsed/collinear grid cannot constrain the planar camera model.
            const auto a = corners[static_cast<std::size_t>(input.target.columns - 1)] - corners.front();
            const auto b = corners.back() - corners.front();
            if (std::abs(a.x * b.y - a.y * b.x) < 1.0F)
                return fail("Degenerate target geometry. Capture a visible tilted board.");
            images.push_back(std::move(corners));
        }
        std::vector<std::vector<cv::Point3f>> objects(images.size(), object);
        cv::Mat k = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat distortion = cv::Mat::zeros(5, 1, CV_64F);
        std::vector<cv::Mat> rotations, translations;
        (void)cv::calibrateCamera(objects, images,
            {static_cast<int>(input.width), static_cast<int>(input.height)}, k, distortion,
            rotations, translations, 0, {cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-9});
        IntrinsicCalibration result;
        result.sourceId = input.sourceId;
        result.width = input.width;
        result.height = input.height;
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                result.cameraMatrix[static_cast<std::size_t>(3 * row + col)] = k.at<double>(row, col);
        result.distortion.assign(distortion.ptr<double>(), distortion.ptr<double>() + distortion.total());
        double totalSquares = 0;
        std::size_t totalPoints = 0;
        for (std::size_t i = 0; i < images.size(); ++i) {
            std::vector<cv::Point2f> projected;
            cv::projectPoints(object, rotations[i], translations[i], k, distortion, projected);
            double squares = 0;
            for (std::size_t j = 0; j < projected.size(); ++j) {
                const double du = static_cast<double>(projected[j].x) - input.observations[i].corners[j].u;
                const double dv = static_cast<double>(projected[j].y) - input.observations[i].corners[j].v;
                squares += du * du + dv * dv;
            }
            result.perViewErrors.push_back({input.observations[i].frameSequence, projected.size(),
                std::sqrt(squares / static_cast<double>(projected.size()))});
            totalSquares += squares;
            totalPoints += projected.size();
        }
        result.rmsPixels = std::sqrt(totalSquares / static_cast<double>(totalPoints));
        if (const auto error = validateIntrinsicCalibration(result, config); !error.empty()) return fail(error);
        return {std::move(result), {}};
    } catch (const std::exception& error) {
        return fail(std::string{"Intrinsic calibration failed: "} + error.what());
    }
}
} // namespace micro_robot
