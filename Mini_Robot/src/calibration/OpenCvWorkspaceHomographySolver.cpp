#include "calibration/OpenCvWorkspaceHomographySolver.hpp"
#include <opencv2/core.hpp>
#include <cmath>

namespace micro_robot {
WorkspaceHomographyOutcome OpenCvWorkspaceHomographySolver::solve(const WorkspaceHomographyInput& input)
{
    if (const auto error = validateWorkspaceHomographyInput(input); !error.empty()) return {{}, error};
    try {
        const auto& k = input.profile.intrinsics;
        const double width = input.profile.bounds.width().value, height = input.profile.bounds.height().value;
        cv::Mat a = cv::Mat::zeros(static_cast<int>(input.fitPoints.size() * 2), 9, CV_64F);
        for (std::size_t i = 0; i < input.fitPoints.size(); ++i) {
            const auto& p = input.fitPoints[i];
            const double u = p.image.u / k.width, v = p.image.v / k.height;
            const double x = p.world.x.value / width, y = p.world.y.value / height;
            auto* r = a.ptr<double>(static_cast<int>(i * 2));
            r[0] = -u; r[1] = -v; r[2] = -1; r[6] = x * u; r[7] = x * v; r[8] = x;
            r = a.ptr<double>(static_cast<int>(i * 2 + 1));
            r[3] = -u; r[4] = -v; r[5] = -1; r[6] = y * u; r[7] = y * v; r[8] = y;
        }
        cv::Mat w, u, vt;
        cv::SVD::compute(a, w, u, vt, cv::SVD::FULL_UV);
        if (w.at<double>(7) <= w.at<double>(0) * 1e-9) return {{}, "Correspondences are numerically degenerate."};
        WorkspaceHomography h;
        const double rows[]{width, height, 1}, columns[]{1.0 / k.width, 1.0 / k.height, 1};
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                h.imageToWorld[static_cast<std::size_t>(row * 3 + col)] = vt.at<double>(8, row * 3 + col) * rows[row] * columns[col];
        return evaluateWorkspaceHomography(input, h);
    } catch (const cv::Exception& e) { return {{}, std::string{"Homography estimation failed: "} + e.what()}; }
}
} // namespace micro_robot
