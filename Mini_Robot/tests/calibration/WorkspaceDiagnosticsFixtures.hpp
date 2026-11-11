#pragma once
#include "HomographyFixtures.hpp"
#include "calibration/WorkspaceDiagnostics.hpp"
#include <random>
namespace micro_robot::test
{
inline WorkspaceCalibrationProfile diagnosticsProfile()
{
    auto p = homographyInput().profile;
    p.homography = knownHomography();
    return p;
}
inline std::shared_ptr<WorkspaceAcquisitionRecord> diagnosticsAcquisition(double noise = 0)
{
    auto a = std::make_shared<WorkspaceAcquisitionRecord>(
        WorkspaceAcquisitionRecord{"survey", diagnosticsProfile(), {}, {}});
    a->profile.acquisitionId = a->id;
    std::uint64_t id = 0;
    for (auto c : homographyInput().fitPoints)
        a->references.push_back({++id, c, false, false, {}});
    std::mt19937 random(249);
    std::normal_distribution<double> gaussian(0, noise);
    for (int y = 0; y < 3; ++y)
        for (int x = 0; x < 3; ++x) {
            WorldPoint2D world{Meters{0.2 + 0.3 * x}, Meters{0.1 + 0.2 * y}};
            auto pixel = *unprojectWorkspacePoint(*a->profile.homography, world);
            // Deliberate affine systematic error plus independent deterministic pseudo-random survey noise.
            pixel.u += (0.002 + 0.001 * world.x.value + gaussian(random)) / 0.00125;
            pixel.v -= (0.001 - 0.001 * world.y.value + gaussian(random)) / 0.001;
            a->references.push_back({++id, {pixel, world}, true, false, {}});
        }
    return a;
}
inline WorkspaceSurveyEvidence diagnosticsEvidence(const WorkspaceAcquisitionRecord& a)
{
    WorkspaceSurveyEvidence e;
    for (const auto& r : a.references)
        if (r.check)
            e.referenceIds.push_back(r.id);
    e.independentSurveyConfirmed = true;
    e.gaussianAffineModelConfirmed = true;
    e.additionalStandardUncertaintyMeters = 0.0005;
    e.method = "Synthetic independently surveyed Checks; Gaussian affine test model, 0.5 mm per-axis "
               "additional floor.";
    return e;
}
} // namespace micro_robot::test
