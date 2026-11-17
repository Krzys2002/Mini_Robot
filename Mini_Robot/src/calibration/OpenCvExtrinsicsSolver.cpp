#include "calibration/OpenCvExtrinsicsSolver.hpp"
#include "calibration/WorkspaceProfile.hpp"
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/geometry/3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif
#include <algorithm>
#include <cmath>

namespace micro_robot {
ExtrinsicSolveOutcome OpenCvExtrinsicsSolver::solve(const WorkspaceCalibrationProfile& profile, const ExtrinsicSolveInput& input) {
    if (auto error=validateWorkspaceCalibrationProfile(profile); !error.empty()) return {{},error};
    if (!profile.homography) return {{},"Calculate/load a tabletop profile before estimating extrinsics."};
    if (auto error=validateExtrinsicSurvey(input.survey,profile.intrinsics); !error.empty()) return {{},error};
    if (auto error=validateRobotMounts(input.mounts); !error.empty()) return {{},error};
    if (input.revision.empty() || input.revision.size()>MaximumGeometryNoteBytes) return {{},"A new calibration revision is required."};
    try {
        std::vector<cv::Point3d> object;
        std::vector<cv::Point2d> image;
        for (const auto& p:input.survey.references) if (!p.check) {
            // Solve on canonical z=0, then translate the measured surface back to world z.
            object.emplace_back(p.world.x.value,p.world.y.value,0);
            image.emplace_back(p.image.u,p.image.v);
        }
        const auto& k=profile.intrinsics.cameraMatrix;
        cv::Mat camera(3,3,CV_64F,const_cast<double*>(k.data()));
        std::vector<cv::Mat> rotations,translations;
        // Native pixels were already undistorted; never apply distortion twice.
        cv::solvePnPGeneric(object,image,camera,cv::noArray(),rotations,translations,false,cv::SOLVEPNP_IPPE);
        struct Candidate { ElevatedLocalization value; double fitRms; };
        std::vector<Candidate> candidates;
        for (std::size_t i=0;i<rotations.size();++i) {
            ElevatedLocalization e;
            e.revision=input.revision; e.tabletopBinding=tabletopProfileBinding(profile);
            e.survey=input.survey; e.mounts=input.mounts;
            cv::Mat rotation; cv::Rodrigues(rotations[i],rotation);
            for (std::size_t j=0;j<e.worldToCamera.size();++j) e.worldToCamera[j]=rotation.ptr<double>()[j];
            for (std::size_t j=0;j<e.translationMeters.size();++j) e.translationMeters[j]=translations[i].ptr<double>()[j];
            for (std::size_t j=0;j<3;++j) e.translationMeters[j]-=e.worldToCamera[j*3+2]*input.survey.surfaceHeightMeters;
            const auto centre=cameraWorldPosition(e);
            if (!centre || (*centre)[2]<=input.survey.surfaceHeightMeters+RayTolerance) continue;
            double squares=0; bool supported=true;
            for (const auto& p:input.survey.references) {
                const auto projected=projectReference(e,profile.intrinsics,p.world,input.survey.surfaceHeightMeters);
                if (!projected || !intersectTagPlane(e,profile.intrinsics,p.image,input.survey.surfaceHeightMeters)) { supported=false; break; }
                if (!p.check) squares+=std::pow(projected->u-p.image.u,2)+std::pow(projected->v-p.image.v,2);
            }
            const double rms=std::sqrt(squares/static_cast<double>(object.size()));
            if (supported && std::isfinite(rms)) candidates.push_back({std::move(e),rms});
        }
        if (candidates.empty()) return {{},"No above-table camera solution with positive-depth reference rays. Check axes/point labels and surface height."};
        std::sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b){return a.fitRms<b.fitRms;});
        auto result=std::move(candidates.front().value);
        if (candidates.size()>1) result.alternativeGapPixels=candidates[1].fitRms-candidates[0].fitRms;
        if (auto error=validateElevatedLocalization(result,profile); !error.empty()) return {{},error};
        return {std::move(result),{}};
    } catch (const std::exception& e) { return {{},e.what()}; }
}
} // namespace micro_robot
