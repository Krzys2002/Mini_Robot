#include "vision/OpenCvReferenceMarkers.hpp"
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/geometry/3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif
#include <algorithm>
namespace micro_robot {
ReferenceFramePtr acquireReferenceFrame(const MarkerDetections& detections,const std::string& source,
    const RobotMarkerMap& map,const WorkspaceCalibrationProfile& p) {
    if(!p.references) return {};
    auto f=std::make_shared<ReferenceFrame>();
    f->cameraBinding=referenceCameraBinding(p.intrinsics); f->worldFrameId=p.frameId;
    f->registryRevision=p.references->revision; f->inputCalibrationRevision=p.localization?p.localization->revision:std::string{};
    f->frameSequence=detections.frameSequence; f->inputGeneration=detections.inputGeneration; f->capturedAt=detections.capturedAt;
    f->error=validateReferenceConfiguration(*p.references,p,map.profile());
    if(source!=p.intrinsics.sourceId || detections.width!=p.intrinsics.width || detections.height!=p.intrinsics.height)
        f->error="Reference camera/resolution mismatch; native pixels cannot be rescaled.";
    if(!detections.error.empty()) f->error=detections.error;
    if(!f->error.empty()) return f;
    try {
        const cv::Mat k(3,3,CV_64F,const_cast<double*>(p.intrinsics.cameraMatrix.data()));
        const cv::Mat d(1,5,CV_64F,const_cast<double*>(p.intrinsics.distortion.data()));
        for(const auto& marker:p.references->markers) {
            ReferenceObservation row; row.id=marker.id;
            const MarkerObservation* found=nullptr; std::size_t matches=0;
            for(const auto& observation:detections.markers) if(observation.id==marker.id && observation.family==marker.family) { found=&observation; ++matches; }
            if(found) row.raw=found->corners;
            if(matches>1) row.rejection=ReferenceRejection::Duplicate;
            else if(found) {
                row.rejection=ReferenceRejection::InvalidCorners;
                const auto geometry=markerGeometry(row.raw);
                if(geometry && geometry->minimumEdgePixels>=MinimumReferenceEdgePixels) {
                    std::vector<cv::Point2d> input,output;
                    for(auto pixel:row.raw) input.emplace_back(pixel.u,pixel.v);
                    cv::undistortPoints(input,output,k,d,cv::noArray(),k,
                        cv::TermCriteria{cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS,ReferenceUndistortMaximumIterations,ReferenceUndistortConvergence});
                    if(output.size()==row.undistorted.size()) {
                        bool inside=true;
                        for(std::size_t i=0;i<output.size();++i) {
                            row.undistorted[i]={output[i].x,output[i].y};
                            inside=inside && output[i].x>=0 && output[i].x<=p.intrinsics.width-1 && output[i].y>=0 && output[i].y<=p.intrinsics.height-1;
                        }
                        row.rejection=inside && markerGeometry(row.undistorted) && markerGeometry(row.undistorted)->minimumEdgePixels>=MinimumReferenceEdgePixels?ReferenceRejection::Included:ReferenceRejection::UnsupportedRay;
                    }
                }
            }
            f->observations.push_back(row);
        }
    } catch(const std::exception& e) { f->error=e.what(); }
    return f;
}
} // namespace micro_robot
