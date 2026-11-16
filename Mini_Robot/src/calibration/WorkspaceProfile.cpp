#include "calibration/WorkspaceProfile.hpp"
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace micro_robot {
std::string serializeWorkspaceProfile(const WorkspaceCalibrationProfile& p)
{
    if (const auto error = validateWorkspaceCalibrationProfile(p); !error.empty())
        throw std::invalid_argument(error);
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<double>::max_digits10)
        << "micro_robot_workspace 3\nworkspace " << std::quoted(p.workspaceId) << "\ndimensions "
        << p.bounds.width().value << ' ' << p.bounds.height().value
        << "\nminimum " << p.bounds.minimum().x.value << ' ' << p.bounds.minimum().y.value
        << "\nframe " << std::quoted(p.frameId) << "\nboundary " << (p.boundary ? 1 : 0);
    if (p.boundary)
        for (const auto& corner : *p.boundary) out << ' ' << corner.u << ' ' << corner.v;
    out << "\nacquisition " << std::quoted(p.acquisitionId);
    out << "\nimage_coordinates undistorted_native_k\nhomography " << (p.homography ? 1 : 0);
    if (p.homography)
        for (double value : p.homography->imageToWorld)
            out << ' ' << value;
    out << "\nintrinsics " << std::quoted(serializeCalibrationProfile(p.intrinsics)) << '\n';
    auto text = out.str();
    if (text.size() > MaximumWorkspaceProfileBytes)
        throw std::invalid_argument("Workspace profile is too large.");
    return text;
}

WorkspaceProfileLoadOutcome parseWorkspaceProfile(std::string_view text)
{
    const auto reject = [](std::string error) {
        return WorkspaceProfileLoadOutcome{ProfileLoadState::Rejected, {}, std::move(error)};
    };
    if (text.empty() || text.size() > MaximumWorkspaceProfileBytes)
        return reject("Empty or oversized workspace profile.");
    std::istringstream in{std::string{text}};
    in.imbue(std::locale::classic());
    std::string token, version, id, convention, flag, intrinsicText;
    double width{}, height{}, minX{}, minY{};
    std::string frameId{"legacy-lower-left"}, acquisitionId;
    std::optional<WorkspaceBoundary> boundary;
    auto keyword = [&](const char* expected) { return static_cast<bool>(in >> token) && token == expected; };
    auto quoted = [&](std::string& value) {
        in >> std::ws;
        return in.peek() == '"' && static_cast<bool>(in >> std::quoted(value));
    };
    try {
        if (!keyword("micro_robot_workspace") || !(in >> version) || (version != "1" && version != "2" && version != "3") ||
            !keyword("workspace") || !quoted(id) || !keyword("dimensions") || !(in >> width >> height))
            return reject("Invalid workspace profile header, version or dimensions.");
        if (version != "1") {
            if (!keyword("minimum") || !(in >> minX >> minY) || !keyword("frame") || !quoted(frameId)
                || !keyword("boundary") || !(in >> flag) || (flag != "0" && flag != "1"))
                return reject("Invalid workspace frame descriptor.");
            if (flag == "1") {
                boundary.emplace();
                for (auto& corner : *boundary)
                    if (!(in >> corner.u >> corner.v)) return reject("Invalid workspace boundary.");
            }
        }
        if (version == "3" && (!keyword("acquisition") || !quoted(acquisitionId)))
            return reject("Missing workspace acquisition binding.");
        if (!keyword("image_coordinates") || !(in >> convention) || convention != "undistorted_native_k" ||
            !keyword("homography") || !(in >> flag) || (flag != "0" && flag != "1"))
            return reject("Invalid workspace profile header, version or coordinate convention.");
        std::optional<WorkspaceHomography> transform;
        if (flag == "1") {
            transform.emplace();
            for (auto& value : transform->imageToWorld)
                if (!(in >> value))
                    return reject("Invalid workspace homography matrix.");
        }
        if (!keyword("intrinsics") || !quoted(intrinsicText))
            return reject("Missing bound intrinsic profile.");
        in >> std::ws;
        if (!in.eof())
            return reject("Unexpected workspace profile trailing data.");
        auto optical = parseCalibrationProfile(intrinsicText);
        if (!optical.result)
            return reject("Invalid bound intrinsic profile: " + optical.error);
        WorkspaceCalibrationProfile p{WorkspaceCalibrationProfileVersion,
                                      std::move(id),
                                      WorkspaceBounds{Meters{width}, Meters{height}, {Meters{minX}, Meters{minY}}},
                                      std::move(*optical.result),
                                      WorkspaceImageCoordinates::UndistortedNativeCameraMatrix,
                                      transform, std::move(frameId), boundary, std::move(acquisitionId)};
        if (auto error = validateWorkspaceCalibrationProfile(p); !error.empty())
            return reject(error);
        return {ProfileLoadState::Loaded, std::move(p), {}};
    } catch (const std::exception& e) {
        return reject(e.what());
    }
}
} // namespace micro_robot
