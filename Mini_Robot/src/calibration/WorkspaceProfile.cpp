#include "calibration/WorkspaceProfile.hpp"
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
namespace micro_robot {
std::string serializeWorkspaceProfile(const WorkspaceCalibrationProfile& p)
{
    if (const auto error = validateWorkspaceCalibrationProfile(p); !error.empty()) throw std::invalid_argument(error);
    std::ostringstream out; out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<double>::max_digits10)
        << "micro_robot_workspace 1\nworkspace " << std::quoted(p.workspaceId)
        << "\ndimensions " << p.bounds.width().value << ' ' << p.bounds.height().value
        << "\nimage_coordinates undistorted_native_k\nhomography " << (p.homography ? 1 : 0);
    if (p.homography) for (double value : p.homography->imageToWorld) out << ' ' << value;
    out << "\nintrinsics " << std::quoted(serializeCalibrationProfile(p.intrinsics)) << '\n';
    auto text = out.str();
    if (text.size() > MaximumWorkspaceProfileBytes) throw std::invalid_argument("Workspace profile is too large.");
    return text;
}
WorkspaceProfileLoadOutcome parseWorkspaceProfile(std::string_view text)
{
    const auto reject = [](std::string error) { return WorkspaceProfileLoadOutcome{ProfileLoadState::Rejected, {}, std::move(error)}; };
    if (text.empty() || text.size() > MaximumWorkspaceProfileBytes) return reject("Empty or oversized workspace profile.");
    std::istringstream in{std::string{text}}; in.imbue(std::locale::classic());
    std::string token, version, id, convention, flag, intrinsicText;
    double width{}, height{};
    auto keyword = [&](const char* expected) { return static_cast<bool>(in >> token) && token == expected; };
    auto quoted = [&](std::string& value) {
        in >> std::ws;
        return in.peek() == '"' && static_cast<bool>(in >> std::quoted(value));
    };
    try {
        if (!keyword("micro_robot_workspace") || !(in >> version) || version != "1"
            || !keyword("workspace") || !quoted(id) || !keyword("dimensions") || !(in >> width >> height)
            || !keyword("image_coordinates") || !(in >> convention) || convention != "undistorted_native_k"
            || !keyword("homography") || !(in >> flag) || (flag != "0" && flag != "1"))
            return reject("Invalid workspace profile header, version or coordinate convention.");
        std::optional<WorkspaceHomography> transform;
        if (flag == "1") {
            transform.emplace();
            for (auto& value : transform->imageToWorld) if (!(in >> value)) return reject("Invalid workspace homography matrix.");
        }
        if (!keyword("intrinsics") || !quoted(intrinsicText)) return reject("Missing bound intrinsic profile.");
        in >> std::ws;
        if (!in.eof()) return reject("Unexpected workspace profile trailing data.");
        auto optical = parseCalibrationProfile(intrinsicText);
        if (!optical.result) return reject("Invalid bound intrinsic profile: " + optical.error);
        WorkspaceCalibrationProfile p{WorkspaceCalibrationProfileVersion, std::move(id),
            WorkspaceBounds{Meters{width}, Meters{height}}, std::move(*optical.result),
            WorkspaceImageCoordinates::UndistortedNativeCameraMatrix, transform};
        if (auto error = validateWorkspaceCalibrationProfile(p); !error.empty()) return reject(error);
        return {ProfileLoadState::Loaded, std::move(p), {}};
    } catch (const std::exception& e) { return reject(e.what()); }
}
}
