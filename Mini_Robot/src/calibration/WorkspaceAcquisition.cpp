#include "calibration/WorkspaceAcquisition.hpp"
#include "calibration/WorkspaceProfile.hpp"
#include <cmath>
#include <algorithm>
#include <charconv>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace micro_robot {
WorkspaceBoardGeneration generateWorkspaceBoard(const WorkspaceBoardPlacement& p, std::optional<WorkspaceBounds> bounds)
{
    if (p.target.columns != WorkspaceBoardTarget.columns || p.target.rows != WorkspaceBoardTarget.rows
        || !std::isfinite(p.target.squareSizeMeters) || p.target.squareSizeMeters <= 0
        || !std::isfinite(p.anchor.x.value) || !std::isfinite(p.anchor.y.value)
        || !std::isfinite(p.columnAngle.value) || (p.rowDirection != 1 && p.rowDirection != -1)
        || p.note.size() > MaximumWorkspacePlacementNoteBytes)
        return {{},"Use the shared 9x6 inner-corner target, positive measured spacing, finite anchor/angle and explicit row direction."};
    const auto object = calibrationObjectPoints(p.target);
    if (p.detected.size() != object.size() || p.anchorIndex >= object.size()
        || p.included.size() != object.size()) return {{},"A complete exact-frame detection and review mask are required."};
    std::set<std::pair<double,double>> pixels;
    for (const auto image:p.detected)
        if (!std::isfinite(image.u) || !std::isfinite(image.v) || !pixels.insert({image.u,image.v}).second)
            return {{},"Redetect the complete board: original pixels are non-finite or duplicated."};
    WorkspaceBoundary perimeter{{p.detected.front(),p.detected[static_cast<std::size_t>(p.target.columns-1)],
        p.detected.back(),p.detected[p.detected.size()-static_cast<std::size_t>(p.target.columns)]}};
    double minU=perimeter[0].u,maxU=minU,minV=perimeter[0].v,maxV=minV;
    for (const auto image:perimeter) {minU=std::min(minU,image.u); maxU=std::max(maxU,image.u); minV=std::min(minV,image.v); maxV=std::max(maxV,image.v);}
    const double width=maxU-minU,height=maxV-minV;
    if (!std::isfinite(width) || !std::isfinite(height) || width<=0 || height<=0)
        return {{},"Redetect a non-degenerate two-dimensional board."};
    for (auto& image:perimeter) image={(image.u-minU)/width,(image.v-minV)/height};
    // Check only topology here, not a physical image-coverage/accuracy threshold.
    if (!validateWorkspaceBoundary(perimeter,MinimumWorkspaceImageDimensionPixels,MinimumWorkspaceImageDimensionPixels).empty())
        return {{},"Redetect the board: its ordered outer inner-corners are crossed/collinear/degenerate."};
    const double cosine = std::cos(p.columnAngle.value), sine = std::sin(p.columnAngle.value);
    WorkspaceBoardGeneration result;
    for (std::size_t i = 0; i < object.size(); ++i) {
        const int row = static_cast<int>(i) / p.target.columns, column = static_cast<int>(i) % p.target.columns;
        const int detectedRow = p.reverseRows ? p.target.rows-1-row : row;
        const int detectedColumn = p.reverseColumns ? p.target.columns-1-column : column;
        const auto image = p.detected[static_cast<std::size_t>(detectedRow*p.target.columns+detectedColumn)];
        if (!std::isfinite(image.u) || !std::isfinite(image.v)) return {{},"Detection contains non-finite pixels."};
        const double dx = object[i].x-object[p.anchorIndex].x;
        const double dy = (object[i].y-object[p.anchorIndex].y)*p.rowDirection;
        WorldPoint2D world{Meters{p.anchor.x.value+cosine*dx-sine*dy},
                                 Meters{p.anchor.y.value+sine*dx+cosine*dy}};
        if (!std::isfinite(world.x.value) || !std::isfinite(world.y.value)) return {{},"Board coordinates overflow."};
        if (bounds) {
            auto snap = [](double value, double lo, double hi) {
                const double tolerance = WorkspaceBoardRoundoffUlps*std::numeric_limits<double>::epsilon()
                    * std::max({std::abs(lo),std::abs(hi),hi-lo});
                if (value < lo && lo-value <= tolerance) return lo;
                if (value > hi && value-hi <= tolerance) return hi;
                return value; // Never clamp a physically outside reference.
            };
            world.x = Meters{snap(world.x.value,bounds->minimum().x.value,bounds->maximum().x.value)};
            world.y = Meters{snap(world.y.value,bounds->minimum().y.value,bounds->maximum().y.value)};
        }
        result.points.push_back({image,world});
    }
    return result;
}
std::string validateWorkspaceAcquisition(const WorkspaceAcquisitionRecord& record)
{
    if (record.id.empty() || record.id.size() > MaximumWorkspaceAcquisitionIdBytes
        || record.id != record.profile.acquisitionId) return "Acquisition identity/profile binding mismatch.";
    if (record.references.size() > MaximumWorkspaceReferences || record.placements.size() > MaximumWorkspacePlacements)
        return "Acquisition exceeds reference/placement capacity.";
    if (auto error = validateWorkspaceCalibrationProfile(record.profile); !error.empty()) return error;
    std::map<std::uint64_t,const WorkspaceBoardPlacement*> placements;
    std::set<std::uint64_t> referenceIds;
    std::set<std::pair<std::uint64_t,std::size_t>> sources;
    std::vector<WorkspaceCorrespondence> fit,check;
    for (const auto& p : record.placements) {
        if (!p.id || p.id==std::numeric_limits<std::uint64_t>::max() || !placements.emplace(p.id,&p).second) return "Duplicate/invalid placement identity.";
        auto generated = generateWorkspaceBoard(p);
        if (!generated.error.empty()) return generated.error;
        if (p.reviewRejections.size()!=p.included.size()) return "Missing original placement review results.";
        for (std::size_t i=0;i<p.included.size();++i) {
            const auto reason=p.reviewRejections[i];
            if (reason!=WorkspaceReferenceRejection::None && reason!=WorkspaceReferenceRejection::Duplicate
                && reason!=WorkspaceReferenceRejection::OutsideDomain) return "Unknown placement review result.";
            if (p.included[i] && reason!=WorkspaceReferenceRejection::None) return "Rejected placement point cannot be included.";
        }
        if (!std::isfinite(p.measuredOffset.x.value) || !std::isfinite(p.measuredOffset.y.value)
            || !std::isfinite(p.relativeReferenceSnapshot.x.value) || !std::isfinite(p.relativeReferenceSnapshot.y.value)) return "Invalid measured placement offset.";
        if (p.relativeReferenceId && (p.anchor.x.value!=p.relativeReferenceSnapshot.x.value+p.measuredOffset.x.value
            || p.anchor.y.value!=p.relativeReferenceSnapshot.y.value+p.measuredOffset.y.value)) return "Relative anchor/measurement snapshot mismatch.";
        for (const auto pixel : p.detected)
            if (pixel.u < 0 || pixel.v < 0 || pixel.u > record.profile.intrinsics.width-1
                || pixel.v > record.profile.intrinsics.height-1) return "Stored detection is outside its native image.";
    }
    std::size_t boundaryCount = 0;
    for (const auto& r : record.references) {
        if (!r.id || r.id==std::numeric_limits<std::uint64_t>::max() || !referenceIds.insert(r.id).second) return "Duplicate/invalid reference identity.";
        if (r.boundary) {
            if (r.check || r.source || !record.profile.boundary || boundaryCount >= WorkspaceCornerCount)
                return "Invalid boundary reference metadata.";
            const auto world = workspaceCorners(record.profile.bounds)[boundaryCount];
            const auto image = (*record.profile.boundary)[boundaryCount++];
            if (r.point.image.u != image.u || r.point.image.v != image.v
                || r.point.world.x.value != world.x.value || r.point.world.y.value != world.y.value)
                return "Boundary reference differs from its frame.";
        }
        if (r.source) {
            const auto p = placements.find(r.source->placementId);
            if (p == placements.end() || r.source->cornerIndex >= p->second->included.size()
                || !p->second->included[r.source->cornerIndex]
                || !sources.insert({r.source->placementId,r.source->cornerIndex}).second)
                return "Invalid/duplicate placement reference source.";
        }
        (r.check ? check : fit).push_back(r.point);
    }
    if (record.profile.boundary && boundaryCount != WorkspaceCornerCount) return "Missing boundary references.";
    for (const auto& p : record.placements) {
        for (std::size_t i = 0; i < p.included.size(); ++i)
            if (p.included[i] != sources.contains({p.id,i})) return "Placement inclusion mask/reference mismatch.";
        // Relative anchors retain their measured offset AND resolved anchor. The
        // original reference may later be corrected or removed; never reinterpret it.
        if (p.relativeReferenceId && !*p.relativeReferenceId) return "Invalid relative reference identity.";
    }
    return validateWorkspaceHomographyInput({record.profile,std::move(fit),std::move(check)});
}
std::string serializeWorkspaceAcquisition(const WorkspaceAcquisitionRecord& record)
{
    if (auto error = validateWorkspaceAcquisition(record); !error.empty()) throw std::invalid_argument(error);
    std::ostringstream out; out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<double>::max_digits10)
        << "micro_robot_workspace_acquisition 1\nid " << std::quoted(record.id)
        << "\nprofile " << std::quoted(serializeWorkspaceProfile(record.profile))
        << "\nplacements " << record.placements.size() << '\n';
    for (const auto& p : record.placements) {
        out << "placement " << p.id << ' ' << p.target.columns << ' ' << p.target.rows << ' ' << p.target.squareSizeMeters
            << ' ' << p.anchorIndex << ' ' << p.anchor.x.value << ' ' << p.anchor.y.value << ' ' << p.columnAngle.value
            << ' ' << p.rowDirection << ' ' << p.reverseColumns << ' ' << p.reverseRows << ' ' << p.relativeReferenceId.value_or(0)
            << ' ' << p.measuredOffset.x.value << ' ' << p.measuredOffset.y.value
            << ' ' << p.relativeReferenceSnapshot.x.value << ' ' << p.relativeReferenceSnapshot.y.value << ' ' << p.frameSequence
            << ' ' << p.capturedTicksNanoseconds << ' ' << std::quoted(p.note) << ' ' << p.detected.size();
        for (std::size_t i = 0; i < p.detected.size(); ++i)
            out << ' ' << p.detected[i].u << ' ' << p.detected[i].v << ' ' << p.included[i]
                << ' ' << static_cast<int>(p.reviewRejections[i]);
        out << '\n';
    }
    out << "references " << record.references.size() << '\n';
    for (const auto& r : record.references)
        out << "reference " << r.id << ' ' << r.check << ' ' << r.boundary << ' ' << r.point.image.u << ' ' << r.point.image.v
            << ' ' << r.point.world.x.value << ' ' << r.point.world.y.value << ' ' << r.source.has_value()
            << ' ' << (r.source ? r.source->placementId : 0) << ' ' << (r.source ? r.source->cornerIndex : 0) << '\n';
    auto text = out.str();
    if (text.size() > MaximumWorkspaceAcquisitionBytes) throw std::invalid_argument("Acquisition exceeds its 4 MiB byte bound.");
    return text;
}
WorkspaceAcquisitionLoadOutcome parseWorkspaceAcquisition(std::string_view text)
{
    const auto reject = [](std::string error) { return WorkspaceAcquisitionLoadOutcome{{},std::move(error)}; };
    if (text.empty() || text.size() > MaximumWorkspaceAcquisitionBytes) return reject("Empty/oversized acquisition record.");
    std::istringstream in{std::string{text}}; in.imbue(std::locale::classic());
    std::string token,version,id,profileText;
    auto key = [&](const char* expected) { return static_cast<bool>(in>>token) && token==expected; };
    auto quoted = [&](std::string& value) { in>>std::ws; return in.peek()=='"' && static_cast<bool>(in>>std::quoted(value)); };
    auto unsignedInteger = [&](auto& value) {
        std::string digits;
        if (!(in>>digits)) return false;
        const auto parsed=std::from_chars(digits.data(),digits.data()+digits.size(),value);
        return parsed.ec==std::errc{} && parsed.ptr==digits.data()+digits.size();
    };
    std::size_t count{};
    if (!key("micro_robot_workspace_acquisition") || !(in>>version) || version!="1" || !key("id") || !quoted(id)
        || !key("profile") || !quoted(profileText)) return reject("Invalid acquisition header.");
    const auto profile = parseWorkspaceProfile(profileText);
    if (!profile.profile) return reject("Invalid acquisition profile: "+profile.error);
    auto record = std::make_shared<WorkspaceAcquisitionRecord>(WorkspaceAcquisitionRecord{id,*profile.profile,{},{}});
    if (!key("placements") || !unsignedInteger(count) || count>MaximumWorkspacePlacements) return reject("Invalid placement count.");
    for (std::size_t n = 0; n < count; ++n) {
        WorkspaceBoardPlacement p; std::uint64_t referenceId{}; std::size_t corners{};
        if (!key("placement") || !unsignedInteger(p.id) || !(in>>p.target.columns>>p.target.rows>>p.target.squareSizeMeters)
            || !unsignedInteger(p.anchorIndex) || !(in>>p.anchor.x.value>>p.anchor.y.value>>p.columnAngle.value>>p.rowDirection>>p.reverseColumns>>p.reverseRows)
            || !unsignedInteger(referenceId) || !(in>>p.measuredOffset.x.value>>p.measuredOffset.y.value
            >>p.relativeReferenceSnapshot.x.value>>p.relativeReferenceSnapshot.y.value) || !unsignedInteger(p.frameSequence)
            || !(in>>p.capturedTicksNanoseconds) || !quoted(p.note) || !unsignedInteger(corners) || corners!=static_cast<std::size_t>(WorkspaceBoardTarget.columns*WorkspaceBoardTarget.rows))
            return reject("Invalid placement metadata/count.");
        if (referenceId) p.relativeReferenceId=referenceId;
        for (std::size_t i=0;i<corners;++i) {
            ImageCorner pixel; bool include{}; int rejection{};
            if (!(in>>pixel.u>>pixel.v>>include>>rejection) || rejection<0 || rejection>static_cast<int>(WorkspaceReferenceRejection::Duplicate)) return reject("Invalid placement detection/mask.");
            p.detected.push_back(pixel); p.included.push_back(include);
            p.reviewRejections.push_back(static_cast<WorkspaceReferenceRejection>(rejection));
        }
        record->placements.push_back(std::move(p));
    }
    if (!key("references") || !unsignedInteger(count) || count>MaximumWorkspaceReferences) return reject("Invalid reference count.");
    for (std::size_t n = 0; n < count; ++n) {
        WorkspaceReference r; bool source{}; WorkspaceBoardSource origin;
        if (!key("reference") || !unsignedInteger(r.id) || !(in>>r.check>>r.boundary>>r.point.image.u>>r.point.image.v
            >>r.point.world.x.value>>r.point.world.y.value>>source) || !unsignedInteger(origin.placementId) || !unsignedInteger(origin.cornerIndex)) return reject("Invalid reference metadata.");
        if (source) r.source=origin;
        else if (origin.placementId || origin.cornerIndex) return reject("Unexpected reference provenance.");
        record->references.push_back(r);
    }
    in>>std::ws;
    if (!in.eof()) return reject("Unexpected acquisition trailing data.");
    if (auto error=validateWorkspaceAcquisition(*record); !error.empty()) return reject(error);
    return {std::move(record),{}};
}
} // namespace micro_robot
