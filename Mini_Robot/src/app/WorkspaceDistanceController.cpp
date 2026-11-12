#include "app/WorkspaceDistanceController.hpp"
#include "app/WorkspaceCalibrationController.hpp"
#include "app/WorkspaceDiagnosticsController.hpp"
namespace micro_robot
{
WorkspaceDistanceController::WorkspaceDistanceController(WorkspaceCalibrationController& source,
                                                         QObject* parent)
    : QObject(parent), source_(source)
{
    geometry_ = source_.diagnostics().snapshot();
    shownCanFreeze_ = canFreeze();
    connect(&source_.diagnostics(), &WorkspaceDiagnosticsController::calibrationChanged, this,
            &WorkspaceDistanceController::calibrationChanged);
    connect(&source_.diagnostics(), &WorkspaceDiagnosticsController::changed, this, [this] {
        if (const auto& snapshot = source_.diagnostics().snapshot())
            geometry_ = snapshot;
        recompute();
    });
    connect(&source_, &WorkspaceCalibrationController::measurementFrameChanged, this, [this] { clear(); });
    connect(&source_, &WorkspaceCalibrationController::changed, this, [this] {
        const bool enabled = canFreeze();
        if (pendingFreeze_ && !source_.busy()) {
            pendingFreeze_ = false;
            message_ = source_.message();
            shownCanFreeze_ = enabled;
            emit changed();
        } else if (enabled != shownCanFreeze_) {
            shownCanFreeze_ = enabled;
            emit changed();
        }
    });
}
FramePtr WorkspaceDistanceController::frame() const { return source_.measurementFrame(); }
bool WorkspaceDistanceController::canFreeze() const { return source_.canFreezeMeasurement(); }
void WorkspaceDistanceController::calibrationChanged()
{
    geometry_.reset();
    endpoints_ = {};
    measurement_ = {};
    message_ = "Active calibration changed; endpoints and uncertainty cleared.";
    emit changed();
}
void WorkspaceDistanceController::recompute()
{
    measurement_ = {};
    if (geometry_ && endpoints_[0] && endpoints_[1]) {
        measurement_ = measureWorkspaceDistance(*geometry_, {*endpoints_[0], *endpoints_[1]});
        if (measurement_.result && !source_.diagnostics().snapshot()) {
            auto& result = *measurement_.result;
            result.plusMinusMeters.reset();
            result.lowerMeters.reset();
            result.upperMeters.reset();
            result.calibrationRadiiMeters = {};
            result.uncertaintyUnavailable =
                "Current calibration diagnostics are recomputing; previous uncertainty is not reused.";
        }
    }
    emit changed();
}
bool WorkspaceDistanceController::pickImage(std::size_t index, ImageCorner p)
{
    if (index >= endpoints_.size() || !geometry_ || !frame()) {
        message_ =
            "Freeze a compatible undistorted frame and wait for calibration diagnostics before picking.";
        emit changed();
        return false;
    }
    const auto q = workspaceDistancePointFromImage(*geometry_, p);
    if (!q) {
        message_ = "Rejected image pick: outside supported geometry or invalid mapping. Endpoint unchanged.";
        emit changed();
        return false;
    }
    endpoints_[index] =
        WorkspaceDistanceEndpoint{*q, {WorkspaceSelectionSpace::NativeUndistortedPixels, {}, {}}};
    message_ = "Endpoint selected. Declare its maximum selection disk if justified; unknown means "
               "uncertainty unavailable.";
    recompute();
    return true;
}
bool WorkspaceDistanceController::setWorld(std::size_t index, WorldPoint2D q)
{
    if (index >= endpoints_.size() || !geometry_ || !workspaceDistanceDiskSupported(*geometry_, q, 0)) {
        message_ = "Rejected mapped-world coordinate: outside supported geometry or diagnostics unavailable. "
                   "Endpoint unchanged.";
        emit changed();
        return false;
    }
    endpoints_[index] = WorkspaceDistanceEndpoint{q, {WorkspaceSelectionSpace::WorldMeters, {}, {}}};
    message_ = "Mapped-world endpoint edited; previous selection bound cleared.";
    recompute();
    return true;
}
void WorkspaceDistanceController::setSelection(std::size_t index, WorkspaceEndpointSelection selection)
{
    if (index >= endpoints_.size() || !endpoints_[index])
        return;
    if (selection.justification.size() > MaximumWorkspaceSelectionMethodBytes) {
        message_ = "Selection method exceeds bounded capacity; prior selection unchanged.";
        emit changed();
        return;
    }
    endpoints_[index]->selection = std::move(selection);
    message_ =
        "Endpoint selection bound updated. Coverage remains conditional on calibration-model assumptions.";
    recompute();
}
void WorkspaceDistanceController::clear(std::optional<std::size_t> index)
{
    if (index) {
        if (*index < endpoints_.size())
            endpoints_[*index].reset();
    } else
        endpoints_ = {};
    measurement_ = {};
    message_ = "Endpoints cleared. Re-freezing or replacing calibration clears old picks.";
    emit changed();
}
void WorkspaceDistanceController::freeze()
{
    if (!canFreeze()) {
        message_ = "A compatible active calibration and fresh available camera input are required.";
        emit changed();
        return;
    }
    pendingFreeze_ = true;
    source_.freezeMeasurement();
    message_ = source_.message();
    emit changed();
}
} // namespace micro_robot
