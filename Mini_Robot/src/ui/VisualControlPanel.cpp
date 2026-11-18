#include "ui/VisualControlPanel.hpp"
#include "app/LiveVisualController.hpp"
#include "ui/VisionDebugView.hpp"
#include <QDoubleValidator>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <array>

namespace micro_robot {
VisualControlPanel::VisualControlPanel(LiveVisualController& control, QWidget* parent)
    : QWidget(parent), control_(control)
{
    auto* layout = new QVBoxLayout(this);
    auto* group = new QGroupBox(QStringLiteral("Visual goal control — selected robot"), this);
    auto* form = new QFormLayout(group);
    view_ = new VisionOverlay(group);
    view_->setMinimumHeight(220);
    previewInstruction_ = new QLabel(QStringLiteral("Calibrated view — click a target"), group);
    previewInstruction_->setObjectName(QStringLiteral("visualPreviewInstruction"));
    form->addRow(previewInstruction_, view_);
    auto* pathButtons = new QHBoxLayout;
    pathBeginButton_ = new QPushButton(QStringLiteral("Draw path"), group);
    pathBeginButton_->setObjectName(QStringLiteral("pathEditorBegin"));
    pathUndoButton_ = new QPushButton(QStringLiteral("Undo waypoint"), group);
    pathUndoButton_->setObjectName(QStringLiteral("pathEditorUndo")); pathUndoButton_->setEnabled(false);
    pathClearButton_ = new QPushButton(QStringLiteral("Clear path"), group);
    pathClearButton_->setObjectName(QStringLiteral("pathEditorClear")); pathClearButton_->setEnabled(false);
    pathFinishButton_ = new QPushButton(QStringLiteral("Finish path"), group);
    pathFinishButton_->setObjectName(QStringLiteral("pathEditorFinish")); pathFinishButton_->setEnabled(false);
    pathCancelButton_ = new QPushButton(QStringLiteral("Cancel drawing"), group);
    pathCancelButton_->setObjectName(QStringLiteral("pathEditorCancel")); pathCancelButton_->setEnabled(false);
    pathButtons->addWidget(pathBeginButton_); pathButtons->addWidget(pathUndoButton_); pathButtons->addWidget(pathClearButton_);
    pathButtons->addWidget(pathFinishButton_); pathButtons->addWidget(pathCancelButton_);
    form->addRow(pathButtons);
    const auto field = [group](const char* name) {
        auto* edit = new QLineEdit(QStringLiteral("0"), group);
        edit->setObjectName(QString::fromLatin1(name));
        auto* validator = new QDoubleValidator(edit);
        validator->setLocale(QLocale::c());
        edit->setValidator(validator);
        return edit;
    };
    auto* x = field("visualTargetX"); auto* y = field("visualTargetY"); auto* h = field("visualTargetHeading");
    form->addRow(QStringLiteral("Target X (m)"), x);
    form->addRow(QStringLiteral("Target Y (m)"), y);
    form->addRow(QStringLiteral("Heading (rad, CCW)"), h);
    auto* buttons = new QHBoxLayout;
    auto* apply = new QPushButton(QStringLiteral("Apply target"), group);
    apply->setObjectName(QStringLiteral("applyVisualTarget"));
    startButton_ = new QPushButton(QStringLiteral("Start visual control"), group);
    startButton_->setObjectName(QStringLiteral("startVisualControl")); startButton_->setEnabled(false);
    auto* pause = new QPushButton(QStringLiteral("Pause"), group);
    pause->setObjectName(QStringLiteral("pauseVisualControl")); pause->setEnabled(false);
    auto* cancel = new QPushButton(QStringLiteral("Cancel target"), group);
    cancel->setObjectName(QStringLiteral("cancelVisualControl")); cancel->setEnabled(false);
    auto* acknowledge = new QPushButton(QStringLiteral("Acknowledge controller fault"), group);
    acknowledge->setObjectName(QStringLiteral("acknowledgeVisualFault")); acknowledge->setEnabled(false);
    buttons->addWidget(apply); buttons->addWidget(startButton_); buttons->addWidget(pause);
    buttons->addWidget(cancel); buttons->addWidget(acknowledge); form->addRow(buttons);
    auto* status = new QLabel(control.description(), group);
    status->setObjectName(QStringLiteral("visualControlStatus")); status->setWordWrap(true); status->setTextFormat(Qt::PlainText);
    form->addRow(status);
    layout->addWidget(group);
    connect(apply, &QPushButton::clicked, this, [&control, x, y, h] {
        bool okX{}, okY{}, okH{};
        const double px = QLocale::c().toDouble(x->text(), &okX), py = QLocale::c().toDouble(y->text(), &okY),
            heading = QLocale::c().toDouble(h->text(), &okH);
        if (okX && okY && okH) (void)control.setTarget(px, py, heading);
    });
    connect(startButton_, &QPushButton::clicked, &control, [&control] { (void)control.start(); });
    connect(acknowledge, &QPushButton::clicked, &control, [&control] { (void)control.acknowledgeControllerFault(); });
    connect(pause, &QPushButton::clicked, &control, &LiveVisualController::pause);
    connect(cancel, &QPushButton::clicked, this, [&control, x, y] {
        control.cancel();
        x->setText(QStringLiteral("0"));
        y->setText(QStringLiteral("0"));
    });
    const auto updatePathButtons = [this] {
        invalidatePathScopeIfChanged();
        refreshPathControls();
    };
    connect(pathBeginButton_, &QPushButton::clicked, this, [this, updatePathButtons] {
        (void)beginPathEditing(); updatePathButtons();
    });
    connect(pathUndoButton_, &QPushButton::clicked, this, [this, updatePathButtons] {
        (void)undoPathPoint(); updatePathButtons();
    });
    connect(pathClearButton_, &QPushButton::clicked, this, [this, updatePathButtons] {
        clearPath(); updatePathButtons();
    });
    connect(pathFinishButton_, &QPushButton::clicked, this, [this, updatePathButtons] {
        (void)finishPathEditing(); updatePathButtons();
    });
    connect(pathCancelButton_, &QPushButton::clicked, this, [this, updatePathButtons] {
        cancelPathEditing(); updatePathButtons();
    });
    connect(&control, &LiveVisualController::actionsChanged, this,
        [this, updatePathButtons](bool canStart, bool) { controlCanStart_ = canStart; updatePathButtons(); });
    connect(view_, &VisionOverlay::imagePicked, this, [this, x, y, h, updatePathButtons](double u, double v) {
        const auto preview = view_->preview();
        if (!preview) return;
        if (pathEditing_) {
            invalidatePathScopeIfChanged();
            if (!pathEditing_) return;
            const auto bounds = control_.workspaceBounds();
            const auto world = control_.mapImagePointToWorld(previewSource_, preview->frame->width,
                preview->frame->height, u, v, preview->frame->capturedAt, pathRevision_);
            if (bounds && world && draftPath_.append(*world, *bounds)) refreshPathOverlay();
            updatePathButtons();
            return;
        }
        bool okHeading{};
        const double heading = QLocale::c().toDouble(h->text(), &okHeading);
        if (!okHeading) return;
        const auto& frame = *preview->frame;
        if (control_.setTargetFromImage(previewSource_, frame.width, frame.height,
            u, v, heading, frame.capturedAt, previewWorkspaceRevision_)) {
            const auto target = control_.targetPose();
            if (target) {
                x->setText(QString::number(target->x.value, 'g', 8));
                y->setText(QString::number(target->y.value, 'g', 8));
            }
        }
    });
    connect(&control, &LiveVisualController::descriptionChanged, status, &QLabel::setText);
    connect(&control, &LiveVisualController::actionsChanged, this, [pause, apply, x, y, h, view = view_](bool, bool canStop) {
        pause->setEnabled(canStop);
        for (QWidget* widget : std::array<QWidget*, 5>{apply, x, y, h, view}) widget->setEnabled(!canStop);
    });
    connect(&control, &LiveVisualController::targetAvailabilityChanged, cancel, &QPushButton::setEnabled);
    connect(&control, &LiveVisualController::controllerFaultAcknowledgementChanged,
            acknowledge, &QPushButton::setEnabled);
    control.refresh();
    updatePathButtons();
    previewGateTimer_.setInterval(PreviewGateRefreshIntervalMs);
    connect(&previewGateTimer_, &QTimer::timeout, this, [this] {
        invalidatePathScopeIfChanged();
        refreshPathControls();
    });
    previewGateTimer_.start();
}
void VisualControlPanel::presentPreview(const QString& source, VisionDebugFramePtr preview)
{
    previewSource_ = source;
    previewWorkspaceRevision_ = control_.workspaceCalibrationRevision();
    view_->present(std::move(preview));
    invalidatePathScopeIfChanged();
    refreshPathControls();
}
bool VisualControlPanel::beginPathEditing()
{
    invalidatePathScopeIfChanged();
    if (pathEditing_ || control_.running() || !currentPreviewCanEditPath()) return false;
    const auto preview = view_->preview();
    const auto frame = control_.workspaceFrameId();
    if (!preview || !frame) return false;
    if (finishedPath_.empty()) {
        pathRobot_ = control_.selectedRobotId();
        pathFrame_ = frame;
        pathRevision_ = previewWorkspaceRevision_;
        pathSource_ = previewSource_;
        pathWidth_ = preview->frame->width;
        pathHeight_ = preview->frame->height;
    }
    draftPath_ = finishedPath_;
    pathEditing_ = true;
    previewInstruction_->setText(QStringLiteral("Path edit mode — click to add waypoints; letterbox margins are ignored"));
    refreshPathControls();
    return true;
}
bool VisualControlPanel::undoPathPoint()
{
    if (!pathEditing_ || !draftPath_.undo()) return false;
    refreshPathOverlay();
    refreshPathControls();
    return true;
}
void VisualControlPanel::clearPath()
{
    if (pathEditing_) draftPath_.clear();
    else finishedPath_.clear();
    refreshPathOverlay();
    refreshPathControls();
}
bool VisualControlPanel::finishPathEditing()
{
    if (!pathEditing_ || draftPath_.empty()) return false;
    invalidatePathScopeIfChanged();
    if (!pathEditing_) return false;
    finishedPath_ = draftPath_;
    draftPath_.clear();
    pathEditing_ = false;
    previewInstruction_->setText(QStringLiteral("Calibrated view — click a target"));
    refreshPathOverlay();
    refreshPathControls();
    return true;
}
void VisualControlPanel::cancelPathEditing()
{
    draftPath_.clear();
    pathEditing_ = false;
    previewInstruction_->setText(QStringLiteral("Calibrated view — click a target"));
    if (finishedPath_.empty()) {
        pathRobot_.reset(); pathFrame_.reset(); pathRevision_.clear(); pathSource_.clear();
        pathWidth_ = 0; pathHeight_ = 0;
    }
    refreshPathOverlay();
    refreshPathControls();
}
void VisualControlPanel::invalidatePathScopeIfChanged()
{
    if ((!draftPath_.empty() || !finishedPath_.empty() || pathEditing_) && (pathRobot_ != control_.selectedRobotId()
        || pathFrame_ != control_.workspaceFrameId()
        || pathRevision_ != control_.workspaceCalibrationRevision()
        || pathSource_ != previewSource_ || pathSource_ != control_.imageSource()
        || (view_->preview() && (pathWidth_ != view_->preview()->frame->width
            || pathHeight_ != view_->preview()->frame->height)))) {
        draftPath_.clear(); finishedPath_.clear(); pathEditing_ = false;
        previewInstruction_->setText(QStringLiteral("Calibrated view — click a target"));
        pathRobot_.reset(); pathFrame_.reset(); pathRevision_.clear(); pathSource_.clear();
        pathWidth_ = 0; pathHeight_ = 0;
        refreshPathOverlay();
        refreshPathControls();
    }
}
void VisualControlPanel::refreshPathControls()
{
    const bool editing = pathEditing_;
    const bool previewCanEdit = currentPreviewCanEditPath();
    pathBeginButton_->setEnabled(!editing && !control_.running() && previewCanEdit);
    pathUndoButton_->setEnabled(editing && !draftPath_.empty());
    pathClearButton_->setEnabled(editing ? !draftPath_.empty() : !finishedPath_.empty());
    pathFinishButton_->setEnabled(editing && !draftPath_.empty());
    pathCancelButton_->setEnabled(editing);
    startButton_->setEnabled(controlCanStart_ && !editing);
    const auto preview = view_->preview();
    const bool overlayVisible = preview && previewCanEdit
        && (!draftPath_.empty() || !finishedPath_.empty());
    view_->setPathOverlayVisible(overlayVisible);
}
bool VisualControlPanel::currentPreviewCanEditPath() const
{
    const auto preview = view_->preview();
    return preview && control_.canEditPathFromImage(previewSource_, preview->frame->width,
        preview->frame->height, preview->frame->capturedAt, previewWorkspaceRevision_);
}
void VisualControlPanel::refreshPathOverlay()
{
    const auto& path = pathEditing_ ? draftPath_ : finishedPath_;
    std::vector<ImagePoint> raw;
    raw.reserve(path.size());
    for (const auto point : path.points()) {
        const auto mapped = control_.mapWorldPointToRawImage(point);
        if (mapped) raw.push_back({mapped->u, mapped->v});
    }
    view_->setPathOverlay(std::move(raw));
}
} // namespace micro_robot
