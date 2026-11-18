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
    form->addRow(QStringLiteral("Calibrated view — click a target"), view_);
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
    auto* start = new QPushButton(QStringLiteral("Start visual control"), group);
    start->setObjectName(QStringLiteral("startVisualControl")); start->setEnabled(false);
    auto* pause = new QPushButton(QStringLiteral("Pause"), group);
    pause->setObjectName(QStringLiteral("pauseVisualControl")); pause->setEnabled(false);
    auto* cancel = new QPushButton(QStringLiteral("Cancel target"), group);
    cancel->setObjectName(QStringLiteral("cancelVisualControl")); cancel->setEnabled(false);
    auto* acknowledge = new QPushButton(QStringLiteral("Acknowledge controller fault"), group);
    acknowledge->setObjectName(QStringLiteral("acknowledgeVisualFault")); acknowledge->setEnabled(false);
    buttons->addWidget(apply); buttons->addWidget(start); buttons->addWidget(pause);
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
    connect(start, &QPushButton::clicked, &control, [&control] { (void)control.start(); });
    connect(acknowledge, &QPushButton::clicked, &control, [&control] { (void)control.acknowledgeControllerFault(); });
    connect(pause, &QPushButton::clicked, &control, &LiveVisualController::pause);
    connect(cancel, &QPushButton::clicked, this, [&control, x, y] {
        control.cancel();
        x->setText(QStringLiteral("0"));
        y->setText(QStringLiteral("0"));
    });
    connect(view_, &VisionOverlay::imagePicked, this, [this, x, y, h](double u, double v) {
        const auto preview = view_->preview();
        if (!preview) return;
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
    connect(&control, &LiveVisualController::actionsChanged, this, [start, pause, apply, x, y, h, view = view_](bool canStart, bool canStop) {
        start->setEnabled(canStart); pause->setEnabled(canStop);
        for (QWidget* widget : std::array<QWidget*, 5>{apply, x, y, h, view}) widget->setEnabled(!canStop);
    });
    connect(&control, &LiveVisualController::targetAvailabilityChanged, cancel, &QPushButton::setEnabled);
    connect(&control, &LiveVisualController::controllerFaultAcknowledgementChanged,
            acknowledge, &QPushButton::setEnabled);
    control.refresh();
}
void VisualControlPanel::presentPreview(const QString& source, VisionDebugFramePtr preview)
{
    previewSource_ = source;
    previewWorkspaceRevision_ = control_.workspaceCalibrationRevision();
    view_->present(std::move(preview));
}
} // namespace micro_robot
