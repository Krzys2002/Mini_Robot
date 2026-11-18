#include "ui/VisualControlPanel.hpp"
#include "app/LiveVisualController.hpp"
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
VisualControlPanel::VisualControlPanel(LiveVisualController& control, QWidget* parent) : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    auto* group = new QGroupBox(QStringLiteral("Visual goal control — selected robot"), this);
    auto* form = new QFormLayout(group);
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
    auto* stop = new QPushButton(QStringLiteral("Stop visual control"), group);
    stop->setObjectName(QStringLiteral("stopVisualControl")); stop->setEnabled(false);
    buttons->addWidget(apply); buttons->addWidget(start); buttons->addWidget(stop); form->addRow(buttons);
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
    connect(stop, &QPushButton::clicked, &control, &LiveVisualController::stop);
    connect(&control, &LiveVisualController::descriptionChanged, status, &QLabel::setText);
    connect(&control, &LiveVisualController::actionsChanged, this, [start, stop, apply, x, y, h](bool canStart, bool canStop) {
        start->setEnabled(canStart); stop->setEnabled(canStop);
        for (QWidget* widget : std::array<QWidget*, 4>{apply, x, y, h}) widget->setEnabled(!canStop);
    });
    control.refresh();
}
} // namespace micro_robot
