#include "ui/ReferenceMarkersDialog.hpp"
#include "ui/CalibrationForm.hpp"
#include "ui/ProfileFileDialog.hpp"

#include <QCheckBox>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

#include <iomanip>
#include <locale>
#include <sstream>

namespace micro_robot {
namespace {
bool readRegistry(const QString& text, std::vector<SurveyedReferenceMarker>& markers)
{
    std::istringstream rows{text.toStdString()};
    for (std::string line; std::getline(rows, line);) {
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
        SurveyedReferenceMarker marker;
        std::string role;
        int flag;
        std::istringstream row{line};
        row.imbue(std::locale::classic());
        auto quoted = [&](std::string& value) { row >> std::ws; return row.peek() == '"' && bool(row >> std::quoted(value)); };
        bool valid = bool(row >> role >> marker.id >> marker.surfaceHeightMeters >> marker.blackEdgeMeters
                             >> marker.standardUncertaintyMeters >> flag)
            && (role == "Fit" || role == "Check" || role == "Monitor") && (flag == 0 || flag == 1)
            && quoted(marker.installationRevision) && quoted(marker.surveyRevision) && quoted(marker.method);
        for (auto& point : marker.corners) valid = valid && bool(row >> point.x.value >> point.y.value);
        if (!valid || !(row >> std::ws).eof() || markers.size() >= MaximumReferenceMarkers) return false;
        marker.role = role == "Fit" ? ReferenceRole::Fit
            : role == "Check" ? ReferenceRole::IndependentCheck : ReferenceRole::DriftMonitor;
        marker.independentlySurveyed = flag == 1;
        markers.push_back(std::move(marker));
    }
    return true;
}

QString registryText(const ReferenceConfiguration& configuration)
{
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const auto& marker : configuration.markers) {
        out << (marker.role == ReferenceRole::Fit ? "Fit" : marker.role == ReferenceRole::IndependentCheck ? "Check" : "Monitor")
            << ' ' << marker.id << ' ' << marker.surfaceHeightMeters << ' ' << marker.blackEdgeMeters
            << ' ' << marker.standardUncertaintyMeters << ' ' << marker.independentlySurveyed
            << ' ' << std::quoted(marker.installationRevision) << ' ' << std::quoted(marker.surveyRevision)
            << ' ' << std::quoted(marker.method);
        for (const auto& point : marker.corners) out << ' ' << point.x.value << ' ' << point.y.value;
        out << '\n';
    }
    return QString::fromStdString(out.str());
}

struct ReferenceCalibrationInputState {
    QStringList baseline;
    bool dirty{};
};
} // namespace

ReferenceMarkersDialog::ReferenceMarkersDialog(WorkspaceCalibrationController& controller, QWidget* parent, bool guided)
    : QDialog(parent)
{
    setWindowTitle(guided ? "Other calibration · Surveyed references" : "Surveyed references and placement monitoring");
    resize(900, 760);
    auto* outer = new QVBoxLayout(this);
    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    auto* body = new QWidget(scroll);
    auto* layout = new QVBoxLayout(body);
    scroll->setWidget(body);
    outer->addWidget(scroll);
    auto label = [body, layout](const QString& text) {
        auto* item = new QLabel(text, body);
        item->setWordWrap(true);
        item->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        item->setTextFormat(Qt::PlainText);
        layout->addWidget(item);
        return item;
    };
    label("Use surveyed marker IDs outside the robot ID allocation. Fit marker centres and independently surveyed "
          "Check marker centres must each surround the workspace.");

    auto* form = new QFormLayout;
    form->setRowWrapPolicy(QFormLayout::WrapLongRows);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    layout->addLayout(form);
    auto* inputs = new CalibrationForm(this);
    const double minMs = static_cast<double>(MinimumReferenceCadence.count());
    const double maxMs = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(MaximumReferenceCadence).count());
    const ReferencePolicy initial;
    auto* cadence = inputs->number(*form, "referenceCadence", "Monitoring cadence (ms)",
                                   minMs, maxMs, static_cast<double>(initial.cadence.count()), true);
    auto* age = inputs->number(*form, "referenceMaximumAge", "Maximum evidence age (ms)",
                               minMs, maxMs, static_cast<double>(initial.maximumEvidenceAge.count()), true);
    auto* duration = inputs->number(*form, "referenceDriftDuration", "Minimum drift persistence (ms)",
                                    minMs, maxMs, static_cast<double>(initial.minimumDriftDuration.count()), true);
    auto* samples = inputs->number(*form, "referenceDriftSamples", "Consecutive drift samples",
                                   MinimumReferenceDriftSamples, MaximumReferenceMarkers, initial.consecutiveDriftSamples, true);
    auto* pixels = inputs->number(*form, "referencePixelLimit", "Maximum residual (px)",
                                  0, MaximumSurveyPixelLimit, initial.maximumResidualPixels);
    auto* metres = inputs->number(*form, "referenceAxisLimit", "Maximum residual on each axis (m)",
                                  0, MaximumSurveyEntryMeters, initial.maximumResidualAxisMeters);
    auto* notes = new QLineEdit(body);
    notes->setObjectName("referenceSharedErrors");
    form->addRow("Survey uncertainty / shared-error limitations", notes);
    inputs->watch(notes, [notes] { return notes->text(); }, [notes] {
        return notes->text().toUtf8().size() <= static_cast<qsizetype>(MaximumGeometryNoteBytes) ? QString{}
            : QStringLiteral("Survey notes exceed the supported length.");
    });

    label("Enter one row per tag36h11 reference using decimal points. Each row needs Fit, Check or Monitor, then the ID, "
          "surface height, black edge length, uncertainty, independence flag (0 or 1), quoted installation revision, "
          "quoted survey revision and quoted method. Finish with the four measured world X/Y corner pairs in decoded "
          "TL, TR, BR, BL order. Use metres.");
    auto* registry = new QPlainTextEdit(body);
    registry->setObjectName("referenceRegistryRows");
    registry->setMinimumHeight(130);
    layout->addWidget(registry);
    inputs->watch(registry, [registry] { return registry->toPlainText(); }, [registry] {
        std::vector<SurveyedReferenceMarker> rows;
        return readRegistry(registry->toPlainText(), rows) && !rows.empty() ? QString{}
            : QStringLiteral("Enter valid reference rows with all survey fields and four ordered corner pairs.");
    });
    auto* confirmed = new QCheckBox("I measured the registry and surveyed Check markers independently.", body);
    confirmed->setObjectName("referenceSurveyConfirmed");
    layout->addWidget(confirmed);
    label("Latest detected reference corners");
    auto* evidence = new QPlainTextEdit(body);
    evidence->setObjectName("referenceEvidenceRows");
    evidence->setReadOnly(true);
    evidence->setMinimumHeight(100);
    layout->addWidget(evidence);
    auto* status = label("");
    status->setObjectName("referenceStatus");

    auto* actions = new QHBoxLayout;
    outer->addLayout(actions);
    auto button = [&](const char* text, const char* name) {
        auto* action = new QPushButton(text, this);
        action->setObjectName(name);
        action->setAutoDefault(false);
        actions->addWidget(action);
        return action;
    };
    auto* apply = button("Apply registry draft", "applyReferenceRegistry");
    auto* fit = button("Fit latest references", "fitReferenceRegistry");
    auto* save = button("Save profile", "referenceCalibrationSave");
    auto* saveFile = button("Save as…", "referenceCalibrationSaveFile");
    saveFile->setVisible(guided);
    auto* close = button("Close", "closeReferences");
    auto* finish = button("Finish", "otherCalibrationFinish");
    save->setVisible(guided);
    finish->setVisible(guided);

    if (controller.draftProfile() && controller.draftProfile()->references) {
        const auto& configuration = *controller.draftProfile()->references;
        const auto& policy = configuration.policy;
        CalibrationForm::setValue(cadence, static_cast<double>(policy.cadence.count()));
        CalibrationForm::setValue(age, static_cast<double>(policy.maximumEvidenceAge.count()));
        CalibrationForm::setValue(duration, static_cast<double>(policy.minimumDriftDuration.count()));
        CalibrationForm::setValue(samples, static_cast<double>(policy.consecutiveDriftSamples));
        CalibrationForm::setValue(pixels, policy.maximumResidualPixels);
        CalibrationForm::setValue(metres, policy.maximumResidualAxisMeters);
        notes->setText(QString::fromStdString(configuration.sharedErrorNotes));
        registry->setPlainText(registryText(configuration));
    }

    const auto state = std::make_shared<ReferenceCalibrationInputState>();
    state->baseline = inputs->snapshot();
    hasPendingInput_ = [state] { return state->dirty; };
    auto canAccept = [&controller, state] {
        return !state->dirty && controller.canActivate() && controller.draftProfile()->references.has_value();
    };
    auto updateActions = [=, &controller] {
        const auto& draft = controller.draftProfile();
        const bool editable = !controller.busy();
        body->setEnabled(editable);
        apply->setEnabled(editable && draft && draft->localization);
        fit->setEnabled(editable && !state->dirty && bool(controller.latestReferenceFrame()));
        const bool ready = canAccept();
        finish->setEnabled(ready);
        save->setEnabled(ready && controller.canSave());
        saveFile->setEnabled(save->isEnabled());
    };
    auto edited = [=] {
        state->dirty = inputs->snapshot() != state->baseline;
        confirmed->setChecked(false);
        updateActions();
    };
    for (auto* field : {cadence, age, duration, samples, pixels, metres, notes})
        connect(field, &QLineEdit::textChanged, this, edited);
    connect(registry, &QPlainTextEdit::textChanged, this, edited);

    connect(apply, &QPushButton::clicked, this, [=, &controller, this] {
        if (!confirmed->isChecked()) {
            status->setText("Confirm measured survey data before applying a registry.");
            return;
        }
        const auto validationError = inputs->validate();
        if (!validationError.isEmpty()) {
            status->setText(validationError);
            return;
        }
        auto milliseconds = [](const QLineEdit* field, std::chrono::milliseconds fallback) {
            return std::chrono::milliseconds{static_cast<std::chrono::milliseconds::rep>(
                CalibrationForm::value(field, static_cast<double>(fallback.count())))};
        };
        ReferenceConfiguration configuration;
        configuration.sharedErrorNotes = notes->text().toStdString();
        configuration.policy = {milliseconds(cadence, initial.cadence), milliseconds(age, initial.maximumEvidenceAge),
            milliseconds(duration, initial.minimumDriftDuration),
            static_cast<std::size_t>(CalibrationForm::value(samples, static_cast<double>(initial.consecutiveDriftSamples))),
            CalibrationForm::value(pixels, initial.maximumResidualPixels),
            CalibrationForm::value(metres, initial.maximumResidualAxisMeters)};
        if (!readRegistry(registry->toPlainText(), configuration.markers)) return;
        const auto& draft = controller.draftProfile();
        const std::string revision = draft && draft->references ? draft->references->revision : std::string{};
        if (!revision.empty() && !confirmCalibrationLoss(this, "Replace reference registry draft?",
                "Applying this registry discards the previous draft registry and its fitted evidence."))
            return;
        controller.configureReferences(std::move(configuration));
        if (draft && draft->references && draft->references->revision != revision) {
            state->baseline = inputs->snapshot();
            state->dirty = false;
            updateActions();
        }
    });
    connect(fit, &QPushButton::clicked, this, [this, &controller, canAccept] {
        if (canAccept() && !confirmCalibrationLoss(this, "Fit surveyed references again?",
                "Fitting new evidence can replace the current reference fit and its acquisition snapshot."))
            return;
        controller.calculateReferences();
    });
    connect(finish, &QPushButton::clicked, this, [this, &controller, canAccept] {
        if (!canAccept()) return;
        const auto revision = controller.draftProfile()->localization->revision;
        const auto& active = controller.profile();
        if (active && active->localization && active->localization->revision != revision
            && !confirmCalibrationLoss(this, "Replace active reference calibration?",
                "Accepting the new reference fit replaces the active camera geometry and surveyed references."))
            return;
        if (!canAccept() || controller.draftProfile()->localization->revision != revision) return;
        controller.activate();
        accept();
    });
    connect(saveFile, &QPushButton::clicked, this, [this, &controller, canAccept] {
        const auto file = chooseProfileFile(this, ProfileFileKind::Workspace, QFileDialog::AcceptSave);
        if (!file.isEmpty() && canAccept()) controller.save(file);
    });
    connect(save, &QPushButton::clicked, this, [this, &controller, canAccept] {
        if (canAccept() && controller.canSave()
            && confirmCalibrationLoss(this, "Save calibration profile?",
                "Saving replaces any stored calibration for this workspace and camera.") && canAccept())
            controller.save();
    });
    connect(close, &QPushButton::clicked, this, &ReferenceMarkersDialog::reject);

    auto refresh = [=, &controller] {
        status->setText(controller.message());
        updateActions();
        std::ostringstream text;
        text.imbue(std::locale::classic());
        if (const auto frame = controller.latestReferenceFrame()) for (const auto& row : frame->observations) {
            static constexpr const char* reasons[]{"Included", "Missing/occluded", "Duplicate identity", "Invalid corners", "Unsupported ray/image"};
            text << row.id << ": " << reasons[static_cast<int>(row.rejection)] << " raw:";
            for (const auto point : row.raw) text << ' ' << point.u << ',' << point.v;
            text << " undistorted:";
            for (const auto point : row.undistorted) text << ' ' << point.u << ',' << point.v;
            text << '\n';
        }
        evidence->setPlainText(QString::fromStdString(text.str()));
    };
    connect(&controller, &WorkspaceCalibrationController::changed, this, refresh);
    refresh();
}

void ReferenceMarkersDialog::reject()
{
    if (!hasPendingInput_ || !hasPendingInput_()
        || confirmCalibrationLoss(this, "Discard reference calibration edits?", "Closing discards unsubmitted registry and policy edits."))
        QDialog::reject();
}
} // namespace micro_robot
