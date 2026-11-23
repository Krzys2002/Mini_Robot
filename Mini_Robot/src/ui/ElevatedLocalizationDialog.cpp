#include "ui/ElevatedLocalizationDialog.hpp"
#include "ui/CalibrationForm.hpp"

#include <QCheckBox>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QUuid>
#include <QVBoxLayout>

#include <iomanip>
#include <locale>
#include <sstream>

namespace micro_robot {
namespace {
bool readSurveyPoints(const QString& text, std::vector<ExtrinsicReference>& points)
{
    std::istringstream input{text.toStdString()};
    for (std::string line; std::getline(input, line);) {
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
        std::istringstream row{line};
        row.imbue(std::locale::classic());
        std::string role;
        ExtrinsicReference point;
        if (!(row >> role >> point.image.u >> point.image.v >> point.world.x.value >> point.world.y.value)
            || (role != "Fit" && role != "Check")
            || points.size() >= MaximumExtrinsicReferences) return false;
        if (!(row >> std::ws).eof()) {
            double height;
            if (!(row >> height) || !(row >> std::ws).eof()) return false;
            point.heightMeters = height;
        }
        point.check = role == "Check";
        points.push_back(point);
    }
    return true;
}

bool readMounts(const QString& text, std::vector<RobotMount>& mounts)
{
    std::istringstream input{text.toStdString()};
    for (std::string line; std::getline(input, line);) {
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
        std::istringstream row{line};
        row.imbue(std::locale::classic());
        RobotMount mount;
        std::uint64_t id;
        auto quoted = [&](std::string& value) { row >> std::ws; return row.peek() == '"' && bool(row >> std::quoted(value)); };
        if (!(row >> id >> mount.tagHeightMeters >> mount.bodyForwardMeters >> mount.bodyLeftMeters >> mount.bodyHeadingOffsetRadians)
            || id > std::numeric_limits<std::uint32_t>::max() || !quoted(mount.revision) || !quoted(mount.measurementMethod)
            || !(row >> std::ws).eof() || mounts.size() >= MaximumRobotMounts) return false;
        mount.robotId = RobotId{static_cast<std::uint32_t>(id)};
        mounts.push_back(std::move(mount));
    }
    return true;
}

struct RobotCalibrationInputState {
    QStringList baseline;
    QStringList submitted;
    std::string pendingRevision;
    bool dirty{};
};
} // namespace

ElevatedLocalizationDialog::ElevatedLocalizationDialog(WorkspaceCalibrationController& controller, QWidget* parent, bool guided)
    : QDialog(parent)
{
    setWindowTitle(guided ? "Robot calibration" : "Elevated tag / axle-body localization");
    resize(800,740);
    auto* outer = new QVBoxLayout(this);
    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    auto* body = new QWidget(scroll);
    auto* layout = new QVBoxLayout(body);
    scroll->setWidget(body);
    outer->addWidget(scroll);

    auto addLabel = [body, layout](const QString& text) {
        auto* label = new QLabel(text, body);
        label->setWordWrap(true);
        label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        layout->addWidget(label);
        return label;
    };
    addLabel("Use the calibrated tabletop frame for the robot tag and axle measurements. "
             "Select pixels in the matching undistorted camera image. Survey Fit and Check points independently.");

    auto* form = new QFormLayout;
    form->setRowWrapPolicy(QFormLayout::WrapLongRows);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    layout->addLayout(form);
    auto* inputs = new CalibrationForm(this);
    auto* surface = inputs->number(*form, "referenceSurfaceHeight", "Measured corner-surface z (m)", 0, MaximumSurveyEntryMeters);
    auto* board = inputs->number(*form, "referenceBoardThickness", "Measured board thickness (m)", 0, MaximumSurveyEntryMeters);
    auto* backing = inputs->number(*form, "referenceBackingThickness", "Measured backing thickness (m)", 0, MaximumSurveyEntryMeters);
    auto* pixelLimit = inputs->number(*form, "extrinsicPixelLimit", "Maximum reprojection error (px)",
                                      0, MaximumSurveyPixelLimit, DefaultExtrinsicPixelLimit);
    auto* axisLimit = inputs->number(*form, "extrinsicAxisLimit", "Maximum held-out error on each axis (m)",
                                     0, MaximumSurveyEntryMeters, DefaultExtrinsicAxisLimitMeters);
    auto* method = new QLineEdit(body);
    method->setObjectName("extrinsicSurveyMethod");
    form->addRow("Survey method / independent evidence ID", method);
    inputs->watch(method, [method] { return method->text(); }, [method] {
        if (method->text().trimmed().isEmpty()) return QStringLiteral("Enter the survey method.");
        if (method->text().toUtf8().size() > static_cast<qsizetype>(MaximumGeometryNoteBytes))
            return QStringLiteral("The survey method exceeds the supported length.");
        return QString{};
    });

    addLabel("References take one row per point. Enter Fit or Check, then u_px v_px x_m y_m using decimal points. "
             "Add z_m at the end of a row whose measured height differs from the corner-surface height above.");
    auto* references = new QPlainTextEdit(body);
    references->setObjectName("extrinsicReferences");
    layout->addWidget(references);
    inputs->watch(references, [references] { return references->toPlainText(); }, [references] {
        std::vector<ExtrinsicReference> points;
        return readSurveyPoints(references->toPlainText(), points) && !points.empty()
            ? QString{} : QStringLiteral("Enter valid Fit and Check rows with native pixels and world coordinates.");
    });
    addLabel("Mount rows use RobotId height_m forward_m left_m heading_rad \"mount_revision\" \"measurement_method\". "
             "Measure offsets from the tag centre to the axle/body centre in the tag-heading frame.");
    auto* mounts = new QPlainTextEdit(body);
    mounts->setObjectName("robotMounts");
    layout->addWidget(mounts);
    inputs->watch(mounts, [mounts] { return mounts->toPlainText(); }, [mounts, guided] {
        std::vector<RobotMount> rows;
        return readMounts(mounts->toPlainText(), rows) && (!guided || !rows.empty())
            ? QString{} : QStringLiteral("Enter valid measured robot mount rows.");
    });

    auto* confirmed = new QCheckBox("I measured the geometry and surveyed Check points independently.", body);
    confirmed->setObjectName("surveyMeasurementsConfirmed");
    layout->addWidget(confirmed);
    auto* status = addLabel(controller.message());
    status->setObjectName("robotCalibrationStatus");
    status->setTextFormat(Qt::PlainText);

    auto* actions = new QHBoxLayout;
    outer->addLayout(actions);
    auto* calculate = new QPushButton("Estimate and validate draft", this);
    calculate->setObjectName("estimateExtrinsics");
    actions->addWidget(calculate);
    auto* save = new QPushButton("Save profile", this);
    save->setObjectName("robotCalibrationSave");
    save->setVisible(guided);
    actions->addWidget(save);
    auto* close = new QPushButton("Close", this);
    actions->addWidget(close);
    auto* next = new QPushButton("Next", this);
    next->setObjectName("robotCalibrationNext");
    next->setVisible(guided);
    actions->addWidget(next);
    for (auto* button : {calculate, save, close, next}) button->setAutoDefault(false);

    if (controller.draftProfile() && controller.draftProfile()->localization) {
        const auto& geometry = *controller.draftProfile()->localization;
        const auto& survey = geometry.survey;
        CalibrationForm::setValue(surface, survey.surfaceHeightMeters);
        CalibrationForm::setValue(board, survey.boardThicknessMeters);
        CalibrationForm::setValue(backing, survey.backingThicknessMeters);
        CalibrationForm::setValue(pixelLimit, survey.maximumReprojectionPixels);
        CalibrationForm::setValue(axisLimit, survey.maximumCheckAxisMeters);
        method->setText(QString::fromStdString(survey.measurementMethod));
        std::ostringstream pointRows, mountRows;
        pointRows.imbue(std::locale::classic());
        mountRows.imbue(std::locale::classic());
        pointRows << std::setprecision(std::numeric_limits<double>::max_digits10);
        mountRows << std::setprecision(std::numeric_limits<double>::max_digits10);
        for (const auto& point : survey.references) {
            pointRows << (point.check ? "Check" : "Fit") << ' ' << point.image.u << ' ' << point.image.v << ' '
                      << point.world.x.value << ' ' << point.world.y.value;
            if (point.heightMeters) pointRows << ' ' << *point.heightMeters;
            pointRows << '\n';
        }
        for (const auto& mount : geometry.mounts) {
            mountRows << mount.robotId.value() << ' ' << mount.tagHeightMeters << ' ' << mount.bodyForwardMeters << ' '
                      << mount.bodyLeftMeters << ' ' << mount.bodyHeadingOffsetRadians << ' '
                      << std::quoted(mount.revision) << ' ' << std::quoted(mount.measurementMethod) << '\n';
        }
        references->setPlainText(QString::fromStdString(pointRows.str()));
        mounts->setPlainText(QString::fromStdString(mountRows.str()));
    }

    const auto state = std::make_shared<RobotCalibrationInputState>();
    state->baseline = inputs->snapshot();
    hasPendingInput_ = [state] { return state->dirty; };
    auto canAccept = [&controller, state] {
        const auto& draft = controller.draftProfile();
        return !state->dirty && controller.canActivate() && draft->localization && !draft->localization->mounts.empty();
    };
    auto updateActions = [&controller, body, calculate, next, save, canAccept] {
        body->setEnabled(!controller.busy());
        calculate->setEnabled(!controller.busy() && controller.draftProfile() && controller.draftProfile()->homography);
        const bool ready = canAccept();
        next->setEnabled(ready);
        save->setEnabled(ready && controller.canSave());
    };
    auto edited = [inputs, confirmed, state, updateActions] {
        state->dirty = inputs->snapshot() != state->baseline;
        confirmed->setChecked(false);
        updateActions();
    };
    for (auto* field : {surface, board, backing, pixelLimit, axisLimit, method})
        connect(field, &QLineEdit::textChanged, this, edited);
    connect(references, &QPlainTextEdit::textChanged, this, edited);
    connect(mounts, &QPlainTextEdit::textChanged, this, edited);

    connect(&controller, &WorkspaceCalibrationController::changed, this, [&, state, updateActions, status, inputs] {
        if (!controller.busy() && !state->pendingRevision.empty()) {
            const auto& draft = controller.draftProfile();
            if (draft && draft->localization && draft->localization->revision == state->pendingRevision)
                state->baseline = std::move(state->submitted);
            state->pendingRevision.clear();
            state->dirty = inputs->snapshot() != state->baseline;
        }
        status->setText(controller.message());
        updateActions();
    });
    updateActions();
    connect(close, &QPushButton::clicked, this, &ElevatedLocalizationDialog::reject);
    connect(calculate, &QPushButton::clicked, this, [=, &controller, this] {
        if (!confirmed->isChecked()) {
            status->setText("Confirm independently measured geometry before estimating.");
            return;
        }
        const auto error = inputs->validate();
        if (!error.isEmpty()) {
            status->setText(error);
            return;
        }

        ExtrinsicSolveInput input;
        input.revision = QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
        auto& survey = input.survey;
        survey.surfaceHeightMeters = CalibrationForm::value(surface);
        survey.boardThicknessMeters = CalibrationForm::value(board);
        survey.backingThicknessMeters = CalibrationForm::value(backing);
        survey.maximumReprojectionPixels = CalibrationForm::value(pixelLimit, DefaultExtrinsicPixelLimit);
        survey.maximumCheckAxisMeters = CalibrationForm::value(axisLimit, DefaultExtrinsicAxisLimitMeters);
        survey.measurementMethod = method->text().toStdString();
        if (!readSurveyPoints(references->toPlainText(), survey.references) || !readMounts(mounts->toPlainText(), input.mounts))
            return;

        const auto& draft = controller.draftProfile();
        if (draft && (draft->localization || draft->references)
            && !confirmCalibrationLoss(this, "Replace robot calibration draft?",
                "Estimating new robot geometry can replace the current draft and invalidate fitted surveyed references."))
            return;
        state->pendingRevision = input.revision;
        state->submitted = inputs->snapshot();
        controller.calculateLocalization(std::move(input));
    });
    connect(next, &QPushButton::clicked, this, [this, &controller, canAccept] {
        if (!canAccept()) return;
        const auto revision = controller.draftProfile()->localization->revision;
        const auto& active = controller.profile();
        if (active && active->localization && active->localization->revision != revision
            && !confirmCalibrationLoss(this, "Replace active robot calibration?",
                "Accepting this calibration replaces the active robot geometry."))
            return;
        if (!canAccept() || controller.draftProfile()->localization->revision != revision) return;
        controller.activate();
        accept();
    });
    connect(save, &QPushButton::clicked, this, [&controller, this, canAccept] {
        if (canAccept() && controller.canSave()
            && confirmCalibrationLoss(this, "Save calibration profile?",
                "Saving replaces any stored calibration for this workspace and camera.")
            && canAccept())
            controller.save();
    });
}

void ElevatedLocalizationDialog::reject()
{
    if (!hasPendingInput_ || !hasPendingInput_()
        || confirmCalibrationLoss(this, "Discard robot calibration edits?", "Closing discards unsubmitted robot survey and mount edits."))
        QDialog::reject();
}
} // namespace micro_robot
