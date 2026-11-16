#include "ui/ElevatedLocalizationDialog.hpp"
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QUuid>
#include <QVBoxLayout>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>

namespace micro_robot {
ElevatedLocalizationDialog::ElevatedLocalizationDialog(WorkspaceCalibrationController& controller,QWidget* parent):QDialog(parent) {
    setWindowTitle("Elevated tag / axle-body localization"); resize(800,740);
    auto* outer=new QVBoxLayout(this); auto* scroll=new QScrollArea(this); scroll->setWidgetResizable(true);
    auto* body=new QWidget(scroll); auto* layout=new QVBoxLayout(body); scroll->setWidget(body); outer->addWidget(scroll);
    auto* help=new QLabel("Load or calculate the tabletop draft first. Supply measured references in that exact signed world frame. "
        "Pixels must be undistorted native-K coordinates from the matching camera/resolution. World +Z points above the table. "
        "Fit and independently surveyed Check points must cover both axes; Check rows are held out. "
        "A planar fit alone does not validate physical accuracy. Unknown geometry produces no corrected body pose.",body);
    help->setWordWrap(true); layout->addWidget(help);
    auto* form=new QFormLayout; layout->addLayout(form);
    auto spin=[&](const char* name,const char* label,double value,double maximum) {
        auto* w=new QDoubleSpinBox(body); w->setObjectName(name); w->setDecimals(6); w->setRange(0,maximum); w->setValue(value); form->addRow(label,w); return w;
    };
    // These bounds are presentation capacity, not claims about measured hardware.
    auto* surface=spin("referenceSurfaceHeight","Measured corner-surface z (m)",0,MaximumSurveyEntryMeters);
    auto* board=spin("referenceBoardThickness","Measured board thickness (m)",0,MaximumSurveyEntryMeters);
    auto* backing=spin("referenceBackingThickness","Measured backing thickness (m)",0,MaximumSurveyEntryMeters);
    auto* pixelLimit=spin("extrinsicPixelLimit","Maximum reference reprojection error (px)",DefaultExtrinsicPixelLimit,MaximumSurveyPixelLimit);
    auto* axisLimit=spin("extrinsicAxisLimit","Maximum held-out error on each axis (m)",DefaultExtrinsicAxisLimitMeters,MaximumSurveyEntryMeters);
    auto* method=new QLineEdit(body); method->setObjectName("extrinsicSurveyMethod"); method->setMaxLength(static_cast<int>(MaximumGeometryNoteBytes));
    form->addRow("Survey method / independent evidence ID",method);
    auto* refsLabel=new QLabel("References: one row per point, Fit or Check, then u_px v_px x_m y_m (maximum 256). "
        "Surface z above applies to all rows, including board/backing. Do not import a mixed-height survey.",body);
    refsLabel->setWordWrap(true); layout->addWidget(refsLabel);
    auto* references=new QPlainTextEdit(body); references->setObjectName("extrinsicReferences");
    references->setPlaceholderText("Fit  u_px v_px x_m y_m\nCheck u_px v_px x_m y_m"); layout->addWidget(references);
    auto* mountsLabel=new QLabel("Mounts: RobotId height_m body_forward_m body_left_m body_heading_rad \"mount_revision\" \"measurement_method\". "
        "Offsets go from tag centre to axle/body centre in tag-heading axes (+forward, +left). Identity mapping stays separate. "
        "All tag corners, tag centre and body centre must lie in the workspace; this does not check body footprint or control clearance.",body);
    mountsLabel->setWordWrap(true); layout->addWidget(mountsLabel);
    auto* mounts=new QPlainTextEdit(body); mounts->setObjectName("robotMounts");
    mounts->setPlaceholderText("0 0.050 0.000 0.000 0.000 \"mount-A\" \"measured with calipers\""); layout->addWidget(mounts);
    auto* confirmed=new QCheckBox("I measured the surface/thicknesses and mounts, and surveyed Check points independently.",body);
    confirmed->setObjectName("surveyMeasurementsConfirmed"); layout->addWidget(confirmed);
    auto* status=new QLabel(controller.message(),body); status->setWordWrap(true); status->setTextFormat(Qt::PlainText); layout->addWidget(status);
    auto* actions=new QHBoxLayout; outer->addLayout(actions);
    auto* calculate=new QPushButton("Estimate and validate draft",this); calculate->setObjectName("estimateExtrinsics"); actions->addWidget(calculate);
    auto* close=new QPushButton("Close",this); actions->addWidget(close);
    if (controller.draftProfile() && controller.draftProfile()->localization) {
        const auto& e=*controller.draftProfile()->localization; const auto& s=e.survey;
        surface->setValue(s.surfaceHeightMeters); board->setValue(s.boardThicknessMeters); backing->setValue(s.backingThicknessMeters);
        pixelLimit->setValue(s.maximumReprojectionPixels); axisLimit->setValue(s.maximumCheckAxisMeters); method->setText(QString::fromStdString(s.measurementMethod));
        std::ostringstream points,mountText; points.imbue(std::locale::classic()); mountText.imbue(std::locale::classic());
        points<<std::setprecision(std::numeric_limits<double>::max_digits10); mountText<<std::setprecision(std::numeric_limits<double>::max_digits10);
        for (const auto& p:s.references) points<<(p.check?"Check":"Fit")<<' '<<p.image.u<<' '<<p.image.v<<' '<<p.world.x.value<<' '<<p.world.y.value<<'\n';
        for (const auto& m:e.mounts) mountText<<m.robotId.value()<<' '<<m.tagHeightMeters<<' '<<m.bodyForwardMeters<<' '<<m.bodyLeftMeters<<' '<<m.bodyHeadingOffsetRadians
            <<' '<<std::quoted(m.revision)<<' '<<std::quoted(m.measurementMethod)<<'\n';
        references->setPlainText(QString::fromStdString(points.str())); mounts->setPlainText(QString::fromStdString(mountText.str()));
    }
    auto edited=[&controller,confirmed] { confirmed->setChecked(false); controller.localizationInputsChanged(); };
    for (auto* w:{surface,board,backing,pixelLimit,axisLimit}) connect(w,&QDoubleSpinBox::valueChanged,this,edited);
    connect(method,&QLineEdit::textChanged,this,edited);
    connect(references,&QPlainTextEdit::textChanged,this,edited); connect(mounts,&QPlainTextEdit::textChanged,this,edited);
    connect(close,&QPushButton::clicked,this,&QDialog::close);
    auto refresh=[&controller,status,calculate] {
        status->setText(controller.message()); calculate->setEnabled(!controller.busy() && controller.draftProfile() && controller.draftProfile()->homography.has_value());
    };
    connect(&controller,&WorkspaceCalibrationController::changed,this,refresh); refresh();
    connect(calculate,&QPushButton::clicked,this,[=,&controller] {
        if (!confirmed->isChecked()) { status->setText("Confirm independently measured geometry before estimating."); return; }
        ExtrinsicSolveInput input; input.revision=QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
        auto& s=input.survey; s.surfaceHeightMeters=surface->value(); s.boardThicknessMeters=board->value(); s.backingThicknessMeters=backing->value();
        s.maximumReprojectionPixels=pixelLimit->value(); s.maximumCheckAxisMeters=axisLimit->value(); s.measurementMethod=method->text().toStdString();
        std::istringstream points{references->toPlainText().toStdString()},mountText{mounts->toPlainText().toStdString()};
        points.imbue(std::locale::classic()); mountText.imbue(std::locale::classic());
        std::string line;
        while (std::getline(points,line)) {
            if (line.find_first_not_of(" \t\r")==std::string::npos) continue;
            std::istringstream row{line}; row.imbue(std::locale::classic()); std::string role; ExtrinsicReference p;
            if (!(row>>role>>p.image.u>>p.image.v>>p.world.x.value>>p.world.y.value) || (role!="Fit" && role!="Check")
                || !(row>>std::ws).eof() || s.references.size()>=MaximumExtrinsicReferences) { status->setText("Invalid reference row or capacity exceeded. Use Fit/Check u v x y."); return; }
            p.check=role=="Check"; s.references.push_back(p);
        }
        while (std::getline(mountText,line)) {
            if (line.find_first_not_of(" \t\r")==std::string::npos) continue;
            std::istringstream row{line}; row.imbue(std::locale::classic()); RobotMount m; std::uint64_t id;
            auto quoted=[&](std::string& value) { row>>std::ws; return row.peek()=='"' && bool(row>>std::quoted(value)); };
            if (!(row>>id>>m.tagHeightMeters>>m.bodyForwardMeters>>m.bodyLeftMeters>>m.bodyHeadingOffsetRadians)
                || id>std::numeric_limits<std::uint32_t>::max() || !quoted(m.revision) || !quoted(m.measurementMethod)
                || !(row>>std::ws).eof() || input.mounts.size()>=MaximumRobotMounts) { status->setText("Invalid mount row or capacity exceeded. Supply SI values, quoted revision and measurement method."); return; }
            m.robotId=RobotId{static_cast<std::uint32_t>(id)}; input.mounts.push_back(std::move(m));
        }
        controller.calculateLocalization(std::move(input));
    });
}
} // namespace micro_robot
