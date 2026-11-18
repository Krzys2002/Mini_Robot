#include "ui/ReferenceMarkersDialog.hpp"
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QVBoxLayout>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
namespace micro_robot {
ReferenceMarkersDialog::ReferenceMarkersDialog(WorkspaceCalibrationController& controller,QWidget* parent):QDialog(parent) {
    setWindowTitle("Surveyed references and placement monitoring"); resize(900,760);
    auto* outer=new QVBoxLayout(this); auto* scroll=new QScrollArea(this); scroll->setWidgetResizable(true);
    auto* body=new QWidget(scroll); auto* layout=new QVBoxLayout(body); scroll->setWidget(body); outer->addWidget(scroll);
    auto label=[&](const char* text) { auto* l=new QLabel(text,body); l->setWordWrap(true); l->setTextFormat(Qt::PlainText); layout->addWidget(l); return l; };
    label("First load/calculate measured height-aware geometry and robot mounts. Reference IDs are separate from robots (default robot tag IDs 0–15 are reserved). "
          "Surveyed references may lie outside travel bounds; Fit and independent Check marker centres must each surround the workspace. No automatic refit or frame movement occurs.");
    auto* form=new QFormLayout; layout->addLayout(form);
    auto integer=[&](const char* name,const char* title,int value,int minimum,int maximum) {
        auto* box=new QSpinBox(body); box->setObjectName(name); box->setRange(minimum,maximum); box->setValue(value); form->addRow(title,box); return box;
    };
    const auto maxMs=static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(MaximumReferenceCadence).count());
    ReferencePolicy initial;
    auto* cadence=integer("referenceCadence","Monitoring cadence (ms)",static_cast<int>(initial.cadence.count()),static_cast<int>(MinimumReferenceCadence.count()),maxMs);
    auto* age=integer("referenceMaximumAge","Maximum evidence age (ms)",static_cast<int>(initial.maximumEvidenceAge.count()),static_cast<int>(MinimumReferenceCadence.count()),maxMs);
    auto* duration=integer("referenceDriftDuration","Minimum drift persistence (ms)",static_cast<int>(initial.minimumDriftDuration.count()),static_cast<int>(MinimumReferenceCadence.count()),maxMs);
    auto* samples=integer("referenceDriftSamples","Consecutive drift samples",static_cast<int>(initial.consecutiveDriftSamples),static_cast<int>(MinimumReferenceDriftSamples),static_cast<int>(MaximumReferenceMarkers));
    auto real=[&](const char* name,const char* title,double value,double maximum) {
        auto* box=new QDoubleSpinBox(body); box->setObjectName(name); box->setDecimals(6); box->setRange(0,maximum); box->setValue(value); form->addRow(title,box); return box;
    };
    auto* pixels=real("referencePixelLimit","Maximum residual (px)",initial.maximumResidualPixels,MaximumSurveyPixelLimit);
    auto* metres=real("referenceAxisLimit","Maximum residual on each axis (m)",initial.maximumResidualAxisMeters,MaximumSurveyEntryMeters);
    auto* notes=new QLineEdit(body); notes->setObjectName("referenceSharedErrors"); notes->setMaxLength(static_cast<int>(MaximumGeometryNoteBytes));
    form->addRow("Survey uncertainty / shared-error limitations",notes);
    label("One row per tag36h11 reference (maximum 32). Use decimal points in pasted rows. Format: Fit|Check|Monitor id height_m black_edge_m standard_uncertainty_m "
          "independent_0_or_1 \"installation_revision\" \"survey_revision\" \"method/evidence_ID\" x0_m y0_m x1_m y1_m x2_m y2_m x3_m y3_m. "
          "Corners are measured decoded TL/TR/BR/BL, clockwise in world X/Y, at that marker's measured height. Checks require independent survey declaration. Preserve original measurement evidence.");
    auto* registry=new QPlainTextEdit(body); registry->setObjectName("referenceRegistryRows"); registry->setMinimumHeight(130); layout->addWidget(registry);
    auto* confirmed=new QCheckBox("I measured the reference geometry, uncertainty and revisions; Check surveys are independent.",body);
    confirmed->setObjectName("referenceSurveyConfirmed"); layout->addWidget(confirmed);
    label("Latest bounded detector snapshot (native raw and once-undistorted pixels); rejected and missing references remain explicit:");
    auto* evidence=new QPlainTextEdit(body); evidence->setObjectName("referenceEvidenceRows"); evidence->setReadOnly(true); evidence->setMinimumHeight(100); layout->addWidget(evidence);
    auto* status=label(""); status->setObjectName("referenceStatus");
    auto* actions=new QHBoxLayout; outer->addLayout(actions);
    auto button=[&](const char* text,const char* name) { auto* b=new QPushButton(text,this); b->setObjectName(name); actions->addWidget(b); return b; };
    auto* apply=button("Apply registry draft","applyReferenceRegistry"); auto* fit=button("Fit latest references","fitReferenceRegistry"); auto* close=button("Close","closeReferences");
    if(controller.draftProfile() && controller.draftProfile()->references) {
        const auto& c=*controller.draftProfile()->references; const auto& p=c.policy;
        cadence->setValue(static_cast<int>(p.cadence.count())); age->setValue(static_cast<int>(p.maximumEvidenceAge.count())); duration->setValue(static_cast<int>(p.minimumDriftDuration.count()));
        samples->setValue(static_cast<int>(p.consecutiveDriftSamples)); pixels->setValue(p.maximumResidualPixels); metres->setValue(p.maximumResidualAxisMeters); notes->setText(QString::fromStdString(c.sharedErrorNotes));
        std::ostringstream out; out.imbue(std::locale::classic()); out<<std::setprecision(std::numeric_limits<double>::max_digits10);
        for(const auto& m:c.markers) {
            out<<(m.role==ReferenceRole::Fit?"Fit":m.role==ReferenceRole::IndependentCheck?"Check":"Monitor")<<' '<<m.id<<' '
                <<m.surfaceHeightMeters<<' '<<m.blackEdgeMeters<<' '<<m.standardUncertaintyMeters<<' '<<m.independentlySurveyed<<' '
                <<std::quoted(m.installationRevision)<<' '<<std::quoted(m.surveyRevision)<<' '<<std::quoted(m.method);
            for(auto p:m.corners) out<<' '<<p.x.value<<' '<<p.y.value;
            out<<'\n';
        }
        registry->setPlainText(QString::fromStdString(out.str()));
    }
    connect(registry,&QPlainTextEdit::textChanged,this,[confirmed]{confirmed->setChecked(false);});
    connect(notes,&QLineEdit::textChanged,this,[confirmed]{confirmed->setChecked(false);});
    connect(apply,&QPushButton::clicked,this,[=,&controller] {
        if(!confirmed->isChecked()) { status->setText("Confirm measured survey data before applying a registry."); return; }
        ReferenceConfiguration c; c.sharedErrorNotes=notes->text().toStdString();
        c.policy={std::chrono::milliseconds{cadence->value()},std::chrono::milliseconds{age->value()},std::chrono::milliseconds{duration->value()},
            static_cast<std::size_t>(samples->value()),pixels->value(),metres->value()};
        std::istringstream rows{registry->toPlainText().toStdString()}; rows.imbue(std::locale::classic()); std::string line;
        while(std::getline(rows,line)) {
            if(line.find_first_not_of(" \t\r")==std::string::npos) continue;
            SurveyedReferenceMarker m; std::string role; int flag;
            std::istringstream row{line}; row.imbue(std::locale::classic());
            auto quoted=[&](std::string& s){row>>std::ws; return row.peek()=='"' && bool(row>>std::quoted(s));};
            bool valid=bool(row>>role>>m.id>>m.surfaceHeightMeters>>m.blackEdgeMeters>>m.standardUncertaintyMeters>>flag)
                && (role=="Fit" || role=="Check" || role=="Monitor") && (flag==0 || flag==1)
                && quoted(m.installationRevision) && quoted(m.surveyRevision) && quoted(m.method);
            for(auto& p:m.corners) valid=valid && bool(row>>p.x.value>>p.y.value);
            if(!valid || !(row>>std::ws).eof() || c.markers.size()>=MaximumReferenceMarkers) { status->setText("Invalid reference row/capacity. Supply every measured field and four ordered corner pairs; no rows are truncated."); return; }
            m.role=role=="Fit"?ReferenceRole::Fit:role=="Check"?ReferenceRole::IndependentCheck:ReferenceRole::DriftMonitor; m.independentlySurveyed=flag;
            c.markers.push_back(std::move(m));
        }
        controller.configureReferences(std::move(c));
    });
    connect(fit,&QPushButton::clicked,&controller,&WorkspaceCalibrationController::calculateReferences);
    connect(close,&QPushButton::clicked,this,&QDialog::close);
    auto refresh=[=,&controller] {
        status->setText(controller.message()); apply->setEnabled(!controller.busy()); fit->setEnabled(!controller.busy() && bool(controller.latestReferenceFrame()));
        std::ostringstream text; text.imbue(std::locale::classic());
        if(const auto frame=controller.latestReferenceFrame()) for(const auto& row:frame->observations) {
            static constexpr const char* reasons[]{"Included","Missing/occluded","Duplicate identity","Invalid corners","Unsupported ray/image"};
            text<<row.id<<": "<<reasons[static_cast<int>(row.rejection)]<<" raw:";
            for(auto p:row.raw) text<<' '<<p.u<<','<<p.v;
            text<<" undistorted:"; for(auto p:row.undistorted) text<<' '<<p.u<<','<<p.v;
            text<<'\n';
        }
        evidence->setPlainText(QString::fromStdString(text.str()));
    };
    connect(&controller,&WorkspaceCalibrationController::changed,this,refresh); refresh();
}
} // namespace micro_robot
