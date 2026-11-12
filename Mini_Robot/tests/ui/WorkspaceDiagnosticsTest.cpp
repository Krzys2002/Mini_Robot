#include "../calibration/WorkspaceDiagnosticsFixtures.hpp"
#include "app/WorkspaceDiagnosticsController.hpp"
#include "calibration/WorkspaceProfile.hpp"
#include "ui/WorkspaceDiagnosticsDialog.hpp"
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableView>
#include <QTemporaryDir>
#include <QTest>
#include <atomic>
#include <condition_variable>
#include <cstdlib>
#include <gtest/gtest.h>
#include <mutex>

namespace
{
using namespace micro_robot;
using namespace micro_robot::test;
bool until(const std::function<bool()>& done)
{
    for (int i = 0; i < 500 && !done(); ++i) {
        QApplication::processEvents();
        QTest::qWait(2);
    }
    return done();
}
struct Gate {
    std::mutex mutex;
    std::condition_variable cv;
    bool released{};
    std::atomic<int> calls{};
    void release()
    {
        std::lock_guard lock(mutex);
        released = true;
        cv.notify_all();
    }
    WorkspaceDiagnosticsOutcome build(WorkspaceCalibrationProfile p,
                                      std::shared_ptr<const WorkspaceAcquisitionRecord> a,
                                      WorkspaceSurveyEvidence e, std::size_t x, std::size_t y)
    {
        if (++calls == 1) {
            std::unique_lock lock(mutex);
            cv.wait(lock, [&] { return released; });
        }
        return buildWorkspaceDiagnostics(std::move(p), std::move(a), std::move(e), x, y);
    }
};
TEST(WorkspaceDiagnosticsUiTest, LatestRequestReplacesPendingAndRejectsStaleResults)
{
    auto gate = std::make_shared<Gate>();
    WorkspaceDiagnosticsController c(
        nullptr, [gate](auto p, auto a, auto e, auto x, auto y) { return gate->build(p, a, e, x, y); });
    c.setCalibration(diagnosticsProfile());
    bool entered = until([&] { return gate->calls == 1; });
    for (int i = 0; i < 100; ++i)
        c.request({}, 2 + static_cast<std::size_t>(i % 64), 3);
    c.request({}, 7, 9);
    EXPECT_FALSE(c.snapshot());
    gate->release();
    ASSERT_TRUE(entered);
    ASSERT_TRUE(until([&] { return !c.busy(); }));
    ASSERT_TRUE(c.snapshot());
    EXPECT_EQ(c.snapshot()->columns, 7u);
    EXPECT_EQ(c.snapshot()->rows, 9u);
    EXPECT_EQ(gate->calls, 2);
    c.setCalibration({});
    EXPECT_FALSE(c.snapshot());
    EXPECT_FALSE(c.profile());
    c.shutdown();
    EXPECT_TRUE(until([&] { return c.isStopped(); }));
}
TEST(WorkspaceDiagnosticsUiTest, CalibrationInvalidationDiscardsInflightSnapshot)
{
    auto gate = std::make_shared<Gate>();
    WorkspaceDiagnosticsController c(
        nullptr, [gate](auto p, auto a, auto e, auto x, auto y) { return gate->build(p, a, e, x, y); });
    c.setCalibration(diagnosticsProfile());
    bool entered = until([&] { return gate->calls == 1; });
    c.setCalibration({});
    gate->release();
    ASSERT_TRUE(entered);
    ASSERT_TRUE(until([&] { return !c.busy(); }));
    EXPECT_FALSE(c.snapshot());
    EXPECT_FALSE(c.profile());
    auto p = diagnosticsProfile();
    p.frameId = "replacement";
    p.intrinsics.cameraMatrix[0] += 1;
    c.setCalibration(p);
    ASSERT_TRUE(until([&] { return !c.busy(); }));
    ASSERT_TRUE(c.snapshot());
    EXPECT_EQ(c.snapshot()->profile.frameId, "replacement");
    EXPECT_EQ(c.snapshot()->profile.intrinsics.cameraMatrix[0], 801);
}
TEST(WorkspaceDiagnosticsUiTest, ExportRetainsSamplesIdentityEvidenceAndNullMasks)
{
    auto a = diagnosticsAcquisition(0.001);
    WorkspaceDiagnosticsController c;
    c.setCalibration(a->profile, a);
    c.request(diagnosticsEvidence(*a), 7, 5);
    ASSERT_TRUE(until([&] { return !c.busy(); }));
    ASSERT_TRUE(c.snapshot());
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const auto path = dir.filePath("map.json");
    c.exportSnapshot(path);
    ASSERT_TRUE(until([&] { return !c.busy(); }));
    QFile file(path);
    ASSERT_TRUE(file.open(QIODevice::ReadOnly));
    auto object = QJsonDocument::fromJson(file.readAll()).object();
    EXPECT_EQ(object["schema"].toString(), "mini-robot-workspace-diagnostics-v1");
    EXPECT_EQ(object["samples"].toArray().size(), 35);
    EXPECT_EQ(object["observations"].toArray().size(), 13);
    EXPECT_TRUE(object["samples"].toArray()[0].toObject()["predictionRadiusMeters"].isNull());
    EXPECT_EQ(object["surveyEvidence"].toObject()["referenceIds"].toArray().size(), 9);
    EXPECT_FALSE(object["prediction"].toObject().isEmpty());
    auto profile = parseWorkspaceProfile(object["profileCanonical"].toString().toStdString());
    ASSERT_TRUE(profile.profile);
    EXPECT_EQ(profile.profile->acquisitionId, a->id);
    auto acquisition = parseWorkspaceAcquisition(object["acquisitionCanonical"].toString().toStdString());
    ASSERT_TRUE(acquisition.record) << acquisition.error;
    EXPECT_EQ(acquisition.record->references.size(), a->references.size());
    c.exportSnapshot(dir.filePath("missing/map.json"));
    ASSERT_TRUE(until([&] { return !c.busy(); }));
    EXPECT_TRUE(c.message().contains("failed"));
    EXPECT_TRUE(QFile::exists(path));
}
TEST(WorkspaceDiagnosticsUiTest, EvidenceControlsAreExplicitAndResetOnCalibrationChange)
{
    auto a = diagnosticsAcquisition(0.001);
    WorkspaceDiagnosticsController c;
    c.setCalibration(a->profile, a);
    ASSERT_TRUE(until([&] { return !c.busy(); }));
    WorkspaceDiagnosticsDialog dialog(c);
    dialog.show();
    QApplication::processEvents();
    auto* table = dialog.findChild<QTableView*>("surveyReferences");
    ASSERT_NE(table, nullptr);
    auto* model = table->model();
    EXPECT_EQ(model->rowCount(), 13);
    EXPECT_FALSE(model->setData(model->index(0, 0), Qt::Checked, Qt::CheckStateRole));
    for (int row = 4; row < 13; ++row)
        ASSERT_TRUE(model->setData(model->index(row, 0), Qt::Checked, Qt::CheckStateRole));
    dialog.findChild<QCheckBox*>("surveyIndependent")->setChecked(true);
    dialog.findChild<QCheckBox*>("surveyAssumptions")->setChecked(true);
    dialog.findChild<QDoubleSpinBox*>("surveyFloor")->setValue(0.5);
    dialog.findChild<QLineEdit*>("surveyMethod")
        ->setText(
            "Synthetic independent survey; Gaussian affine residual model and positive per-axis floor.");
    dialog.findChild<QPushButton*>("mapCompute")->click();
    ASSERT_TRUE(until([&] { return !c.busy(); }));
    ASSERT_TRUE(c.snapshot());
    ASSERT_TRUE(c.snapshot()->prediction) << c.snapshot()->uncertaintyUnavailable;
    auto* modes = dialog.findChild<QComboBox*>("mapMode");
    ASSERT_NE(modes, nullptr);
    for (int i = 0; i < 5; ++i) {
        modes->setCurrentIndex(i);
        QApplication::processEvents();
        EXPECT_FALSE(dialog.grab().isNull());
    }
    if (const auto* screenshot = std::getenv("MICRO_ROBOT_MAP_SCREENSHOT"))
        EXPECT_TRUE(dialog.grab().save(QString::fromLocal8Bit(screenshot)));
    c.setCalibration({});
    EXPECT_EQ(model->rowCount(), 0);
    EXPECT_FALSE(dialog.findChild<QCheckBox*>("surveyIndependent")->isChecked());
    EXPECT_TRUE(dialog.findChild<QLineEdit*>("surveyMethod")->text().isEmpty());
    EXPECT_FALSE(dialog.findChild<QPushButton*>("mapExport")->isEnabled());
}
TEST(WorkspaceDiagnosticsUiTest, SurveyTableHandlesFullCapacityWithoutCellWidgets)
{
    WorkspaceSurveyModel model;
    auto a = diagnosticsAcquisition();
    // Presentation fixture only: ID/data capacity, not a geometrically valid duplicated acquisition.
    while (a->references.size() < MaximumWorkspaceReferences) {
        auto r = a->references.back();
        r.id = a->references.size() + 1;
        a->references.push_back(r);
    }
    model.setAcquisition(a);
    EXPECT_EQ(model.rowCount(), 8192);
    EXPECT_EQ(model.data(model.index(8191, 1)).toString(), "8192 / Check");
    QTableView view;
    view.setModel(&model);
    view.resize(900, 200);
    view.show();
    bool delivered = false;
    QMetaObject::invokeMethod(&view, [&] { delivered = true; }, Qt::QueuedConnection);
    EXPECT_TRUE(until([&] { return delivered; }));
}
TEST(WorkspaceDiagnosticsUiTest, WorkerExceptionsAndOversizedRequestsRemainRecoverable)
{
    WorkspaceDiagnosticsController c(nullptr,
                                     [](auto, auto, auto, auto, auto) -> WorkspaceDiagnosticsOutcome {
                                         throw std::runtime_error("test worker failure");
                                     });
    c.setCalibration(diagnosticsProfile());
    ASSERT_TRUE(until([&] { return !c.busy(); }));
    EXPECT_FALSE(c.snapshot());
    EXPECT_TRUE(c.message().contains("test worker failure"));
    c.request({}, 66, 2);
    EXPECT_FALSE(c.snapshot());
    EXPECT_TRUE(c.message().contains("capacity"));
    c.shutdown();
    ASSERT_TRUE(until([&] { return c.isStopped(); }));
    EXPECT_FALSE(c.profile());
}
} // namespace
