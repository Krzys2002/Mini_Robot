#include <QDateTime>
#include <QFileInfo>
#include "HomographyFixtures.hpp"
#include "calibration/QtWorkspaceProfileStore.hpp"
#include <QDir>
#include <QFile>
#include <QTemporaryDir>
#include <gtest/gtest.h>
namespace {
using namespace micro_robot;
using namespace micro_robot::test;
TEST(QtWorkspaceProfileStoreTest, SurvivesRecreationAndRejectsAnotherOpticalModel)
{
    QTemporaryDir directory; ASSERT_TRUE(directory.isValid());
    auto p = homographyInput().profile; p.workspaceId = "../table żółć"; p.homography = knownHomography();
    p.bounds = WorkspaceBounds{Meters{1},Meters{0.6},{Meters{-0.25},Meters{-0.3}}};
    p.frameId = "persisted-signed-frame";
    p.boundary = WorkspaceBoundary{{{100,700},{900,700},{900,100},{100,100}}};
    { QtWorkspaceProfileStore writer{directory.path()}; EXPECT_TRUE(writer.save(p).empty()); }
    QtWorkspaceProfileStore reader{directory.path()};
    auto result = reader.load(p.workspaceId, p.intrinsics); ASSERT_TRUE(result.profile) << result.error;
    EXPECT_EQ(result.profile->homography->imageToWorld, p.homography->imageToWorld);
    EXPECT_TRUE(result.profile->bounds == p.bounds);
    EXPECT_EQ(result.profile->frameId,p.frameId);
    ASSERT_TRUE(result.profile->boundary);
    EXPECT_DOUBLE_EQ((*result.profile->boundary)[2].v,100);
    auto optical = p.intrinsics; optical.cameraMatrix[0] += 1;
    EXPECT_EQ(reader.load(p.workspaceId, optical).state, ProfileLoadState::Rejected);
    optical = p.intrinsics; optical.sourceId += "other";
    EXPECT_EQ(reader.load(p.workspaceId, optical).state, ProfileLoadState::Missing);
    EXPECT_EQ(reader.load("other table", p.intrinsics).state, ProfileLoadState::Missing);
    p.homography = WorkspaceHomography{}; EXPECT_FALSE(reader.save(p).empty());
    EXPECT_TRUE(reader.load(p.workspaceId, p.intrinsics).profile); // failed save preserves old file
}
TEST(QtWorkspaceProfileStoreTest, CorruptOversizedAndUnwritableFilesAreExplicitFailures)
{
    QTemporaryDir directory; auto p = homographyInput().profile; p.homography = knownHomography();
    QtWorkspaceProfileStore store{directory.path()}; ASSERT_TRUE(store.save(p).empty());
    const auto files = QDir(directory.path()).entryList({"*.workspace"}, QDir::Files); ASSERT_EQ(files.size(), 1);
    QFile file{directory.filePath(files.front())}; ASSERT_TRUE(file.open(QIODevice::WriteOnly));
    file.write(QByteArray(static_cast<qsizetype>(MaximumWorkspaceProfileBytes + 1), 'x')); file.close();
    EXPECT_EQ(store.load(p.workspaceId, p.intrinsics).state, ProfileLoadState::Rejected);
    QtWorkspaceProfileStore invalid{file.fileName() + "/nested"}; EXPECT_FALSE(invalid.save(p).empty());
}
}

TEST(QtWorkspaceProfileStoreTest, AcquisitionSidecarIsImmutableBoundAndRequiredOnLoad)
{
    QTemporaryDir directory; ASSERT_TRUE(directory.isValid());
    auto input=homographyInput(); auto p=input.profile; p.homography=knownHomography(); p.acquisitionId="record-248";
    WorkspaceAcquisitionRecord record{p.acquisitionId,p,{},{}};
    std::uint64_t next=1;
    for (const auto& pair:input.fitPoints) record.references.push_back({next++,pair,false,false,{}});
    for (const auto& pair:input.checkPoints) record.references.push_back({next++,pair,true,false,{}});
    QtWorkspaceProfileStore store{directory.path()};
    EXPECT_FALSE(store.save(p).empty()); // Never publish a dangling record link.
    ASSERT_TRUE(store.saveAcquisition(p,record).empty());
    auto restored=store.load(p.workspaceId,p.intrinsics); ASSERT_TRUE(restored.profile)<<restored.error;
    ASSERT_TRUE(restored.acquisition); EXPECT_EQ(restored.acquisition->references.size(),5);
    EXPECT_EQ(restored.profile->acquisitionId,record.id);
    auto changed=record; changed.references.back().point.world.x.value+=0.02;
    EXPECT_FALSE(store.saveAcquisition(p,changed).empty()); // Same identity cannot be overwritten.
    restored=store.load(p.workspaceId,p.intrinsics); ASSERT_TRUE(restored.acquisition);
    EXPECT_DOUBLE_EQ(restored.acquisition->references.back().point.world.x.value,0.5);
    auto wrong=p; wrong.frameId="other-frame";
    EXPECT_FALSE(store.saveAcquisition(wrong,record).empty());
    EXPECT_FALSE(store.save(wrong).empty());
    restored=store.load(p.workspaceId,p.intrinsics); ASSERT_TRUE(restored.profile);
    EXPECT_EQ(restored.profile->frameId,p.frameId); // Failed writes preserve the previous pointer and snapshot.
    const auto files=QDir(directory.path()).entryList({"*.acquisition"},QDir::Files); ASSERT_EQ(files.size(),1);
    ASSERT_TRUE(QFile::remove(directory.filePath(files.front())));
    restored=store.load(p.workspaceId,p.intrinsics);
    EXPECT_EQ(restored.state,ProfileLoadState::Rejected); EXPECT_FALSE(restored.profile); EXPECT_FALSE(restored.acquisition);
}

namespace {
WorkspaceAcquisitionRecord storedBoardRecord(bool measured)
{
    auto input=homographyInput(); auto p=input.profile; p.homography=knownHomography(); p.acquisitionId="store-board-record";
    WorkspaceAcquisitionRecord record{p.acquisitionId,p,{},{}};
    std::uint64_t id=1;
    for (const auto& pair:input.fitPoints) record.references.push_back({id++,pair,false,false,{}});
    WorkspaceBoardPlacement board; board.id=1; board.anchor={Meters{0.2},Meters{0.1}};
    board.columnAngle=Radians{0}; board.rowDirection=1; board.note="surveyed board and backing";
    if (measured) {
        board.surfaceElevationMeters=0.004; board.boardThicknessMeters=0.003;
        board.backingThicknessMeters=0.001; board.elevationMethod="caliper from tabletop survey A";
    }
    for(int row=0;row<WorkspaceBoardTarget.rows;++row) for(int col=0;col<WorkspaceBoardTarget.columns;++col) {
        board.detected.push_back({100+800*(0.2+col*0.025),700-1000*(0.1+row*0.025)});
        board.included.push_back(true); board.reviewRejections.push_back(WorkspaceReferenceRejection::None);
    }
    const auto generated=generateWorkspaceBoard(board);
    for(std::size_t i=0;i<generated.points.size();++i)
        record.references.push_back({id++,generated.points[i],false,false,WorkspaceBoardSource{1,i}});
    for (const auto& pair:input.checkPoints) record.references.push_back({id++,pair,true,false,{}});
    record.placements.push_back(board);
    return record;
}
}
TEST(QtWorkspaceProfileStoreTest, UnchangedLegacyV1LoadSaveReloadPreservesOriginalEvidenceBytesAndTimestamp)
{
    QTemporaryDir directory; ASSERT_TRUE(directory.isValid());
    auto record=storedBoardRecord(false); QtWorkspaceProfileStore store{directory.path()};
    ASSERT_TRUE(store.saveAcquisition(record.profile,record).empty());
    const auto files=QDir(directory.path()).entryList({"*.acquisition"},QDir::Files); ASSERT_EQ(files.size(),1);
    auto legacy=serializeWorkspaceAcquisition(record);
    legacy.replace(legacy.find("acquisition 2"),std::string("acquisition 2").size(),"acquisition 1");
    const auto at=legacy.find(" elevation "); ASSERT_NE(at,std::string::npos);
    legacy.erase(at,legacy.find('\n',at)-at);
    QFile sidecar{directory.filePath(files.front())};
    ASSERT_TRUE(sidecar.open(QIODevice::WriteOnly));
    ASSERT_EQ(sidecar.write(QByteArray::fromStdString(legacy)),static_cast<qint64>(legacy.size()));
    const auto timestamp=QDateTime::fromSecsSinceEpoch(946684800);
    ASSERT_TRUE(sidecar.flush());
    ASSERT_TRUE(sidecar.setFileTime(timestamp,QFileDevice::FileModificationTime)); sidecar.close();
    auto loaded=store.load(record.profile.workspaceId,record.profile.intrinsics);
    ASSERT_TRUE(loaded.profile)<<loaded.error; ASSERT_TRUE(loaded.acquisition);
    EXPECT_FALSE(loaded.acquisition->placements.front().surfaceElevationMeters);
    ASSERT_TRUE(store.saveAcquisition(*loaded.profile,*loaded.acquisition).empty());
    QtWorkspaceProfileStore recreated{directory.path()};
    const auto reloaded=recreated.load(record.profile.workspaceId,record.profile.intrinsics);
    ASSERT_TRUE(reloaded.acquisition)<<reloaded.error;
    EXPECT_FALSE(reloaded.acquisition->placements.front().boardThicknessMeters);
    EXPECT_EQ(serializeWorkspaceAcquisition(*reloaded.acquisition),serializeWorkspaceAcquisition(record));
    ASSERT_TRUE(sidecar.open(QIODevice::ReadOnly)); EXPECT_EQ(sidecar.readAll(),QByteArray::fromStdString(legacy)); sidecar.close();
    EXPECT_EQ(QFileInfo(sidecar).lastModified(),timestamp);
    auto changed=*loaded.acquisition; changed.placements.front().surfaceElevationMeters=0.004;
    changed.placements.front().elevationMethod="new measured height";
    EXPECT_FALSE(store.saveAcquisition(*loaded.profile,changed).empty());
    ASSERT_TRUE(sidecar.open(QIODevice::ReadOnly)); EXPECT_EQ(sidecar.readAll(),QByteArray::fromStdString(legacy));
}
TEST(QtWorkspaceProfileStoreTest, MeasuredElevationThicknessAndProvenanceSurviveStoreRecreation)
{
    QTemporaryDir directory; auto record=storedBoardRecord(true);
    { QtWorkspaceProfileStore writer{directory.path()}; ASSERT_TRUE(writer.saveAcquisition(record.profile,record).empty()); }
    QtWorkspaceProfileStore reader{directory.path()}; const auto loaded=reader.load(record.profile.workspaceId,record.profile.intrinsics);
    ASSERT_TRUE(loaded.acquisition)<<loaded.error;
    const auto& board=loaded.acquisition->placements.front();
    EXPECT_EQ(board.surfaceElevationMeters,0.004); EXPECT_EQ(board.boardThicknessMeters,0.003);
    EXPECT_EQ(board.backingThicknessMeters,0.001); EXPECT_EQ(board.elevationMethod,"caliper from tabletop survey A");
    EXPECT_EQ(serializeWorkspaceAcquisition(*loaded.acquisition),serializeWorkspaceAcquisition(record));
}
