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
