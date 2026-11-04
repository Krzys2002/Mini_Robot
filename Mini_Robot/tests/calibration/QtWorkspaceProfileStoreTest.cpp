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
    { QtWorkspaceProfileStore writer{directory.path()}; EXPECT_TRUE(writer.save(p).empty()); }
    QtWorkspaceProfileStore reader{directory.path()};
    auto result = reader.load(p.workspaceId, p.intrinsics); ASSERT_TRUE(result.profile) << result.error;
    EXPECT_EQ(result.profile->homography->imageToWorld, p.homography->imageToWorld);
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
