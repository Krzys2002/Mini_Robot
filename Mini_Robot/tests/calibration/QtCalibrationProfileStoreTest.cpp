#include "calibration/QtCalibrationProfileStore.hpp"
#include "IntrinsicFixtures.hpp"
#include <QDir>
#include <QFile>
#include <QTemporaryDir>
#include <gtest/gtest.h>

namespace {
using namespace micro_robot;
using namespace micro_robot::test;
TEST(QtCalibrationProfileStoreTest, SurvivesStoreRecreationWithUnicodePathsAndSeparateCameraKeys)
{
    QTemporaryDir directory;
    ASSERT_TRUE(directory.isValid());
    const auto root = directory.path() + QStringLiteral("/profile żółć directory");
    auto a = validIntrinsicCalibration();
    a.sourceId = "../camera/Unicode żółć";
    auto b = a;
    b.sourceId = "camera-b";
    b.cameraMatrix[0] = 850;
    auto secondResolution = a;
    secondResolution.width = 1200;
    secondResolution.cameraMatrix[0] = 900;
    {
        QtCalibrationProfileStore writer{root};
        EXPECT_TRUE(writer.save(a).empty());
        EXPECT_TRUE(writer.save(b).empty());
        EXPECT_TRUE(writer.save(secondResolution).empty());
    }
    QtCalibrationProfileStore reader{root};
    auto loaded = reader.load(a.sourceId, a.width, a.height);
    ASSERT_EQ(loaded.state, ProfileLoadState::Loaded) << loaded.error;
    ASSERT_TRUE(loaded.calibration);
    EXPECT_EQ(loaded.calibration->cameraMatrix, a.cameraMatrix);
    loaded = reader.load(b.sourceId, b.width, b.height);
    ASSERT_EQ(loaded.state, ProfileLoadState::Loaded);
    EXPECT_EQ(loaded.calibration->cameraMatrix, b.cameraMatrix);
    loaded = reader.load(a.sourceId, 1200, 800);
    ASSERT_EQ(loaded.state, ProfileLoadState::Loaded);
    ASSERT_TRUE(loaded.calibration);
    EXPECT_EQ(loaded.calibration->cameraMatrix, secondResolution.cameraMatrix);
    EXPECT_EQ(reader.load(a.sourceId, 500, 400).state, ProfileLoadState::Missing);
    EXPECT_EQ(reader.load("camera-c", a.width, a.height).state, ProfileLoadState::Missing);
    EXPECT_EQ(QDir(root).entryList({QStringLiteral("*.intrinsics")}, QDir::Files).size(), 3);
}
TEST(QtCalibrationProfileStoreTest, AtomicallyReplacesSameKeyAndPreservesPreviousOnValidationFailure)
{
    QTemporaryDir directory;
    ASSERT_TRUE(directory.isValid());
    QtCalibrationProfileStore store{directory.path()};
    auto c = validIntrinsicCalibration();
    ASSERT_TRUE(store.save(c).empty());
    c.cameraMatrix[0] = 825;
    ASSERT_TRUE(store.save(c).empty());
    c.cameraMatrix[0] = -1;
    EXPECT_FALSE(store.save(c).empty());
    const auto loaded = store.load(c.sourceId, c.width, c.height);
    ASSERT_TRUE(loaded.calibration) << loaded.error;
    EXPECT_EQ(loaded.calibration->cameraMatrix[0], 825);
    EXPECT_EQ(QDir(directory.path()).entryList({QStringLiteral("*.intrinsics")}, QDir::Files).size(), 1);
}
TEST(QtCalibrationProfileStoreTest, RejectsTamperedIdentityResolutionAndOversizedOrCorruptFiles)
{
    QTemporaryDir directory;
    QtCalibrationProfileStore store{directory.path()};
    const auto original = validIntrinsicCalibration();
    ASSERT_TRUE(store.save(original).empty());
    const auto files = QDir(directory.path()).entryList({QStringLiteral("*.intrinsics")}, QDir::Files);
    ASSERT_EQ(files.size(), 1);
    auto overwrite = [&](const std::string& content) {
        QFile file{QDir(directory.path()).filePath(files.front())};
        ASSERT_TRUE(file.open(QIODevice::WriteOnly | QIODevice::Truncate));
        ASSERT_EQ(file.write(content.data(), static_cast<qint64>(content.size())), static_cast<qint64>(content.size()));
    };
    auto other = original;
    other.sourceId = "different-device";
    overwrite(serializeCalibrationProfile(other));
    EXPECT_EQ(store.load(original.sourceId, 1000, 800).state, ProfileLoadState::Rejected);
    other = original;
    other.width = 1001;
    overwrite(serializeCalibrationProfile(other));
    EXPECT_EQ(store.load(original.sourceId, 1000, 800).state, ProfileLoadState::Rejected);
    overwrite("broken");
    EXPECT_EQ(store.load(original.sourceId, 1000, 800).state, ProfileLoadState::Rejected);
    overwrite(std::string(MaximumCalibrationProfileBytes + 1, 'x'));
    EXPECT_EQ(store.load(original.sourceId, 1000, 800).state, ProfileLoadState::Rejected);
}
TEST(QtCalibrationProfileStoreTest, ReportsUnwritableDirectoryWithoutClaimingSuccess)
{
    QTemporaryDir directory;
    QFile obstruction{directory.path() + QStringLiteral("/file")};
    ASSERT_TRUE(obstruction.open(QIODevice::WriteOnly));
    obstruction.close();
    QtCalibrationProfileStore store{obstruction.fileName() + QStringLiteral("/child")};
    EXPECT_FALSE(store.save(validIntrinsicCalibration()).empty());
}
} // namespace
