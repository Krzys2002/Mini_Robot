#include "infrastructure/configuration/ConfigLoader.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace {

using namespace micro_robot;
using namespace std::chrono_literals;

class CapturingSink final : public ILogSink {
public:
    void write(const LogRecord& record) override { records.push_back(record); }
    std::vector<LogRecord> records;
};

void expectDefaults(const ApplicationConfig& config)
{
    const ApplicationConfig defaults;
    EXPECT_EQ(config.logging.minimumLevel, defaults.logging.minimumLevel);
    EXPECT_EQ(config.logging.fileEnabled, defaults.logging.fileEnabled);
    EXPECT_EQ(config.logging.filePath, defaults.logging.filePath);
    EXPECT_EQ(config.robot.id, defaults.robot.id);
    EXPECT_EQ(config.camera.capture.resolution, defaults.camera.capture.resolution);
    EXPECT_EQ(config.simulation.updateInterval, defaults.simulation.updateInterval);
    EXPECT_EQ(config.simulation.cameraFrameInterval, defaults.simulation.cameraFrameInterval);
    EXPECT_EQ(config.simulation.telemetryInterval, defaults.simulation.telemetryInterval);
}

// Asserts exactly one warning whose message contains every fragment.
void expectSingleWarning(const ConfigParseResult& result, const std::vector<std::string>& fragments)
{
    ASSERT_EQ(result.diagnostics.size(), 1U);
    EXPECT_EQ(result.diagnostics.front().level, LogLevel::Warning);
    for (const auto& fragment : fragments) {
        EXPECT_NE(result.diagnostics.front().message.find(fragment), std::string::npos)
            << "missing '" << fragment << "' in: " << result.diagnostics.front().message;
    }
}

TEST(ConfigDefaultsTest, MatchCurrentApplicationBehavior)
{
    const ApplicationConfig config;
    EXPECT_EQ(config.logging.minimumLevel, LogLevel::Info);
    EXPECT_TRUE(config.logging.fileEnabled);
    EXPECT_TRUE(config.logging.filePath.empty());
    EXPECT_EQ(config.robot.id, RobotId{1});
    EXPECT_FALSE(config.camera.capture.resolution);
    EXPECT_EQ(config.simulation.updateInterval, 100ms);
    EXPECT_EQ(config.simulation.cameraFrameInterval, 33ms);
    EXPECT_EQ(config.simulation.telemetryInterval, 0ms);
}

TEST(ConfigParseTest, EmptyTextYieldsDefaultsWithoutDiagnostics)
{
    const auto result = parseConfig("", "test.ini");
    EXPECT_TRUE(result.diagnostics.empty());
    expectDefaults(result.config);
}

TEST(ConfigParseTest, ReadsEverySetting)
{
    const auto result = parseConfig(
        "[logging]\n"
        "level = debug\n"
        "file_enabled = false\n"
        "file_path = /tmp/robot logs/run.log\n"
        "[robot]\n"
        "id = 42\n"
        "[camera]\n"
        "resolution = 1280x720\n"
        "[simulation]\n"
        "update_interval_ms = 20\n"
        "camera_frame_interval_ms = 50\n"
        "telemetry_interval_ms = 250\n",
        "test.ini");

    EXPECT_TRUE(result.diagnostics.empty());
    EXPECT_EQ(result.config.logging.minimumLevel, LogLevel::Debug);
    EXPECT_FALSE(result.config.logging.fileEnabled);
    EXPECT_EQ(result.config.logging.filePath, "/tmp/robot logs/run.log");
    EXPECT_EQ(result.config.robot.id, RobotId{42});
    EXPECT_EQ(result.config.camera.capture.resolution, (CameraResolution{1280, 720}));
    EXPECT_EQ(result.config.simulation.updateInterval, 20ms);
    EXPECT_EQ(result.config.simulation.cameraFrameInterval, 50ms);
    EXPECT_EQ(result.config.simulation.telemetryInterval, 250ms);
}

TEST(ConfigParseTest, IgnoresCommentsBlankLinesWhitespaceBomAndCrLf)
{
    const auto result = parseConfig(
        "\xEF\xBB\xBF# comment\r\n"
        "\r\n"
        "  ; another comment\r\n"
        "  [ robot ]  \r\n"
        "\tid=7\t\r\n",
        "test.ini");

    EXPECT_TRUE(result.diagnostics.empty());
    EXPECT_EQ(result.config.robot.id, RobotId{7});
}

TEST(ConfigParseTest, EnumerationAndBooleanValuesAreCaseInsensitive)
{
    const auto result = parseConfig("[logging]\nlevel = WARNING\nfile_enabled = Off\n", "test.ini");
    EXPECT_TRUE(result.diagnostics.empty());
    EXPECT_EQ(result.config.logging.minimumLevel, LogLevel::Warning);
    EXPECT_FALSE(result.config.logging.fileEnabled);
}

TEST(ConfigParseTest, AcceptsBoundaryValues)
{
    const auto result = parseConfig(
        "[robot]\nid = 4294967295\n"
        "[simulation]\nupdate_interval_ms = 1\ncamera_frame_interval_ms = 10000\n"
        "telemetry_interval_ms = 0\n",
        "test.ini");
    EXPECT_TRUE(result.diagnostics.empty());
    EXPECT_EQ(result.config.robot.id, RobotId{4294967295U});
    EXPECT_EQ(result.config.simulation.updateInterval, 1ms);
    EXPECT_EQ(result.config.simulation.cameraFrameInterval, 10000ms);
    EXPECT_EQ(result.config.simulation.telemetryInterval, 0ms);

    const auto upper = parseConfig("[simulation]\ntelemetry_interval_ms = 10000\n", "test.ini");
    EXPECT_TRUE(upper.diagnostics.empty());
    EXPECT_EQ(upper.config.simulation.telemetryInterval, 10000ms);
}

TEST(ConfigParseTest, CameraResolutionAcceptsAutoAndBoundarySizes)
{
    const auto bounds = parseConfig("[camera]\nresolution = 1x8192\n", "test.ini");
    EXPECT_TRUE(bounds.diagnostics.empty());
    EXPECT_EQ(bounds.config.camera.capture.resolution, (CameraResolution{1, 8192}));

    const auto spaced = parseConfig("[camera]\nresolution = 640 X 480\n", "test.ini");
    EXPECT_TRUE(spaced.diagnostics.empty());
    EXPECT_EQ(spaced.config.camera.capture.resolution, (CameraResolution{640, 480}));

    // A later "auto" restores the device default.
    const auto reset = parseConfig("[camera]\nresolution = 640x480\nresolution = AUTO\n", "test.ini");
    ASSERT_EQ(reset.diagnostics.size(), 1U); // Duplicate-key notice only.
    EXPECT_FALSE(reset.config.camera.capture.resolution);
}

TEST(ConfigParseTest, CameraResolutionRejectsMalformedAndOutOfRangeSizes)
{
    for (const auto* value : {"1280", "1280x", "x720", "0x720", "1280x0", "8193x720", "-1x720",
                              "1280x720x3", "1280*720", "hd"}) {
        const auto result = parseConfig("[camera]\nresolution = 1920x1080\nresolution = "
                                        + std::string{value} + "\n", "test.ini");
        ASSERT_EQ(result.diagnostics.size(), 2U) << value;
        const auto& message = result.diagnostics.back().message;
        EXPECT_NE(message.find("camera.resolution"), std::string::npos) << message;
        EXPECT_NE(message.find("keeping 1920x1080"), std::string::npos) << message;
        EXPECT_EQ(result.config.camera.capture.resolution, (CameraResolution{1920, 1080})) << value;
    }
}

TEST(ConfigParseTest, EmptyFilePathSelectsDefaultLocation)
{
    const auto result = parseConfig("[logging]\nfile_path =\n", "test.ini");
    EXPECT_TRUE(result.diagnostics.empty());
    EXPECT_TRUE(result.config.logging.filePath.empty());
}

TEST(ConfigParseTest, InvalidValueKeepsDefaultAndExplainsProblem)
{
    const auto result = parseConfig("[robot]\n\nid = abc\n", "settings.ini");
    expectSingleWarning(result, {"settings.ini:3:", "'abc'", "robot.id", "expected an integer",
                                 "keeping 1"});
    EXPECT_EQ(result.config.robot.id, RobotId{1});
}

TEST(ConfigParseTest, RejectsOutOfRangeAndMalformedNumbers)
{
    for (const auto* value : {"0", "10001", "-5", "+5", "12ms", "1.5", ""}) {
        SCOPED_TRACE(value);
        const auto result = parseConfig(
            std::string{"[simulation]\nupdate_interval_ms = "} + value + "\n", "test.ini");
        expectSingleWarning(result, {"simulation.update_interval_ms", "1 to 10000", "keeping 100"});
        EXPECT_EQ(result.config.simulation.updateInterval, 100ms);
    }
    for (const auto* value : {"-1", "10001", "x"}) {
        SCOPED_TRACE(value);
        const auto result = parseConfig(
            std::string{"[simulation]\ntelemetry_interval_ms = "} + value + "\n", "test.ini");
        expectSingleWarning(result, {"simulation.telemetry_interval_ms", "0 to 10000", "keeping 0"});
        EXPECT_EQ(result.config.simulation.telemetryInterval, 0ms);
    }
    const auto overflow = parseConfig("[robot]\nid = 4294967296\n", "test.ini");
    expectSingleWarning(overflow, {"robot.id"});
}

TEST(ConfigParseTest, RejectsUnknownLevelAndBoolean)
{
    const auto result = parseConfig("[logging]\nlevel = verbose\nfile_enabled = maybe\n", "test.ini");
    ASSERT_EQ(result.diagnostics.size(), 2U);
    EXPECT_NE(result.diagnostics[0].message.find("debug, info, warning, error"), std::string::npos);
    EXPECT_NE(result.diagnostics[1].message.find("true or false"), std::string::npos);
    expectDefaults(result.config);
}

TEST(ConfigParseTest, ValidEntriesAreAppliedDespiteOtherProblems)
{
    const auto result = parseConfig(
        "[robot]\n"
        "id = nope\n"
        "this line is malformed\n"
        "[simulation]\n"
        "update_interval_ms = 25\n",
        "test.ini");
    EXPECT_EQ(result.diagnostics.size(), 2U);
    EXPECT_EQ(result.config.robot.id, RobotId{1});
    EXPECT_EQ(result.config.simulation.updateInterval, 25ms);
}

TEST(ConfigParseTest, ReportsMalformedLines)
{
    expectSingleWarning(parseConfig("[robot]\nid 5\n", "test.ini"),
                        {"test.ini:2:", "expected 'key = value'", "'id 5'"});
    expectSingleWarning(parseConfig("[robot\n", "test.ini"), {"test.ini:1:", "expected '[section]'"});
    expectSingleWarning(parseConfig("[]\n", "test.ini"), {"expected '[section]'"});
}

TEST(ConfigParseTest, ReportsSettingOutsideSection)
{
    const auto result = parseConfig("id = 5\n", "test.ini");
    expectSingleWarning(result, {"test.ini:1:", "'id'", "not inside a [section]"});
    EXPECT_EQ(result.config.robot.id, RobotId{1});
}

TEST(ConfigParseTest, ReportsUnknownSettingsToCatchTypos)
{
    const auto result = parseConfig("[robot]\nidd = 5\n", "test.ini");
    expectSingleWarning(result, {"test.ini:2:", "unknown setting 'robot.idd'"});
}

TEST(ConfigParseTest, KeysAreCaseSensitive)
{
    expectSingleWarning(parseConfig("[robot]\nID = 5\n", "test.ini"), {"unknown setting 'robot.ID'"});
}

TEST(ConfigParseTest, ReportsUnknownSectionOnceAndIgnoresItsEntries)
{
    const auto result = parseConfig("[calibration]\nboard = 7x5\nsquare_mm = 25\n[robot]\nid = 3\n", "test.ini");
    expectSingleWarning(result, {"test.ini:1:", "unknown section [calibration]"});
    EXPECT_EQ(result.config.robot.id, RobotId{3});
}

TEST(ConfigParseTest, DuplicateSettingIsReportedAndLastValidValueWins)
{
    const auto result = parseConfig("[robot]\nid = 2\nid = 3\n", "test.ini");
    expectSingleWarning(result, {"test.ini:3:", "duplicate setting 'robot.id'", "line 2"});
    EXPECT_EQ(result.config.robot.id, RobotId{3});

    const auto invalidLater = parseConfig("[robot]\nid = 2\nid = x\n", "test.ini");
    EXPECT_EQ(invalidLater.diagnostics.size(), 2U);
    EXPECT_EQ(invalidLater.config.robot.id, RobotId{2});
}

class ConfigFileTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        const auto* test = ::testing::UnitTest::GetInstance()->current_test_info();
        directory_ = std::filesystem::temp_directory_path()
            / (std::string{"micro_robot_configuration_"} + test->name());
        std::filesystem::remove_all(directory_);
        std::filesystem::create_directories(directory_);
    }
    void TearDown() override { std::filesystem::remove_all(directory_); }

    std::filesystem::path writeFile(const std::string& name, const std::string& contents)
    {
        const auto path = directory_ / name;
        std::ofstream{path, std::ios::binary} << contents;
        return path;
    }

    std::filesystem::path directory_;
};

TEST_F(ConfigFileTest, LoadsValidFile)
{
    const auto path = writeFile("micro_robot.ini", "[robot]\nid = 9\n");
    const auto result = loadConfigFile(path);
    EXPECT_EQ(result.status, ConfigFileStatus::Loaded);
    EXPECT_EQ(result.path, path);
    EXPECT_TRUE(result.diagnostics.empty());
    EXPECT_EQ(result.config.robot.id, RobotId{9});
}

TEST_F(ConfigFileTest, DiagnosticsNameTheFile)
{
    const auto path = writeFile("micro_robot.ini", "[robot]\nid = -1\n");
    const auto result = loadConfigFile(path);
    EXPECT_EQ(result.status, ConfigFileStatus::Loaded);
    ASSERT_EQ(result.diagnostics.size(), 1U);
    EXPECT_NE(result.diagnostics[0].message.find("micro_robot.ini:2:"), std::string::npos);
}

TEST_F(ConfigFileTest, MissingOptionalFileSilentlyUsesDefaults)
{
    const auto result = loadConfigFile(directory_ / "absent.ini");
    EXPECT_EQ(result.status, ConfigFileStatus::NotFound);
    EXPECT_TRUE(result.diagnostics.empty());
    expectDefaults(result.config);
}

TEST_F(ConfigFileTest, MissingRequiredFileIsAnError)
{
    const auto result = loadConfigFile(directory_ / "absent.ini", ConfigFilePolicy::Required);
    EXPECT_EQ(result.status, ConfigFileStatus::NotFound);
    ASSERT_EQ(result.diagnostics.size(), 1U);
    EXPECT_EQ(result.diagnostics[0].level, LogLevel::Error);
    EXPECT_NE(result.diagnostics[0].message.find("absent.ini does not exist"), std::string::npos);
    EXPECT_NE(result.diagnostics[0].message.find("using built-in defaults"), std::string::npos);
    expectDefaults(result.config);
}

TEST_F(ConfigFileTest, NonRegularFileIsUnreadable)
{
    const auto result = loadConfigFile(directory_);
    EXPECT_EQ(result.status, ConfigFileStatus::Unreadable);
    ASSERT_EQ(result.diagnostics.size(), 1U);
    EXPECT_EQ(result.diagnostics[0].level, LogLevel::Error);
    EXPECT_NE(result.diagnostics[0].message.find("not a regular file"), std::string::npos);
    expectDefaults(result.config);
}

// Paths are built from char8_t text so that the tests do not depend on the narrow
// execution or locale encoding of the host.
std::string utf8(std::u8string_view text) { return {text.begin(), text.end()}; }

// Restores the process working directory when a test changes it.
class WorkingDirectoryGuard {
public:
    explicit WorkingDirectoryGuard(const std::filesystem::path& directory)
        : previous_(std::filesystem::current_path())
    {
        std::filesystem::current_path(directory);
    }
    ~WorkingDirectoryGuard()
    {
        std::error_code ignored;
        std::filesystem::current_path(previous_, ignored);
    }
    WorkingDirectoryGuard(const WorkingDirectoryGuard&) = delete;
    WorkingDirectoryGuard& operator=(const WorkingDirectoryGuard&) = delete;

private:
    std::filesystem::path previous_;
};

TEST_F(ConfigFileTest, LoadsFileFromNonAsciiDirectoryWithSpaces)
{
    const auto directory = directory_ / std::filesystem::path{u8"robot config ü/Ω settings"};
    std::filesystem::create_directories(directory);
    const auto path = directory / std::filesystem::path{u8"micro robot é.ini"};
    std::ofstream{path, std::ios::binary} << "[robot]\nid = 5\nid = x\n";

    const auto result = loadConfigFile(path, ConfigFilePolicy::Required);
    EXPECT_EQ(result.status, ConfigFileStatus::Loaded);
    EXPECT_EQ(result.path, path);
    EXPECT_EQ(result.config.robot.id, RobotId{5});
    ASSERT_EQ(result.diagnostics.size(), 2U);
    // Diagnostics name the file in UTF-8 regardless of the native path encoding.
    EXPECT_NE(result.diagnostics[0].message.find(utf8(u8"Ω settings")), std::string::npos)
        << result.diagnostics[0].message;
    EXPECT_NE(result.diagnostics[0].message.find(utf8(u8"micro robot é.ini:3:")), std::string::npos)
        << result.diagnostics[0].message;
}

TEST_F(ConfigFileTest, MissingRequiredNonAsciiFileIsNamedInUtf8)
{
    const auto path = directory_ / std::filesystem::path{u8"absent ü.ini"};
    const auto result = loadConfigFile(path, ConfigFilePolicy::Required);
    EXPECT_EQ(result.status, ConfigFileStatus::NotFound);
    ASSERT_EQ(result.diagnostics.size(), 1U);
    EXPECT_NE(result.diagnostics[0].message.find(utf8(u8"absent ü.ini does not exist")),
              std::string::npos) << result.diagnostics[0].message;
}

TEST_F(ConfigFileTest, RelativePathIsResolvedAgainstTheWorkingDirectory)
{
    std::filesystem::create_directories(directory_ / "conf");
    writeFile("conf/micro_robot.ini", "[robot]\nid = 11\n");
    const WorkingDirectoryGuard workingDirectory{directory_};

    const std::filesystem::path relative = std::filesystem::path{"conf"} / "micro_robot.ini";
    const auto result = loadConfigFile(relative, ConfigFilePolicy::Required);
    EXPECT_EQ(result.status, ConfigFileStatus::Loaded);
    EXPECT_EQ(result.path, relative); // Reported as given, not rewritten.
    EXPECT_EQ(result.config.robot.id, RobotId{11});
    EXPECT_EQ(loadConfigFile("absent.ini").status, ConfigFileStatus::NotFound);
}

TEST_F(ConfigFileTest, FollowsSymbolicLinks)
{
    const auto target = writeFile("target.ini", "[robot]\nid = 12\n");
    const auto link = directory_ / "link.ini";
    const auto dangling = directory_ / "dangling.ini";
    try {
        std::filesystem::create_symlink(target, link);
        std::filesystem::create_symlink(directory_ / "absent.ini", dangling);
    } catch (const std::filesystem::filesystem_error& error) {
        GTEST_SKIP() << "Symbolic links are unavailable: " << error.what();
    }

    const auto linked = loadConfigFile(link, ConfigFilePolicy::Required);
    EXPECT_EQ(linked.status, ConfigFileStatus::Loaded);
    EXPECT_EQ(linked.config.robot.id, RobotId{12});

    const auto broken = loadConfigFile(dangling, ConfigFilePolicy::Required);
    EXPECT_EQ(broken.status, ConfigFileStatus::NotFound);
    ASSERT_EQ(broken.diagnostics.size(), 1U);
    EXPECT_EQ(broken.diagnostics[0].level, LogLevel::Error);
}

TEST_F(ConfigFileTest, UnreadableFileUsesDefaultsAndIsAnError)
{
    const auto path = writeFile("locked.ini", "[robot]\nid = 13\n");
    std::filesystem::permissions(path, std::filesystem::perms::none);
    if (std::ifstream{path}) {
        std::filesystem::permissions(path, std::filesystem::perms::owner_all);
        GTEST_SKIP() << "Permission bits do not prevent reading here (root or Windows)";
    }

    const auto result = loadConfigFile(path);
    std::filesystem::permissions(path, std::filesystem::perms::owner_all); // Allow cleanup.
    EXPECT_EQ(result.status, ConfigFileStatus::Unreadable);
    ASSERT_EQ(result.diagnostics.size(), 1U);
    EXPECT_EQ(result.diagnostics[0].level, LogLevel::Error);
    EXPECT_NE(result.diagnostics[0].message.find("Cannot open configuration file"), std::string::npos);
    expectDefaults(result.config);
}

TEST(ConfigParseTest, LogFilePathIsKeptVerbatimForEveryPlatformStyle)
{
    for (const auto& path : {utf8(u8"/var/log/robot ü/run.log"), std::string{R"(C:\Robot Logs\run.log)"},
                             std::string{"logs/../run.log"}, std::string{"~/run.log"}}) {
        const auto result = parseConfig("[logging]\nfile_path = " + path + "\n", "test.ini");
        EXPECT_TRUE(result.diagnostics.empty()) << path;
        EXPECT_EQ(result.config.logging.filePath, path);
    }
}

TEST(ConfigExampleTest, ShippedExampleIsValidAndDocumentsDefaults)
{
    const auto result = loadConfigFile(MICRO_ROBOT_EXAMPLE_CONFIG, ConfigFilePolicy::Required);
    ASSERT_EQ(result.status, ConfigFileStatus::Loaded);
    for (const auto& diagnostic : result.diagnostics) ADD_FAILURE() << diagnostic.message;
    expectDefaults(result.config);
}

class ConfigLoggingTest : public ::testing::Test {
protected:
    void SetUp() override { logger_.addSink(sink_); }

    std::vector<LogRecord>& records() { return sink_->records; }
    LogChannel channel() { return logger_.channel("config"); }

private:
    Logger logger_{LogLevel::Debug};
    std::shared_ptr<CapturingSink> sink_ = std::make_shared<CapturingSink>();
};

TEST_F(ConfigLoggingTest, LogsSourceOfValidConfiguration)
{
    ConfigLoadResult result;
    result.path = "settings.ini";
    result.status = ConfigFileStatus::Loaded;
    logConfigLoadResult(result, channel());

    ASSERT_EQ(records().size(), 1U);
    EXPECT_EQ(records()[0].level, LogLevel::Info);
    EXPECT_EQ(records()[0].module, "config");
    EXPECT_EQ(records()[0].message, "Configuration loaded from settings.ini");
}

TEST_F(ConfigLoggingTest, LogsDefaultsWhenOptionalFileIsAbsent)
{
    ConfigLoadResult result;
    result.path = "settings.ini";
    result.status = ConfigFileStatus::NotFound;
    logConfigLoadResult(result, channel());

    ASSERT_EQ(records().size(), 1U);
    EXPECT_EQ(records()[0].level, LogLevel::Info);
    EXPECT_EQ(records()[0].message, "No configuration file at settings.ini; using built-in defaults");
}

TEST_F(ConfigLoggingTest, LogsEveryDiagnosticAtItsLevel)
{
    ConfigLoadResult result;
    result.path = "settings.ini";
    result.status = ConfigFileStatus::Loaded;
    result.diagnostics = {{LogLevel::Warning, "first"}, {LogLevel::Warning, "second"}};
    logConfigLoadResult(result, channel());

    ASSERT_EQ(records().size(), 3U);
    EXPECT_EQ(records()[0].level, LogLevel::Warning);
    EXPECT_EQ(records()[0].message,
              "Configuration loaded from settings.ini with 2 problem(s); "
              "affected settings keep their defaults");
    EXPECT_EQ(records()[1].message, "first");
    EXPECT_EQ(records()[2].message, "second");
}

TEST_F(ConfigLoggingTest, LogsLoadErrorsOnlyOnce)
{
    ConfigLoadResult result;
    result.path = "settings.ini";
    result.status = ConfigFileStatus::Unreadable;
    result.diagnostics = {{LogLevel::Error, "Cannot open configuration file settings.ini"}};
    logConfigLoadResult(result, channel());

    ASSERT_EQ(records().size(), 1U);
    EXPECT_EQ(records()[0].level, LogLevel::Error);
}

} // namespace

TEST(ConfigLoaderTest, ManualSpeedLimitsAreConfigurableAndRejectInvalidValues)
{
    const auto valid = micro_robot::parseConfig("[manual]\nmaximum_linear_m_per_s = 0.025\nmaximum_angular_rad_per_s = 0.4\n", "test.ini");
    EXPECT_DOUBLE_EQ(valid.config.manual.maximumLinearMetersPerSecond, 0.025);
    EXPECT_DOUBLE_EQ(valid.config.manual.maximumAngularRadiansPerSecond, 0.4);
    for (const auto value : {"0", "-1", "nan", "inf", "1junk"}) {
        const auto invalid = micro_robot::parseConfig(std::string{"[manual]\nmaximum_linear_m_per_s = "} + value, "test.ini");
        EXPECT_DOUBLE_EQ(invalid.config.manual.maximumLinearMetersPerSecond, 0.1);
        EXPECT_FALSE(invalid.diagnostics.empty());
    }
}
