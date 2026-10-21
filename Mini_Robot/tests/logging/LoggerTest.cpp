#include "infrastructure/logging/LogSinks.hpp"
#include "infrastructure/logging/Logger.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using namespace micro_robot;
using namespace std::chrono;
using namespace std::chrono_literals;

class CapturingSink final : public ILogSink {
public:
    void write(const LogRecord& record) override { records.push_back(record); }
    std::vector<LogRecord> records;
};

class ThrowingSink final : public ILogSink {
public:
    void write(const LogRecord&) override { throw std::runtime_error{"sink failure"}; }
};

// 2026-10-12T09:41:07.250Z
WallTime fixedTime()
{
    return sys_days{year{2026} / October / 12} + 9h + 41min + 7s + 250ms;
}

Logger::ClockFunction fixedClock()
{
    return [] { return fixedTime(); };
}

TEST(LogFormatTest, ContainsUtcTimestampLevelModuleAndMessage)
{
    const LogRecord record{fixedTime(), LogLevel::Info, "app", "Application started", std::nullopt};
    EXPECT_EQ(formatLogRecord(record), "2026-10-12T09:41:07.250Z [INFO] [app] Application started");
}

TEST(LogFormatTest, PadsTimestampFieldsAndIncludesRobotId)
{
    const auto time = sys_days{year{2027} / January / 3} + 4h + 5min + 6s + 7ms;
    const LogRecord record{time, LogLevel::Error, "transport", "Link lost", RobotId{3}};
    EXPECT_EQ(formatLogRecord(record), "2027-01-03T04:05:06.007Z [ERROR] [transport] robot=3 Link lost");
}

TEST(LogFormatTest, EscapesLineBreaksToKeepOneRecordPerLine)
{
    const LogRecord record{fixedTime(), LogLevel::Warning, "camera", "first\nsecond\r", std::nullopt};
    EXPECT_EQ(formatLogRecord(record), "2026-10-12T09:41:07.250Z [WARNING] [camera] first\\nsecond\\r");
}

TEST(LogFormatTest, ErrorsAreDistinguishableFromNormalMessages)
{
    EXPECT_EQ(toString(LogLevel::Debug), "DEBUG");
    EXPECT_EQ(toString(LogLevel::Info), "INFO");
    EXPECT_EQ(toString(LogLevel::Warning), "WARNING");
    EXPECT_EQ(toString(LogLevel::Error), "ERROR");
    EXPECT_LT(LogLevel::Info, LogLevel::Warning);
    EXPECT_LT(LogLevel::Warning, LogLevel::Error);
}

TEST(LoggerTest, DeliversRecordsWithClockTimeToEverySink)
{
    Logger logger{LogLevel::Debug, fixedClock()};
    auto first = std::make_shared<CapturingSink>();
    auto second = std::make_shared<CapturingSink>();
    logger.addSink(first);
    logger.addSink(second);
    logger.addSink(nullptr);

    logger.log(LogLevel::Info, "app", "started", RobotId{7});

    ASSERT_EQ(first->records.size(), 1U);
    ASSERT_EQ(second->records.size(), 1U);
    const auto& record = first->records.front();
    EXPECT_EQ(record.time, fixedTime());
    EXPECT_EQ(record.level, LogLevel::Info);
    EXPECT_EQ(record.module, "app");
    EXPECT_EQ(record.message, "started");
    EXPECT_EQ(record.robotId, RobotId{7});
}

TEST(LoggerTest, DropsRecordsBelowMinimumLevel)
{
    Logger logger{LogLevel::Warning, fixedClock()};
    auto sink = std::make_shared<CapturingSink>();
    logger.addSink(sink);

    logger.log(LogLevel::Debug, "app", "debug");
    logger.log(LogLevel::Info, "app", "info");
    logger.log(LogLevel::Warning, "app", "warning");
    logger.log(LogLevel::Error, "app", "error");
    EXPECT_FALSE(logger.isEnabled(LogLevel::Info));
    EXPECT_TRUE(logger.isEnabled(LogLevel::Error));

    ASSERT_EQ(sink->records.size(), 2U);
    EXPECT_EQ(sink->records[0].level, LogLevel::Warning);
    EXPECT_EQ(sink->records[1].level, LogLevel::Error);

    logger.setMinimumLevel(LogLevel::Debug);
    EXPECT_EQ(logger.minimumLevel(), LogLevel::Debug);
    logger.log(LogLevel::Debug, "app", "debug");
    EXPECT_EQ(sink->records.size(), 3U);
}

TEST(LoggerTest, ChannelsFromDifferentModulesShareOneLogger)
{
    Logger logger{LogLevel::Debug, fixedClock()};
    auto sink = std::make_shared<CapturingSink>();
    logger.addSink(sink);
    const auto camera = logger.channel("camera");
    const LogChannel transport{logger, "transport"};

    camera.debug("frame");
    camera.info("streaming");
    transport.warning("retrying", RobotId{2});
    transport.error("connection failed", RobotId{2});

    ASSERT_EQ(sink->records.size(), 4U);
    EXPECT_EQ(camera.module(), "camera");
    EXPECT_EQ(sink->records[0].module, "camera");
    EXPECT_EQ(sink->records[0].level, LogLevel::Debug);
    EXPECT_EQ(sink->records[1].level, LogLevel::Info);
    EXPECT_EQ(sink->records[2].module, "transport");
    EXPECT_EQ(sink->records[2].level, LogLevel::Warning);
    EXPECT_EQ(sink->records[3].level, LogLevel::Error);
    EXPECT_EQ(sink->records[3].robotId, RobotId{2});
}

TEST(LoggerTest, FailingSinkDoesNotThrowOrBlockOtherSinks)
{
    Logger logger{LogLevel::Info, fixedClock()};
    auto sink = std::make_shared<CapturingSink>();
    logger.addSink(std::make_shared<ThrowingSink>());
    logger.addSink(sink);

    EXPECT_NO_THROW(logger.log(LogLevel::Error, "app", "still delivered"));
    EXPECT_EQ(sink->records.size(), 1U);
}

TEST(LoggerTest, FailingClockDoesNotThrow)
{
    Logger logger{LogLevel::Info, []() -> WallTime { throw std::runtime_error{"clock failure"}; }};
    auto sink = std::make_shared<CapturingSink>();
    logger.addSink(sink);

    EXPECT_NO_THROW(logger.log(LogLevel::Info, "app", "dropped"));
    EXPECT_TRUE(sink->records.empty());
}

TEST(LoggerTest, DefaultClockProducesCurrentWallTime)
{
    Logger logger;
    auto sink = std::make_shared<CapturingSink>();
    logger.addSink(sink);

    const auto before = WallClock::now();
    logger.log(LogLevel::Info, "app", "now");
    const auto after = WallClock::now();

    ASSERT_EQ(sink->records.size(), 1U);
    EXPECT_GE(sink->records.front().time, before);
    EXPECT_LE(sink->records.front().time, after);
}

TEST(StreamLogSinkTest, WritesOneFormattedLinePerRecord)
{
    std::ostringstream stream;
    Logger logger{LogLevel::Info, fixedClock()};
    logger.addSink(std::make_shared<StreamLogSink>(stream));

    logger.channel("app").info("Application started");
    logger.channel("app").error("Failure");

    EXPECT_EQ(stream.str(),
              "2026-10-12T09:41:07.250Z [INFO] [app] Application started\n"
              "2026-10-12T09:41:07.250Z [ERROR] [app] Failure\n");
}

class FileLogSinkTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        const auto* test = ::testing::UnitTest::GetInstance()->current_test_info();
        directory_ = std::filesystem::temp_directory_path()
            / (std::string{"micro_robot_logging_"} + test->name());
        std::filesystem::remove_all(directory_);
    }
    void TearDown() override { std::filesystem::remove_all(directory_); }

    static std::string readFile(const std::filesystem::path& path)
    {
        std::ifstream file{path};
        std::ostringstream contents;
        contents << file.rdbuf();
        return contents.str();
    }

    std::filesystem::path directory_;
};

TEST_F(FileLogSinkTest, CreatesDirectoriesAndAppendsAcrossSessions)
{
    const auto path = directory_ / "nested" / "micro_robot.log";
    {
        Logger logger{LogLevel::Info, fixedClock()};
        auto sink = std::make_shared<FileLogSink>(path);
        ASSERT_TRUE(sink->isOpen());
        EXPECT_EQ(sink->path(), path);
        logger.addSink(sink);
        logger.channel("app").info("first session");
    }
    {
        Logger logger{LogLevel::Info, fixedClock()};
        logger.addSink(std::make_shared<FileLogSink>(path));
        logger.channel("app").info("second session");
    }

    EXPECT_EQ(readFile(path),
              "2026-10-12T09:41:07.250Z [INFO] [app] first session\n"
              "2026-10-12T09:41:07.250Z [INFO] [app] second session\n");
}

TEST_F(FileLogSinkTest, CreatesNonAsciiDirectoriesWithSpaces)
{
    // char8_t text keeps the path independent of the host's narrow encoding.
    const auto path = directory_ / std::filesystem::path{u8"robot logs ü/Ω run"} / "micro_robot.log";
    {
        Logger logger{LogLevel::Info, fixedClock()};
        auto sink = std::make_shared<FileLogSink>(path);
        ASSERT_TRUE(sink->isOpen());
        EXPECT_EQ(sink->path(), path);
        logger.addSink(sink);
        logger.channel("app").info("written");
    }

    EXPECT_TRUE(std::filesystem::is_regular_file(path));
    EXPECT_EQ(readFile(path), "2026-10-12T09:41:07.250Z [INFO] [app] written\n");
}

TEST_F(FileLogSinkTest, UnopenableFileReportsClosedAndIgnoresWrites)
{
    // A regular file cannot be used as the parent directory of the log file.
    std::filesystem::create_directories(directory_);
    const auto blocker = directory_ / "not_a_directory";
    std::ofstream{blocker} << "x";

    Logger logger{LogLevel::Info, fixedClock()};
    auto sink = std::make_shared<FileLogSink>(blocker / "micro_robot.log");
    EXPECT_FALSE(sink->isOpen());
    logger.addSink(sink);
    EXPECT_NO_THROW(logger.channel("app").error("not written"));
}

} // namespace
