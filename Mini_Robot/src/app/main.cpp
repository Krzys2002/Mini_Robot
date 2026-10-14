#include "ui/MainWindow.hpp"
#include "app/DesktopController.hpp"
#include "app/CameraSelectionController.hpp"
#include "camera/QtCameraDeviceDiscovery.hpp"
#include "infrastructure/configuration/ConfigLoader.hpp"
#include "infrastructure/logging/LogSinks.hpp"
#include "infrastructure/logging/Logger.hpp"
#include "simulation/FakeCameraSourceFactory.hpp"
#include "simulation/SimulationTransport.hpp"

#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QStandardPaths>
#include <QTimer>

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace {

std::filesystem::path toPath(const QString& path)
{
    const auto utf8 = path.toUtf8();
    return std::filesystem::path{std::u8string(utf8.begin(), utf8.end())};
}

std::filesystem::path utf8ToPath(const std::string& path)
{
    return std::filesystem::path{std::u8string(path.begin(), path.end())};
}

std::string toDisplayString(const std::filesystem::path& path)
{
    return QDir::toNativeSeparators(QString::fromStdU16String(path.u16string())).toStdString();
}

} // namespace

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    QApplication::setApplicationName(QStringLiteral("Micro Robot"));
    QApplication::setApplicationVersion(QStringLiteral(MICRO_ROBOT_VERSION));

    QCommandLineParser arguments;
    arguments.setApplicationDescription(QStringLiteral("Micro-robot swarm desktop control"));
    arguments.addHelpOption();
    arguments.addVersionOption();
    const QCommandLineOption configOption{QStringLiteral("config"),
        QStringLiteral("Load settings from <file> instead of the per-user configuration file."),
        QStringLiteral("file")};
    arguments.addOption(configOption);
    arguments.process(application);

    using namespace micro_robot;
    // Configuration is loaded once, before anything else, and its values are injected
    // below. Diagnostics are logged after the logger has been configured from it.
    const bool configRequested = arguments.isSet(configOption);
    const auto configPath = configRequested
        ? toPath(QDir::current().absoluteFilePath(arguments.value(configOption)))
        : toPath(QDir(QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation))
              .filePath(QStringLiteral("micro_robot.ini")));
    const auto configResult = loadConfigFile(
        configPath, configRequested ? ConfigFilePolicy::Required : ConfigFilePolicy::Optional);
    const ApplicationConfig& config = configResult.config;

    // The logger is declared first so that it outlives every module that logs.
    // Records go to stderr and, unless disabled, are appended to a log file. Sinks
    // are attached before the first record so that startup diagnostics reach both.
    Logger logger{config.logging.minimumLevel};
    logger.addSink(std::make_shared<StreamLogSink>(std::clog));
    std::shared_ptr<FileLogSink> logFile;
    if (config.logging.fileEnabled) {
        logFile = std::make_shared<FileLogSink>(config.logging.filePath.empty()
            ? toPath(QDir(QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation))
                  .filePath(QStringLiteral("logs/micro_robot.log")))
            : utf8ToPath(config.logging.filePath));
        if (logFile->isOpen()) logger.addSink(logFile);
    }

    const auto appLog = logger.channel("app");
    appLog.info("Application starting (version " + QApplication::applicationVersion().toStdString()
        + ")");
    logConfigLoadResult(configResult, logger.channel("config"));
    if (!logFile) {
        appLog.info("File logging disabled by configuration");
    } else if (logFile->isOpen()) {
        appLog.info("Log file: " + toDisplayString(logFile->path()));
    } else {
        appLog.warning("File logging disabled; cannot open " + toDisplayString(logFile->path()));
    }

    // Composition root alone chooses concrete implementations. Declaration order
    // keeps sources/model alive until after the controller detaches its event sink;
    // the camera factory outlives the sources the controller owns. Until a physical
    // capture backend exists, a selected device is served by a simulated feed.
    FakeCameraConfig cameraConfig;
    cameraConfig.frameInterval = config.simulation.cameraFrameInterval;
    FakeCameraSourceFactory cameraFactory{cameraConfig};
    auto transport = std::make_unique<SimulationTransport>();
    QtCameraDeviceDiscovery cameraDiscovery;
    WorldModel world;
    const RobotId robotId = config.robot.id;
    DesktopController controller{*transport, world, robotId, logger.channel("controller")};
    CameraSelectionController cameraSelection{cameraDiscovery, cameraFactory, controller,
                                              logger.channel("camera")};
    MainWindow window{controller};
    window.bindCameraSelection(cameraSelection);
    QTimer simulationTimer;
    simulationTimer.setInterval(config.simulation.updateInterval);

    bool robotConnected = false;
    QObject::connect(&controller, &DesktopController::connectionStatusChanged, &window,
                     [&robotConnected](const QString&, bool connected) { robotConnected = connected; });
    QObject::connect(&simulationTimer, &QTimer::timeout, &window, [&] {
        const auto now = MonotonicClock::now();
        cameraFactory.advanceTo(now);
        transport->advanceTo(now);
        if (robotConnected) {
            // Explicit demo observation, independent of camera frames or telemetry.
            // No tracking, filtering, physics, or closed-loop control is performed.
            (void)controller.applyPose(robotId,
                {Meters{0.25}, Meters{0.15}, Radians{0.0}}, now);
        }
    });
    QObject::connect(&application, &QCoreApplication::aboutToQuit, &window, [&] {
        appLog.info("Application shutting down");
        simulationTimer.stop();
        controller.shutdown();
        const auto now = MonotonicClock::now();
        cameraFactory.advanceTo(now);
        transport->advanceTo(now);
    });

    controller.initialize(MonotonicClock::now());
    cameraSelection.initialize();
    window.show();
    simulationTimer.start();
    appLog.info("Application started");

    const int exitCode = application.exec();
    appLog.info("Application shutdown complete (exit code " + std::to_string(exitCode) + ")");
    return exitCode;
}
