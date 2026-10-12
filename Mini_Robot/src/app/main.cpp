#include "ui/MainWindow.hpp"
#include "app/DesktopController.hpp"
#include "infrastructure/logging/LogSinks.hpp"
#include "infrastructure/logging/Logger.hpp"
#include "simulation/FakeCameraSource.hpp"
#include "simulation/SimulationTransport.hpp"

#include <QApplication>
#include <QDir>
#include <QStandardPaths>
#include <QTimer>

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    QApplication::setApplicationName(QStringLiteral("Micro Robot"));
    QApplication::setApplicationVersion(QStringLiteral(MICRO_ROBOT_VERSION));

    using namespace micro_robot;
    // The logger is declared first so that it outlives every module that logs.
    // Records go to stderr and are appended to a per-user log file.
    Logger logger{LogLevel::Info};
    logger.addSink(std::make_shared<StreamLogSink>(std::clog));
    const auto logPath = QDir(QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation))
        .filePath(QStringLiteral("logs/micro_robot.log"));
    const auto logPathUtf8 = logPath.toUtf8();
    const auto logFile = std::make_shared<FileLogSink>(
        std::filesystem::path{std::u8string(logPathUtf8.begin(), logPathUtf8.end())});
    const auto appLog = logger.channel("app");
    const auto displayedLogPath = QDir::toNativeSeparators(logPath).toStdString();
    if (logFile->isOpen()) {
        logger.addSink(logFile);
    }
    appLog.info("Application starting (version " + QApplication::applicationVersion().toStdString()
        + ")");
    if (logFile->isOpen()) {
        appLog.info("Log file: " + displayedLogPath);
    } else {
        appLog.warning("File logging disabled; cannot open " + displayedLogPath);
    }

    // Composition root alone chooses concrete implementations. Declaration order
    // keeps sources/model alive until after the controller detaches its event sink.
    auto camera = std::make_unique<FakeCameraSource>();
    auto transport = std::make_unique<SimulationTransport>();
    WorldModel world;
    DesktopController controller{*camera, *transport, world, RobotId{1},
                                 logger.channel("controller")};
    MainWindow window{controller};
    QTimer simulationTimer;
    simulationTimer.setInterval(100);

    bool robotConnected = false;
    QObject::connect(&controller, &DesktopController::connectionStatusChanged, &window,
                     [&robotConnected](const QString&, bool connected) { robotConnected = connected; });
    QObject::connect(&simulationTimer, &QTimer::timeout, &window, [&] {
        const auto now = MonotonicClock::now();
        camera->advanceTo(now);
        transport->advanceTo(now);
        if (robotConnected) {
            // Explicit demo observation, independent of camera frames or telemetry.
            // No tracking, filtering, physics, or closed-loop control is performed.
            (void)controller.applyPose(RobotId{1},
                {Meters{0.25}, Meters{0.15}, Radians{0.0}}, now);
        }
    });
    QObject::connect(&application, &QCoreApplication::aboutToQuit, &window, [&] {
        appLog.info("Application shutting down");
        simulationTimer.stop();
        controller.shutdown();
        const auto now = MonotonicClock::now();
        camera->advanceTo(now);
        transport->advanceTo(now);
    });

    controller.initialize(MonotonicClock::now());
    window.show();
    simulationTimer.start();
    appLog.info("Application started");

    const int exitCode = application.exec();
    appLog.info("Application shutdown complete (exit code " + std::to_string(exitCode) + ")");
    return exitCode;
}
