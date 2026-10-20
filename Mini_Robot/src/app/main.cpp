#include "ui/MainWindow.hpp"
#include "app/DesktopController.hpp"
#include "app/CalibrationController.hpp"
#include "calibration/OpenCvCalibrationDetector.hpp"
#include "app/CameraSelectionController.hpp"
#if defined(MICRO_ROBOT_CAMERA_BACKEND_V4L2)
#include "camera/V4l2CameraDeviceDiscovery.hpp"
#include "camera/V4l2CameraSource.hpp"
#else
#include "camera/QtCameraDeviceDiscovery.hpp"
#include "camera/QtCameraSource.hpp"
#endif
#include "infrastructure/configuration/ConfigLoader.hpp"
#include "infrastructure/logging/LogSinks.hpp"
#include "infrastructure/logging/Logger.hpp"
#include "simulation/SimulationTransport.hpp"
#if defined(MICRO_ROBOT_HAS_BLE)
#include "transport/ble/BleTransport.hpp"
#include "transport/ble/QtBleBackend.hpp"
#endif

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
    const QCommandLineOption bleOption{QStringLiteral("ble"),
        QStringLiteral("Use the connection-only BLE transport instead of simulation (requires BLE build).")};
    arguments.addOption(bleOption);
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

    // Composition root selects the native adapter; controllers depend on interfaces.
    // The build chooses the live camera backend (MICRO_ROBOT_CAMERA_BACKEND).
#if defined(MICRO_ROBOT_CAMERA_BACKEND_V4L2)
    V4l2CameraSourceFactory cameraFactory{config.camera.capture};
    using CameraDiscovery = V4l2CameraDeviceDiscovery;
    appLog.info("Camera backend: OpenCV V4L2");
#else
    QtCameraSourceFactory cameraFactory{config.camera.capture};
    using CameraDiscovery = QtCameraDeviceDiscovery;
    appLog.info("Camera backend: Qt Multimedia");
#endif
    if (const auto& resolution = config.camera.capture.resolution) {
        appLog.info("Camera resolution requested: " + std::to_string(resolution->width) + "x"
                    + std::to_string(resolution->height));
    }
    SimulationTransportConfig transportConfig;
    transportConfig.telemetryInterval = config.simulation.telemetryInterval;
    std::unique_ptr<IRobotTransport> transport;
    SimulationTransport* simulation = nullptr; // Non-owning alias, only in simulation mode.
    if (arguments.isSet(bleOption)) {
#if defined(MICRO_ROBOT_HAS_BLE)
        auto ble = std::make_unique<BleTransport>(makeQtBleBackend(), BleTransportConfig{}, logger.channel("ble"));
        QObject::connect(ble.get(), &BleTransport::phaseChanged, &application,
            [appLog](RobotId id, BlePhase phase) { appLog.info(std::string{"BLE setup: "} + blePhaseName(phase), id); });
        transport = std::move(ble);
#else
        appLog.error("BLE support was not built; configure with -DMICRO_ROBOT_BUILD_BLE=ON");
        return 2;
#endif
    } else {
        auto backend = std::make_unique<SimulationTransport>(transportConfig, logger.channel("simulation"));
        simulation = backend.get();
        transport = std::move(backend);
    }
    CameraDiscovery cameraDiscovery;
    WorldModel world;
    const RobotId robotId = config.robot.id;
    DesktopController controller{*transport, world, robotId, logger.channel("controller")};
    CameraSelectionController cameraSelection{cameraDiscovery, cameraFactory, controller,
                                              logger.channel("camera")};
    CalibrationController calibration{std::make_unique<OpenCvCalibrationDetector>(),
                                      logger.channel("calibration")};
    QObject::connect(&controller, &DesktopController::cameraInputChanged,
                     &calibration, &CalibrationController::setInput);
    QObject::connect(&controller, &DesktopController::frameAvailable,
                     &calibration, &CalibrationController::receiveFrame);
    MainWindow window{controller};
    if (arguments.isSet(bleOption)) window.setWindowTitle(QStringLiteral("Micro Robot — BLE connections"));
    window.bindCalibration(calibration);
    window.bindCameraSelection(cameraSelection);
    QTimer simulationTimer;
    simulationTimer.setInterval(config.simulation.updateInterval);

    application.setQuitOnLastWindowClosed(false);
    bool closing = false;
    bool cameraActive = false;
    bool transportActive = false;
    const auto finishShutdown = [&] {
        if (closing && !cameraActive && !transportActive && calibration.isStopped()) application.quit();
    };
    QObject::connect(&controller, &DesktopController::cameraStatusChanged, &window,
                     [&](const QString&, bool active) {
        cameraActive = active;
        finishShutdown();
    });
    QObject::connect(&controller, &DesktopController::connectionStatusChanged, &window,
                     [&](const QString&, bool, bool active) {
        transportActive = active;
        finishShutdown();
    });
    QObject::connect(&application, &QGuiApplication::lastWindowClosed, &window, [&] {
        closing = true;
        calibration.shutdown();
        controller.shutdown();
        finishShutdown();
    });
    QObject::connect(&calibration, &CalibrationController::stopped, &window, finishShutdown);
    bool robotConnected = false;
    QObject::connect(&controller, &DesktopController::connectionStatusChanged, &window,
                     [&robotConnected](const QString&, bool connected) { robotConnected = connected; });
    QObject::connect(&simulationTimer, &QTimer::timeout, &window, [&] {
        const auto now = MonotonicClock::now();
        if (simulation) simulation->advanceTo(now);
        if (simulation && robotConnected) {
            // Explicit demo observation, independent of camera frames or telemetry.
            // No tracking, filtering, physics, or closed-loop control is performed.
            (void)controller.applyPose(robotId,
                {Meters{0.25}, Meters{0.15}, Radians{0.0}}, now);
        }
    });
    QObject::connect(&application, &QCoreApplication::aboutToQuit, &window, [&] {
        appLog.info("Application shutting down");
        simulationTimer.stop();
        calibration.shutdown();
        controller.shutdown();
        const auto now = MonotonicClock::now();
        if (simulation) simulation->advanceTo(now);
    });

    controller.initialize(MonotonicClock::now());
    cameraSelection.initialize();
    window.show();
    if (simulation) simulationTimer.start();
    appLog.info("Application started");

    const int exitCode = application.exec();
    appLog.info("Application shutdown complete (exit code " + std::to_string(exitCode) + ")");
    return exitCode;
}
