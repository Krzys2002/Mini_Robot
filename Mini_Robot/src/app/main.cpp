#include "ui/MainWindow.hpp"
#include "app/DesktopController.hpp"
#include "app/CalibrationController.hpp"
#include "app/VisionController.hpp"
#include "vision/OpenCvAprilTagDetector.hpp"
#include "calibration/OpenCvCalibrationDetector.hpp"
#include "calibration/OpenCvIntrinsicCalibrationSolver.hpp"
#include "calibration/OpenCvCalibrationUndistorter.hpp"
#include "calibration/QtCalibrationProfileStore.hpp"
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
        QStringLiteral("Use the BLE transport instead of simulation (requires BLE build).")};
    arguments.addOption(bleOption);
    const QCommandLineOption exitAfterStartupOption{QStringLiteral("exit-after-startup"),
        QStringLiteral("Close the window once startup completes and exit through the normal "
                       "shutdown path (automated startup checks).")};
    arguments.addOption(exitAfterStartupOption);
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
    RobotManager robots{*transport, world, logger.channel("robots")};
    DesktopController controller{robots, world, robotId, logger.channel("controller"), nullptr, config.manual};
    CameraSelectionController cameraSelection{cameraDiscovery, cameraFactory, controller,
                                              logger.channel("camera")};
    CalibrationController calibration{std::make_unique<OpenCvCalibrationDetector>(),
                                      std::make_unique<OpenCvIntrinsicCalibrationSolver>(),
                                      logger.channel("calibration")};
    calibration.setProfileServices(std::make_unique<QtCalibrationProfileStore>(
        QDir(QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation))
            .filePath(QStringLiteral("calibration"))), std::make_unique<OpenCvCalibrationUndistorter>());
    QObject::connect(&controller, &DesktopController::calibrationInputChanged,
                     &calibration, &CalibrationController::setInput);
    QObject::connect(&controller, &DesktopController::frameAvailable,
                     &calibration, &CalibrationController::receiveFrame);
    VisionController vision{std::make_unique<OpenCvAprilTagDetector>(), logger.channel("vision")};
    QObject::connect(&controller, &DesktopController::cameraInputChanged,
                     &vision, &VisionController::setInput);
    QObject::connect(&controller, &DesktopController::frameAvailable,
                     &vision, &VisionController::receiveFrame);
    // Native discovery mapping stays in composition; the UI receives only IDs,
    // names and ambiguity, never radio addresses or BLE device objects.
#if defined(MICRO_ROBOT_HAS_BLE)
    if (auto* ble = dynamic_cast<BleTransport*>(transport.get())) {
        QObject::connect(&controller, &DesktopController::robotScanRequested, ble, &BleTransport::startDiscovery);
        QObject::connect(ble, &BleTransport::discoveryChanged, &controller, [ble, &controller] {
            std::vector<DiscoveredRobot> found;
            for (const auto& device : ble->devices()) {
                if (device.advertisedId)
                    found.push_back({*device.advertisedId, device.name, MonotonicClock::now(), false});
            }
            controller.setDiscoveredRobots(std::move(found));
            controller.setRobotScanState(ble->discovering());
        });
        QObject::connect(ble, &BleTransport::discoveryError, &controller,
            [&controller](const QString& message) { emit controller.errorOccurred(message); });
    }
#endif
    if (simulation) {
        QObject::connect(&controller, &DesktopController::robotScanRequested, &controller,
            [&controller, robotId] {
                controller.setDiscoveredRobots({{robotId, "Simulated robot", MonotonicClock::now(), false}});
                controller.setRobotScanState(false);
            });
    }
    MainWindow window{controller};
    window.bindVision(vision);
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
        if (closing && !cameraActive && !transportActive && robots.isStopped() && calibration.isStopped() && vision.isStopped()) application.quit();
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
        vision.shutdown();
        calibration.shutdown();
        controller.shutdown();
        finishShutdown();
    });
    QObject::connect(&controller, &DesktopController::robotSessionsStopped, &window, finishShutdown);
    QObject::connect(&calibration, &CalibrationController::stopped, &window, finishShutdown);
    QObject::connect(&vision, &VisionController::stopped, &window, finishShutdown);
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
        vision.shutdown();
        calibration.shutdown();
        controller.shutdown();
        const auto now = MonotonicClock::now();
        if (simulation) simulation->advanceTo(now);
    });

    controller.initialize(MonotonicClock::now());
    cameraSelection.initialize();
    if (simulation) controller.scanRobots();
    window.show();
    if (simulation) simulationTimer.start();
    appLog.info("Application started");
    if (arguments.isSet(exitAfterStartupOption)) {
        // Queued so that shutdown begins from the running event loop, as for a user close.
        QTimer::singleShot(0, &window, [&window] { window.close(); });
    }

    const int exitCode = application.exec();
    appLog.info("Application shutdown complete (exit code " + std::to_string(exitCode) + ")");
    return exitCode;
}
