#include "ui/MainWindow.hpp"
#include "app/DesktopController.hpp"
#include "simulation/FakeCameraSource.hpp"
#include "simulation/SimulationTransport.hpp"

#include <QApplication>
#include <QTimer>

#include <memory>

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    QApplication::setApplicationName(QStringLiteral("Micro Robot"));

    using namespace micro_robot;
    // Composition root alone chooses concrete implementations. Declaration order
    // keeps sources/model alive until after the controller detaches its event sink.
    auto camera = std::make_unique<FakeCameraSource>();
    auto transport = std::make_unique<SimulationTransport>();
    WorldModel world;
    DesktopController controller{*camera, *transport, world, RobotId{1}};
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
        simulationTimer.stop();
        controller.shutdown();
        const auto now = MonotonicClock::now();
        camera->advanceTo(now);
        transport->advanceTo(now);
    });

    controller.initialize(MonotonicClock::now());
    window.show();
    simulationTimer.start();

    return application.exec();
}
