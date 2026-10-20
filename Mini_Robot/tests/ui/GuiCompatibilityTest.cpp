#include "app/CalibrationController.hpp"
#include "app/CameraSelectionController.hpp"
#include "app/DesktopController.hpp"
#include "infrastructure/logging/Logger.hpp"
#include "simulation/FakeCameraSourceFactory.hpp"
#include "simulation/SimulationTransport.hpp"
#include "ui/CalibrationWizard.hpp"
#include "ui/MainWindow.hpp"
#include "ManualBackends.hpp"

#include <QApplication>
#include <QComboBox>
#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QPushButton>
#include <QScreen>
#include <QSet>
#include <QStyleHints>
#include <QTest>
#include <QWindow>
#include <gtest/gtest.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

// Portable Qt GUI checks for #130. They run on the offscreen platform in CI and,
// with MICRO_ROBOT_TEST_QPA_PLATFORM set, on a real platform plugin (xcb, wayland,
// cocoa) inside a desktop session. Input is injected at the QWindow, so platform
// event dispatch and device-pixel scaling apply; it does not pass through the
// display server, which remains a manual check.
namespace {

using namespace micro_robot;
using namespace std::chrono_literals;

// Never finds a target; the wizard only needs to open and close here.
class NoTargetDetector final : public ICalibrationDetector {
public:
    CalibrationDetection detect(const Frame&, const CalibrationTarget&) override { return {}; }
};

QString expectedPlatform()
{
    const auto requested = qEnvironmentVariable("MICRO_ROBOT_TEST_QPA_PLATFORM");
    return requested.isEmpty() ? QStringLiteral("offscreen") : requested.section(u':', 0, 0);
}

// The same composition as main.cpp with fake camera and transport backends.
struct GuiFixture {
    Logger logger;
    test::ManualDiscovery discovery;
    FakeCameraSourceFactory cameras; // Outlives the source owned by the controller.
    SimulationTransport transport;
    WorldModel world;
    DesktopController controller{transport, world, RobotId{1}, logger.channel("controller")};
    CameraSelectionController selection{discovery, cameras, controller, logger.channel("camera")};
    CalibrationController calibration{std::make_unique<NoTargetDetector>(),
                                      logger.channel("calibration")};
    MainWindow window{controller};
    Timestamp now{1s};

    GuiFixture()
    {
        window.bindCalibration(calibration);
        window.bindCameraSelection(selection);
        controller.initialize(now);
        selection.initialize();
        discovery.publish({{"camera-a", "Camera A"}, {"camera-b", "Camera B"}});
        window.show();
    }
    ~GuiFixture()
    {
        window.close();
        calibration.shutdown();
        controller.shutdown();
        advanceTo(now + 1ms);
        (void)QTest::qWaitFor([this] { return calibration.isStopped(); });
    }
    void advanceTo(Timestamp at)
    {
        now = at;
        cameras.advanceTo(at);
        transport.advanceTo(at);
    }
    template <typename Widget>
    Widget* find(const char* name)
    {
        return window.findChild<Widget*>(QString::fromLatin1(name));
    }
    QLabel* label(const char* name) { return find<QLabel>(name); }
    QPushButton* button(const char* name) { return find<QPushButton>(name); }
    QComboBox* selector() { return find<QComboBox>("cameraSelector"); }
    QWindow* handle() { return window.windowHandle(); }
    // A compositor may fit the window into a smaller work area than the layout
    // minimum (Wayland), which clips the lowest controls.
    [[nodiscard]] bool clippedByScreen() const
    {
        return window.width() < window.minimumWidth() || window.height() < window.minimumHeight();
    }
    [[nodiscard]] std::string clippedReason() const
    {
        return "Platform shrank the window to " + std::to_string(window.width()) + "x"
            + std::to_string(window.height()) + ", below its minimum size "
            + std::to_string(window.minimumWidth()) + "x" + std::to_string(window.minimumHeight())
            + "; see the small-screen limitation in docs/development/gui-compatibility.md";
    }

    // Window-local logical coordinates; Qt maps them to native pixels and back.
    void click(QWidget* widget, QPoint at)
    {
        QTest::mouseClick(handle(), Qt::LeftButton, {}, widget->mapTo(&window, at));
    }
    void key(Qt::Key key) { QTest::keyClick(handle(), key); }

    // Named controls the operator relies on, in their visual top-to-bottom order.
    QList<QWidget*> controls()
    {
        QList<QWidget*> widgets;
        for (const char* name : {"applicationStatus", "selectedCamera", "activeCamera",
                                 "cameraStatus", "frameDescription", "connectionStatus",
                                 "telemetryDescription", "worldDescription"}) {
            widgets.push_back(label(name));
        }
        widgets.push_back(selector());
        for (const char* name : {"openCalibration", "startCamera", "stopCamera", "connectRobot",
                                 "disconnectRobot"}) {
            widgets.push_back(button(name));
        }
        return widgets;
    }
};

// Asserts that every control is shown inside the window at no less than the
// minimum size its style and font require.
void expectControlsUsable(GuiFixture& fixture)
{
    const QRect bounds = fixture.window.rect();
    for (QWidget* widget : fixture.controls()) {
        ASSERT_NE(widget, nullptr);
        SCOPED_TRACE(widget->objectName().toStdString());
        EXPECT_TRUE(widget->isVisibleTo(&fixture.window));
        const QRect geometry{widget->mapTo(&fixture.window, QPoint{}), widget->size()};
        EXPECT_TRUE(bounds.contains(geometry));
        const QSize minimum = widget->minimumSizeHint();
        EXPECT_GE(widget->width(), minimum.width());
        EXPECT_GE(widget->height(), minimum.height());
    }
}

// Counts paints that reach the platform backing store.
class PaintCounter final : public QObject {
public:
    explicit PaintCounter(QWidget& widget) { widget.installEventFilter(this); }
    int paints{};

protected:
    bool eventFilter(QObject*, QEvent* event) override
    {
        if (event->type() == QEvent::Paint) ++paints;
        return false;
    }
};

TEST(GuiCompatibilityTest, MainWindowStartsOnTheRequestedPlatform)
{
    GuiFixture fixture;
    ASSERT_TRUE(QTest::qWaitForWindowExposed(&fixture.window));
    const QString platform = QGuiApplication::platformName();
    const QScreen* screen = fixture.handle()->screen();
    ASSERT_NE(screen, nullptr);
    std::cout << "[ GUI      ] platform=" << platform.toStdString()
              << " devicePixelRatio=" << fixture.window.devicePixelRatioF()
              << " screen=" << screen->size().width() << 'x' << screen->size().height()
              << " available=" << screen->availableSize().width() << 'x'
              << screen->availableSize().height() << " logicalDpi=" << screen->logicalDotsPerInch()
              << " window=" << fixture.window.width() << 'x' << fixture.window.height()
              << " minimum=" << fixture.window.minimumWidth() << 'x'
              << fixture.window.minimumHeight() << '\n';
    ::testing::Test::RecordProperty("platform", platform.toStdString());
    ::testing::Test::RecordProperty("devicePixelRatio",
                                    std::to_string(fixture.window.devicePixelRatioF()));

    EXPECT_EQ(platform, expectedPlatform());
    EXPECT_TRUE(fixture.label("applicationStatus")->text().contains(QStringLiteral("Ready")));
    EXPECT_TRUE(fixture.selector()->isEnabled());
    EXPECT_EQ(fixture.selector()->count(), 2);
    if (fixture.clippedByScreen()) GTEST_SKIP() << fixture.clippedReason();
    expectControlsUsable(fixture);
}

TEST(GuiCompatibilityTest, ResizedWindowKeepsControlsUsableAndRepaints)
{
    GuiFixture fixture;
    ASSERT_TRUE(QTest::qWaitForWindowExposed(&fixture.window));
    if (fixture.clippedByScreen()) GTEST_SKIP() << fixture.clippedReason();
    PaintCounter painted{*fixture.window.centralWidget()};
    const QSize minimum = fixture.window.minimumSize();
    ASSERT_FALSE(minimum.isEmpty());
    const QSize available = fixture.handle()->screen()->availableSize() * 0.9;
    const QSize larger = (fixture.window.size() + QSize{240, 180}).boundedTo(available);

    // A request below the layout minimum is clamped instead of compressing controls.
    for (const QSize requested : {larger, QSize{200, 150}, fixture.window.size()}) {
        SCOPED_TRACE(std::to_string(requested.width()) + "x" + std::to_string(requested.height()));
        const QSize expected = requested.expandedTo(minimum);
        const int paintsBefore = painted.paints;
        fixture.window.resize(requested);
        EXPECT_TRUE(QTest::qWaitFor([&] { return fixture.window.size() == expected; }));
        EXPECT_TRUE(QTest::qWaitFor([&] { return painted.paints > paintsBefore; }));
        EXPECT_TRUE(fixture.handle()->isExposed());
        expectControlsUsable(fixture);

        // Rendering uses the window's device-pixel ratio and produces visible content.
        const QPixmap rendered = fixture.window.grab();
        EXPECT_DOUBLE_EQ(rendered.devicePixelRatio(), fixture.window.devicePixelRatioF());
        EXPECT_EQ(rendered.size(), fixture.window.size() * fixture.window.devicePixelRatioF());
        const QImage image = rendered.toImage();
        QSet<QRgb> colors;
        for (int y = 0; y < image.height(); y += 4) {
            for (int x = 0; x < image.width(); x += 4) colors.insert(image.pixel(x, y));
        }
        EXPECT_GT(colors.size(), 2);
    }
}

TEST(GuiCompatibilityTest, MouseClicksDriveCameraAndRobotControls)
{
    GuiFixture fixture;
    ASSERT_TRUE(QTest::qWaitForWindowExposed(&fixture.window));
    if (fixture.clippedByScreen()) GTEST_SKIP() << fixture.clippedReason();
    auto* connect = fixture.button("connectRobot");
    auto* disconnect = fixture.button("disconnectRobot");

    // A disabled button ignores clicks.
    fixture.click(disconnect, disconnect->rect().center());
    fixture.advanceTo(Timestamp{1s + 100ms});
    EXPECT_EQ(fixture.label("connectionStatus")->text(), QStringLiteral("Disconnected"));

    // Clicks one logical pixel inside opposite corners still hit the button.
    fixture.click(connect, {1, 1});
    fixture.advanceTo(Timestamp{1s + 200ms});
    EXPECT_EQ(fixture.label("connectionStatus")->text(), QStringLiteral("Connected"));
    EXPECT_FALSE(connect->isEnabled());
    ASSERT_TRUE(disconnect->isEnabled());
    fixture.click(disconnect, {disconnect->width() - 2, disconnect->height() - 2});
    fixture.advanceTo(Timestamp{1s + 300ms});
    EXPECT_EQ(fixture.label("connectionStatus")->text(), QStringLiteral("Disconnected"));

    // Releasing outside the button cancels the press.
    const QPoint inside = connect->mapTo(&fixture.window, connect->rect().center());
    const QPoint outside = connect->mapTo(&fixture.window, QPoint{connect->width() / 2, -4});
    QTest::mousePress(fixture.handle(), Qt::LeftButton, {}, inside);
    QTest::mouseMove(fixture.handle(), outside);
    QTest::mouseRelease(fixture.handle(), Qt::LeftButton, {}, outside);
    fixture.advanceTo(Timestamp{1s + 400ms});
    EXPECT_EQ(fixture.label("connectionStatus")->text(), QStringLiteral("Disconnected"));

    // A camera chosen by the operator can be started and stopped with the mouse.
    fixture.selector()->setFocus(Qt::MouseFocusReason);
    fixture.key(Qt::Key_Down);
    ASSERT_TRUE(fixture.button("startCamera")->isEnabled());
    fixture.click(fixture.button("startCamera"), fixture.button("startCamera")->rect().center());
    fixture.advanceTo(Timestamp{1s + 500ms});
    EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Streaming"));
    fixture.click(fixture.button("stopCamera"), fixture.button("stopCamera")->rect().center());
    fixture.advanceTo(Timestamp{1s + 600ms});
    EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Stopped"));
}

TEST(GuiCompatibilityTest, KeyboardSelectsCamerasAndActivatesFocusedButtons)
{
    GuiFixture fixture;
    ASSERT_TRUE(QTest::qWaitForWindowExposed(&fixture.window));

    // Arrow keys on the closed drop-down choose cameras without opening a popup.
    fixture.selector()->setFocus(Qt::TabFocusReason);
    fixture.key(Qt::Key_Down);
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("Camera A (simulated feed)"));
    fixture.key(Qt::Key_Down);
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("Camera B (simulated feed)"));
    fixture.key(Qt::Key_Up);
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("Camera A (simulated feed)"));
    EXPECT_EQ(fixture.selector()->currentData().toString(), QStringLiteral("camera-a"));

    // Space activates the focused button.
    fixture.button("startCamera")->setFocus(Qt::TabFocusReason);
    fixture.key(Qt::Key_Space);
    fixture.advanceTo(Timestamp{1s + 100ms});
    EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Streaming"));
    fixture.button("connectRobot")->setFocus(Qt::TabFocusReason);
    fixture.key(Qt::Key_Space);
    fixture.advanceTo(Timestamp{1s + 200ms});
    EXPECT_EQ(fixture.label("connectionStatus")->text(), QStringLiteral("Connected"));
}

TEST(GuiCompatibilityTest, TabFollowsTheVisualOrderOfEnabledControls)
{
    if (QGuiApplication::styleHints()->tabFocusBehavior() != Qt::TabFocusAllControls) {
        // For example macOS without keyboard navigation: buttons take no Tab focus.
        GTEST_SKIP() << "Platform keeps Tab focus on text and list controls";
    }
    GuiFixture fixture;
    ASSERT_TRUE(QTest::qWaitForWindowExposed(&fixture.window));
    fixture.selector()->setFocus(Qt::TabFocusReason);
    fixture.key(Qt::Key_Down); // Select a camera so that Start camera is enabled.

    // Stop camera and Disconnect robot are disabled and skipped.
    for (const char* name : {"openCalibration", "startCamera", "connectRobot"}) {
        fixture.key(Qt::Key_Tab);
        ASSERT_NE(fixture.window.focusWidget(), nullptr);
        EXPECT_EQ(fixture.window.focusWidget()->objectName().toStdString(), name);
    }
    fixture.key(Qt::Key_Tab);
    EXPECT_EQ(fixture.window.focusWidget(), fixture.selector());
    QTest::keyClick(fixture.handle(), Qt::Key_Backtab, Qt::ShiftModifier);
    EXPECT_EQ(fixture.window.focusWidget(), fixture.button("connectRobot"));
}

TEST(GuiCompatibilityTest, CalibrationDialogOpensAndClosesWithTheMainWindow)
{
    GuiFixture fixture;
    ASSERT_TRUE(QTest::qWaitForWindowExposed(&fixture.window));
    auto* openCalibration = fixture.button("openCalibration");
    fixture.click(openCalibration, openCalibration->rect().center());
    auto* wizard = fixture.window.findChild<CalibrationWizard*>();
    ASSERT_NE(wizard, nullptr);
    ASSERT_TRUE(QTest::qWaitForWindowExposed(wizard));
    EXPECT_TRUE(wizard->isWindow());

    // Closing through the platform window, as a window-manager close does.
    fixture.handle()->close();
    EXPECT_TRUE(QTest::qWaitFor([&] { return !fixture.window.isVisible(); }));
    EXPECT_FALSE(wizard->isVisible());
}

TEST(GuiCompatibilityTest, ScaleFactorSetsTheDevicePixelRatio)
{
    GuiFixture fixture;
    ASSERT_TRUE(QTest::qWaitForWindowExposed(&fixture.window));
    const qreal ratio = fixture.window.devicePixelRatioF();
    EXPECT_DOUBLE_EQ(ratio, fixture.handle()->devicePixelRatio());
    bool numeric = false;
    const qreal factor = qEnvironmentVariable("QT_SCALE_FACTOR").toDouble(&numeric);
    if (numeric && factor > 0 && expectedPlatform() == QStringLiteral("offscreen")) {
        // The offscreen screen reports a native ratio of 1.
        EXPECT_NEAR(ratio, factor, 1e-9);
    } else {
        EXPECT_GE(ratio, 1.0);
    }
    // Layout is in logical pixels; the pixel buffer follows the ratio.
    const QImage image = fixture.window.grab().toImage();
    EXPECT_EQ(image.size(), fixture.window.size() * ratio);
    if (fixture.clippedByScreen()) GTEST_SKIP() << fixture.clippedReason();
    expectControlsUsable(fixture);
}

} // namespace
