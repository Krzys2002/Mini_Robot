#include "camera/QtCameraSource.hpp"

#include <QEventLoop>
#include <QThread>
#include <QTimer>
#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace {
using namespace micro_robot;

class Events : public ICameraEvents {
public:
    explicit Events(ICameraSource& source) : source_(source) {}
    void onFrame(FramePtr frame) override { frames.push_back(std::move(frame)); }
    void onCameraStateChanged(CameraState state, Timestamp) override
    {
        EXPECT_EQ(QThread::currentThread(), thread);
        EXPECT_EQ(source_.state(), state);
        states.push_back(state);
        if (state == CameraState::Stopped || state == CameraState::Fault) loop.quit();
    }
    void onCameraError(const CameraError& error) override
    {
        EXPECT_EQ(QThread::currentThread(), thread);
        errors.push_back(error);
    }
    void awaitCompletion()
    {
        QTimer deadline;
        deadline.setSingleShot(true);
        QObject::connect(&deadline, &QTimer::timeout, &loop, &QEventLoop::quit);
        deadline.start(5000);
        loop.exec();
        EXPECT_TRUE(deadline.isActive()) << "Timed out awaiting camera completion";
    }
    QEventLoop loop;
    QThread* thread{QThread::currentThread()};
    ICameraSource& source_;
    std::vector<CameraState> states;
    std::vector<CameraError> errors;
    std::vector<FramePtr> frames;
};

CameraDevice missingDevice()
{
    return {QByteArray("mini-robot-test-device-that-does-not-exist").toHex().toStdString(),
            "Unavailable test camera"};
}

TEST(QtCameraSourceTest, FactoryCreatesStoppedSourcesWithoutOpeningDevices)
{
    QtCameraSourceFactory factory;
    EXPECT_FALSE(factory.supports({{}, "No ID"}));
    EXPECT_FALSE(factory.supports({"not-hex", "Malformed ID"}));
    EXPECT_FALSE(factory.create({"0", "Truncated ID"}));
    auto source = factory.create(missingDevice());
    ASSERT_TRUE(source);
    EXPECT_EQ(source->state(), CameraState::Stopped);
    EXPECT_EQ(source->info().sourceId, missingDevice().id);
}

TEST(QtCameraSourceTest, AbsentDeviceFailsAsynchronouslyAndCanBeRetried)
{
    QtCameraSource source{missingDevice()};
    auto events = std::make_shared<Events>(source);
    source.setEventSink(events);
    for (int attempt = 0; attempt < 3; ++attempt) {
        const auto count = events->states.size();
        source.start();
        source.start();
        EXPECT_EQ(events->states.size(), count);
        events->awaitCompletion();
        ASSERT_EQ(events->states.size(), count + 2);
        EXPECT_EQ(events->states[count], CameraState::Starting);
        EXPECT_EQ(events->states.back(), CameraState::Fault);
        ASSERT_EQ(events->errors.size(), static_cast<std::size_t>(attempt + 1));
        EXPECT_EQ(events->errors.back().code, CameraErrorCode::Unavailable);
        EXPECT_FALSE(events->errors.back().message.empty());
        EXPECT_TRUE(events->frames.empty());
    }
    source.stop(); // Terminal states are already quiescent.
    EXPECT_EQ(source.state(), CameraState::Fault);
}

TEST(QtCameraSourceTest, StopCancelsPendingStartAndAllowsRepeatedCycles)
{
    QtCameraSource source{missingDevice()};
    auto events = std::make_shared<Events>(source);
    source.setEventSink(events);
    for (int cycle = 0; cycle < 3; ++cycle) {
        source.start();
        source.stop();
        source.stop();
        source.start(); // Ignored until cancellation is acknowledged.
        const auto count = events->states.size();
        events->awaitCompletion();
        ASSERT_EQ(events->states.size(), count + 2);
        EXPECT_EQ(events->states[count], CameraState::Stopping);
        EXPECT_EQ(events->states.back(), CameraState::Stopped);
        EXPECT_TRUE(events->errors.empty());
        EXPECT_TRUE(events->frames.empty());
    }
}

TEST(QtCameraSourceTest, AStateCallbackCanCancelWithoutNestedDispatch)
{
    class CancellingEvents final : public Events {
    public:
        explicit CancellingEvents(ICameraSource& source) : Events(source) {}
        void onCameraStateChanged(CameraState state, Timestamp at) override
        {
            EXPECT_FALSE(inCallback);
            inCallback = true;
            Events::onCameraStateChanged(state, at);
            if (state == CameraState::Starting) source_.stop();
            inCallback = false;
        }
        bool inCallback{};
    };
    QtCameraSource source{missingDevice()};
    auto events = std::make_shared<CancellingEvents>(source);
    source.setEventSink(events);
    source.start();
    events->awaitCompletion();
    EXPECT_EQ(source.state(), CameraState::Stopped);
    EXPECT_TRUE(events->errors.empty());
}
} // namespace
