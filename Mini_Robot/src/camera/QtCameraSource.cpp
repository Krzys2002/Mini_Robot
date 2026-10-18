#include "camera/QtCameraSource.hpp"

#include <QCamera>
#include <QCameraDevice>
#include <QCoreApplication>
#include <QImage>
#include <QMediaCaptureSession>
#include <QMediaDevices>
#include <QMetaObject>
#include <QTimer>
#include <QVideoFrame>
#include <QVideoSink>
#if QT_VERSION >= QT_VERSION_CHECK(6, 5, 0)
#include <QPermissions>
#endif

#include <algorithm>
#include <cstring>
#include <mutex>
#include <optional>
#include <utility>

namespace micro_robot {

class QtCameraSource::Backend final : public QObject {
public:
    explicit Backend(QtCameraSource& owner) : owner_(owner) {}
    ~Backend() override { releaseCapture(); }

    void start(std::uint64_t generation, const std::string& id)
    {
        generation_ = generation;
        desired_ = true;
        failed_ = false;
        lastState_.reset();
        mailbox_ = std::make_shared<Mailbox>();
        // Recreate the native objects on each start, including after a failure.
        session_.reset();
        camera_.reset();
        video_.reset();
        const auto inputs = QMediaDevices::videoInputs();
        const auto found = std::find_if(inputs.begin(), inputs.end(), [&](const QCameraDevice& input) {
            return input.id().toHex().toStdString() == id;
        });
        if (found == inputs.end()) {
            fail(CameraErrorCode::Unavailable, "Selected camera is unavailable");
            return;
        }
        camera_ = std::make_unique<QCamera>(*found);
        video_ = std::make_unique<QVideoSink>();
        session_ = std::make_unique<QMediaCaptureSession>();
        session_->setCamera(camera_.get());
        session_->setVideoSink(video_.get());
        connect(camera_.get(), &QCamera::activeChanged, this, [this](bool active) {
            if (failed_) return;
            if (active && desired_) postState(CameraState::Streaming);
            else if (!active && desired_) fail(CameraErrorCode::Unavailable, "Camera capture stopped unexpectedly");
        });
        connect(camera_.get(), &QCamera::errorOccurred, this,
                [this](QCamera::Error, const QString& message) {
            if (!failed_) fail(CameraErrorCode::BackendFailure, message.toStdString());
        });
        // The multimedia backend may emit from another thread. Retain only its
        // newest native frame before scheduling conversion on our worker.
        const auto nativeMailbox = std::make_shared<NativeMailbox>();
        nativeMailbox->target = this;
        connect(video_.get(), &QVideoSink::videoFrameChanged, this,
                [nativeMailbox](const QVideoFrame& frame) {
            // Teardown invalidates target under the same lock. A callback already
            // in flight can retain the mailbox without accessing a dead backend.
            std::lock_guard lock{nativeMailbox->mutex};
            auto* target = nativeMailbox->target;
            if (!target) return;
            nativeMailbox->frame = frame;
            nativeMailbox->capturedAt = MonotonicClock::now();
            if (nativeMailbox->scheduled) return;
            nativeMailbox->scheduled = true;
            QMetaObject::invokeMethod(target, [target, nativeMailbox] {
                QVideoFrame latest;
                Timestamp capturedAt;
                {
                    std::lock_guard lock{nativeMailbox->mutex};
                    latest = std::move(nativeMailbox->frame);
                    capturedAt = nativeMailbox->capturedAt;
                    nativeMailbox->scheduled = false;
                }
                if (nativeMailbox != target->nativeMailbox_) return;
                target->receiveFrame(latest, capturedAt);
            }, Qt::QueuedConnection);
        }, Qt::DirectConnection);
        nativeMailbox_ = nativeMailbox;
        camera_->start();
    }

    void stop(std::uint64_t generation)
    {
        generation_ = generation;
        lastState_.reset();
        desired_ = false;
        failed_ = false;
        releaseCapture();
        postState(CameraState::Stopped);
    }

private:
    struct NativeMailbox {
        std::mutex mutex;
        QVideoFrame frame;
        Backend* target{}; // Protected by mutex; nulled before native teardown.
        Timestamp capturedAt{};
        bool scheduled{};
    };

    struct Mailbox {
        std::mutex mutex;
        FramePtr frame;
        bool scheduled{};
    };

    void postState(CameraState state)
    {
        if (lastState_ == state) return;
        lastState_ = state;
        QMetaObject::invokeMethod(&owner_, [owner = &owner_, generation = generation_, state] {
            owner->receiveState(generation, state);
        }, Qt::QueuedConnection);
    }

    void reportError(CameraErrorCode code, const std::string& message)
    {
        const CameraError error{code, message, MonotonicClock::now()};
        QMetaObject::invokeMethod(&owner_, [owner = &owner_, generation = generation_, error] {
            owner->receiveError(generation, error);
        }, Qt::QueuedConnection);
    }

    void releaseCapture()
    {
        // Disconnect before stopping: inactive/error signals during teardown are
        // not a new capture failure. Release native resources on the worker before
        // acknowledging quiescence to the owner.
        if (nativeMailbox_) {
            std::lock_guard lock{nativeMailbox_->mutex};
            nativeMailbox_->target = nullptr;
        }
        if (camera_) {
            QObject::disconnect(camera_.get(), nullptr, this, nullptr);
            camera_->stop();
        }
        session_.reset();
        camera_.reset();
        video_.reset();
        nativeMailbox_.reset();
        mailbox_.reset();
    }

    void fail(CameraErrorCode code, const std::string& message)
    {
        if (failed_) return;
        failed_ = true;
        desired_ = false;
        // Failure may arrive inside a native object's signal. Defer destruction
        // until that callback has returned; a concurrent stop supersedes it.
        QTimer::singleShot(0, this, [this, generation = generation_, code, message] {
            if (generation != generation_) return;
            releaseCapture();
            reportError(code, message);
            postState(CameraState::Fault);
        });
    }

    void receiveFrame(const QVideoFrame& native, Timestamp capturedAt)
    {
        if (!desired_ || failed_ || lastState_ != CameraState::Streaming) return;
        const auto image = native.toImage().convertToFormat(QImage::Format_RGBA8888);
        if (image.isNull()) {
            reportError(CameraErrorCode::InvalidFrame, "Could not convert camera frame");
            return;
        }
        const auto stride = static_cast<std::size_t>(image.bytesPerLine());
        std::vector<std::byte> pixels(stride * static_cast<std::size_t>(image.height()));
        std::memcpy(pixels.data(), image.constBits(), pixels.size());
        auto frame = std::make_shared<const Frame>(Frame{
            sequence_++, capturedAt, static_cast<std::uint32_t>(image.width()),
            static_cast<std::uint32_t>(image.height()), stride, PixelFormat::Rgba8,
            std::make_shared<const std::vector<std::byte>>(std::move(pixels))});
        const auto mailbox = mailbox_;
        {
            std::lock_guard lock{mailbox->mutex};
            mailbox->frame = std::move(frame);
            if (mailbox->scheduled) return;
            mailbox->scheduled = true;
        }
        QMetaObject::invokeMethod(&owner_, [owner = &owner_, mailbox, generation = generation_] {
            FramePtr latest;
            {
                std::lock_guard lock{mailbox->mutex};
                latest = std::move(mailbox->frame);
                mailbox->scheduled = false;
            }
            if (generation != owner->generation_ || owner->state_ != CameraState::Streaming) return;
            if (auto sink = owner->sink_.lock()) sink->onFrame(std::move(latest));
        }, Qt::QueuedConnection);
    }

    QtCameraSource& owner_;
    std::unique_ptr<QVideoSink> video_;
    std::unique_ptr<QCamera> camera_;
    std::unique_ptr<QMediaCaptureSession> session_;
    std::shared_ptr<NativeMailbox> nativeMailbox_;
    std::shared_ptr<Mailbox> mailbox_;
    std::optional<CameraState> lastState_;
    std::uint64_t generation_{};
    std::uint64_t sequence_{};
    bool desired_{};
    bool failed_{};
};

QtCameraSource::QtCameraSource(CameraDevice device) : device_(std::move(device))
{
    backend_ = new Backend(*this);
    backend_->moveToThread(&worker_);
    connect(&worker_, &QThread::finished, backend_, &QObject::deleteLater);
    worker_.start();
}

QtCameraSource::~QtCameraSource()
{
    sink_.reset();
    worker_.quit();
    worker_.wait();
}

void QtCameraSource::setEventSink(std::weak_ptr<ICameraEvents> sink) { sink_ = std::move(sink); }
CameraInfo QtCameraSource::info() const { return {device_.id, device_.displayName}; }
CameraState QtCameraSource::state() const { return state_; }

void QtCameraSource::start()
{
    if (startRequested_ || stopRequested_ || state_ == CameraState::Streaming) return;
    startRequested_ = true;
    const auto generation = ++generation_;
    QTimer::singleShot(0, this, [this, generation] {
        if (generation != generation_) return;
        receiveState(generation, CameraState::Starting);
        // A state callback may cancel this request.
        if (generation != generation_) return;
#if QT_VERSION >= QT_VERSION_CHECK(6, 5, 0)
        // An absent device must fail without prompting for camera permission.
        const auto inputs = QMediaDevices::videoInputs();
        if (std::none_of(inputs.begin(), inputs.end(), [this](const QCameraDevice& input) {
                return input.id().toHex().toStdString() == device_.id;
            })) {
            beginStart(generation);
            return;
        }
        const QCameraPermission permission;
        const auto status = QCoreApplication::instance()->checkPermission(permission);
        if (status == Qt::PermissionStatus::Undetermined) {
            QCoreApplication::instance()->requestPermission(permission, this,
                [this, generation](const QPermission& result) {
                    if (generation != generation_) return;
                    if (result.status() == Qt::PermissionStatus::Granted) beginStart(generation);
                    else {
                        receiveError(generation, {CameraErrorCode::PermissionDenied,
                            "Camera permission denied; allow camera access in system settings",
                            MonotonicClock::now()});
                        receiveState(generation, CameraState::Fault);
                    }
                });
            return;
        }
        if (status == Qt::PermissionStatus::Denied) {
            receiveError(generation, {CameraErrorCode::PermissionDenied,
                "Camera permission denied; allow camera access in system settings", MonotonicClock::now()});
            receiveState(generation, CameraState::Fault);
            return;
        }
#endif
        beginStart(generation);
    });
}

void QtCameraSource::beginStart(std::uint64_t generation)
{
    QMetaObject::invokeMethod(backend_, [backend = backend_, generation, id = device_.id] {
        backend->start(generation, id);
    }, Qt::QueuedConnection);
}

void QtCameraSource::stop()
{
    if (stopRequested_ || (!startRequested_ && state_ != CameraState::Streaming
                           && state_ != CameraState::Starting)) return;
    startRequested_ = false;
    stopRequested_ = true;
    const auto generation = ++generation_;
    QTimer::singleShot(0, this, [this, generation] {
        if (generation != generation_) return;
        receiveState(generation, CameraState::Stopping);
        QMetaObject::invokeMethod(backend_, [backend = backend_, generation] {
            backend->stop(generation);
        }, Qt::QueuedConnection);
    });
}

void QtCameraSource::receiveState(std::uint64_t generation, CameraState state)
{
    if (generation != generation_) return;
    state_ = state;
    if (state != CameraState::Starting) startRequested_ = false;
    if (state == CameraState::Stopped || state == CameraState::Fault) stopRequested_ = false;
    if (auto sink = sink_.lock()) sink->onCameraStateChanged(state, MonotonicClock::now());
}

void QtCameraSource::receiveError(std::uint64_t generation, const CameraError& error)
{
    if (generation != generation_) return;
    if (auto sink = sink_.lock()) sink->onCameraError(error);
}

bool QtCameraSourceFactory::supports(const CameraDevice& device) const
{
    const auto id = QByteArray::fromStdString(device.id);
    return !id.isEmpty() && id.size() % 2 == 0 && QByteArray::fromHex(id).toHex() == id;
}

std::unique_ptr<ICameraSource> QtCameraSourceFactory::create(const CameraDevice& device)
{
    if (!supports(device)) return nullptr;
    return std::make_unique<QtCameraSource>(device);
}

} // namespace micro_robot
