#include "camera/V4l2CameraSource.hpp"
#include "camera/V4l2Device.hpp"

#include <QLoggingCategory>
#include <QMetaObject>
#include <QString>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <limits>
#include <mutex>
#include <optional>
#include <system_error>
#include <utility>
#include <vector>

namespace micro_robot {
namespace {
Q_LOGGING_CATEGORY(captureLog, "micro_robot.camera.capture")

// Upper bound of one blocking wait on the worker. Stop requests and the
// CameraFrameTimeout check are served between waits.
constexpr std::chrono::milliseconds FramePollInterval{100};

bool nodeExists(const std::string& path)
{
    std::error_code error;
    return std::filesystem::exists(path, error);
}

} // namespace

class V4l2CameraSource::Backend final : public QObject {
public:
    Backend(V4l2CameraSource& owner, std::string path, CameraCaptureSettings settings)
        : owner_(owner), path_(std::move(path)), settings_(settings) {}
    ~Backend() override { releaseCapture(); }

    void start(std::uint64_t generation)
    {
        generation_ = generation;
        lastState_.reset();
        // Reopen on each start, including after a failure.
        releaseCapture();
        if (!checkNode()) return;
        try {
            streams_.resize(1);
            auto& capture = streams_.front();
            if (!capture.open(path_, cv::CAP_V4L2)) {
                fail(CameraErrorCode::BackendFailure, "OpenCV V4L2 backend cannot open " + path_
                     + "; it may be in use by another application", CameraState::Fault);
                return;
            }
            if (settings_.resolution) requestResolution(capture, *settings_.resolution);
            qCInfo(captureLog) << "event=camera_capture_opened" << "backend=v4l2"
                               << "id=" << QString::fromStdString(path_)
                               << "width=" << capture.get(cv::CAP_PROP_FRAME_WIDTH)
                               << "height=" << capture.get(cv::CAP_PROP_FRAME_HEIGHT);
        } catch (const cv::Exception& error) {
            fail(CameraErrorCode::BackendFailure, "Cannot open " + path_ + ": " + error.what(),
                 CameraState::Fault);
            return;
        }
        active_ = true;
        mailbox_ = std::make_shared<Mailbox>();
        // Also bounds startup: a device which never delivers a frame faults.
        lastProgress_ = MonotonicClock::now();
        scheduleRead();
    }

    void stop(std::uint64_t generation)
    {
        generation_ = generation;
        lastState_.reset();
        active_ = false;
        releaseCapture();
        postState(CameraState::Stopped);
    }

private:
    struct Mailbox {
        std::mutex mutex;
        FramePtr frame;
        bool scheduled{};
    };

    // Classifies the node before OpenCV opens it: OpenCV reports only success or
    // failure, which cannot distinguish a missing device from a permission error.
    bool checkNode()
    {
        using Status = V4l2NodeProbe::Status;
        const auto probe = probeV4l2Node(path_);
        switch (probe.status) {
        case Status::Capture:
            return true;
        case Status::Missing:
            fail(CameraErrorCode::Unavailable, "Selected camera is unavailable (" + path_ + ")",
                 CameraState::Fault);
            return false;
        case Status::AccessDenied:
            fail(CameraErrorCode::PermissionDenied, "Camera permission denied for " + path_
                 + "; grant your user access to video devices (for example the 'video' group) and retry",
                 CameraState::Fault);
            return false;
        case Status::Busy:
            fail(CameraErrorCode::BackendFailure, "Camera " + path_ + " is busy; close other "
                 "applications using it and retry", CameraState::Fault);
            return false;
        case Status::NotCapture:
            fail(CameraErrorCode::BackendFailure, path_ + " is not a video capture device",
                 CameraState::Fault);
            return false;
        case Status::Failed:
            break;
        }
        fail(CameraErrorCode::BackendFailure, "Cannot open " + path_ + ": " + probe.detail,
             CameraState::Fault);
        return false;
    }

    // V4L2 drivers adjust an unsupported size to a mode they offer; report that.
    void requestResolution(cv::VideoCapture& capture, CameraResolution requested)
    {
        capture.set(cv::CAP_PROP_FRAME_WIDTH, requested.width);
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, requested.height);
        const auto width = capture.get(cv::CAP_PROP_FRAME_WIDTH);
        const auto height = capture.get(cv::CAP_PROP_FRAME_HEIGHT);
        if (width != requested.width || height != requested.height) {
            qCWarning(captureLog) << "event=camera_resolution_fallback" << "backend=v4l2"
                                  << "id=" << QString::fromStdString(path_)
                                  << "requested=" << QStringLiteral("%1x%2").arg(requested.width).arg(requested.height)
                                  << "actual=" << QStringLiteral("%1x%2").arg(width).arg(height);
        }
    }

    void scheduleRead()
    {
        QMetaObject::invokeMethod(this, [this, generation = generation_] {
            if (generation == generation_ && active_) readFrame();
        }, Qt::QueuedConnection);
    }

    void readFrame()
    {
        try {
            std::vector<int> ready;
            cv::Mat image;
            const auto timeout = std::chrono::nanoseconds{FramePollInterval}.count();
            if (cv::VideoCapture::waitAny(streams_, ready, timeout) && !ready.empty()
                && streams_.front().retrieve(image) && !image.empty()) {
                if (!publish(image, MonotonicClock::now())) return;
                lastProgress_ = MonotonicClock::now();
            } else if (!nodeExists(path_)) {
                fail(CameraErrorCode::Unavailable, "Selected camera was disconnected",
                     CameraState::Disconnected);
                return;
            } else if (MonotonicClock::now() - lastProgress_ >= CameraFrameTimeout) {
                fail(CameraErrorCode::BackendFailure,
                     "Camera stopped delivering valid frames; select a camera or retry Start",
                     CameraState::Fault);
                return;
            }
        } catch (const cv::Exception& error) {
            if (nodeExists(path_)) {
                fail(CameraErrorCode::BackendFailure, std::string{"Camera capture failed: "}
                     + error.what(), CameraState::Fault);
            } else {
                fail(CameraErrorCode::Unavailable, "Selected camera was disconnected",
                     CameraState::Disconnected);
            }
            return;
        }
        // Queued, so stop requests posted meanwhile run before the next wait.
        scheduleRead();
    }

    // Converts to owned packed RGB and hands the newest frame to the owner thread.
    bool publish(const cv::Mat& image, Timestamp capturedAt)
    {
        if (image.depth() != CV_8U
            || (image.channels() != 1 && image.channels() != 3 && image.channels() != 4)) {
            fail(CameraErrorCode::InvalidFrame, "Unsupported camera pixel layout", CameraState::Fault);
            return false;
        }
        cv::Mat rgb;
        if (image.channels() == 1) {
            cv::cvtColor(image, rgb, cv::COLOR_GRAY2RGB);
        } else {
            cv::cvtColor(image, rgb, image.channels() == 3 ? cv::COLOR_BGR2RGB : cv::COLOR_BGRA2RGB);
        }
        const auto stride = static_cast<std::size_t>(rgb.cols) * 3;
        if (static_cast<std::size_t>(rgb.rows) > std::numeric_limits<std::size_t>::max() / stride) {
            fail(CameraErrorCode::InvalidFrame, "Camera frame is too large", CameraState::Fault);
            return false;
        }
        std::vector<std::byte> pixels(stride * static_cast<std::size_t>(rgb.rows));
        for (int row = 0; row < rgb.rows; ++row) {
            std::memcpy(pixels.data() + static_cast<std::size_t>(row) * stride, rgb.ptr(row), stride);
        }
        auto frame = std::make_shared<const Frame>(Frame{
            sequence_++, capturedAt, static_cast<std::uint32_t>(rgb.cols),
            static_cast<std::uint32_t>(rgb.rows), stride, PixelFormat::Rgb8,
            std::make_shared<const std::vector<std::byte>>(std::move(pixels))});
        // Streaming is confirmed by the first frame and queued ahead of it.
        postState(CameraState::Streaming);
        const auto mailbox = mailbox_;
        {
            std::lock_guard lock{mailbox->mutex};
            mailbox->frame = std::move(frame);
            if (mailbox->scheduled) return true;
            mailbox->scheduled = true;
        }
        QMetaObject::invokeMethod(&owner_, [owner = &owner_, mailbox, generation = generation_] {
            FramePtr latest;
            {
                std::lock_guard lock{mailbox->mutex};
                latest = std::move(mailbox->frame);
                mailbox->scheduled = false;
            }
            if (!latest || generation != owner->generation_
                || owner->state_ != CameraState::Streaming) return;
            if (auto sink = owner->sink_.lock()) sink->onFrame(std::move(latest));
        }, Qt::QueuedConnection);
        return true;
    }

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

    // Releases the device before the terminal state is acknowledged to the owner.
    void fail(CameraErrorCode code, const std::string& message, CameraState terminal)
    {
        active_ = false;
        if (mailbox_) {
            // Invalidate already queued converted delivery immediately.
            std::lock_guard lock{mailbox_->mutex};
            mailbox_->frame.reset();
        }
        releaseCapture();
        reportError(code, message);
        postState(terminal);
    }

    void releaseCapture()
    {
        streams_.clear();
        mailbox_.reset();
    }

    V4l2CameraSource& owner_;
    const std::string path_;
    const CameraCaptureSettings settings_;
    // waitAny() takes a list of captures; this adapter always has zero or one.
    std::vector<cv::VideoCapture> streams_;
    std::shared_ptr<Mailbox> mailbox_;
    std::optional<CameraState> lastState_;
    Timestamp lastProgress_{};
    std::uint64_t generation_{};
    std::uint64_t sequence_{};
    bool active_{};
};

V4l2CameraSource::V4l2CameraSource(CameraDevice device, CameraCaptureSettings settings)
    : device_(std::move(device))
{
    backend_ = new Backend(*this, device_.id, settings);
    backend_->moveToThread(&worker_);
    connect(&worker_, &QThread::finished, backend_, &QObject::deleteLater);
    worker_.start();
}

V4l2CameraSource::~V4l2CameraSource()
{
    sink_.reset();
    worker_.quit();
    worker_.wait();
}

void V4l2CameraSource::setEventSink(std::weak_ptr<ICameraEvents> sink) { sink_ = std::move(sink); }
CameraInfo V4l2CameraSource::info() const { return {device_.id, device_.displayName}; }
CameraState V4l2CameraSource::state() const { return state_; }

void V4l2CameraSource::start()
{
    if (startRequested_ || stopRequested_ || state_ == CameraState::Streaming) return;
    startRequested_ = true;
    const auto generation = ++generation_;
    QMetaObject::invokeMethod(this, [this, generation] {
        if (generation != generation_) return;
        receiveState(generation, CameraState::Starting);
        // A state callback may cancel this request.
        if (generation != generation_) return;
        QMetaObject::invokeMethod(backend_, [backend = backend_, generation] {
            backend->start(generation);
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}

void V4l2CameraSource::stop()
{
    if (stopRequested_ || (!startRequested_ && state_ != CameraState::Streaming
                           && state_ != CameraState::Starting)) return;
    startRequested_ = false;
    stopRequested_ = true;
    const auto generation = ++generation_;
    QMetaObject::invokeMethod(this, [this, generation] {
        if (generation != generation_) return;
        receiveState(generation, CameraState::Stopping);
        QMetaObject::invokeMethod(backend_, [backend = backend_, generation] {
            backend->stop(generation);
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}

void V4l2CameraSource::receiveState(std::uint64_t generation, CameraState state)
{
    if (generation != generation_) return;
    state_ = state;
    if (state != CameraState::Starting) startRequested_ = false;
    if (state == CameraState::Stopped || state == CameraState::Disconnected
        || state == CameraState::Fault) stopRequested_ = false;
    if (auto sink = sink_.lock()) sink->onCameraStateChanged(state, MonotonicClock::now());
}

void V4l2CameraSource::receiveError(std::uint64_t generation, const CameraError& error)
{
    if (generation != generation_) return;
    if (auto sink = sink_.lock()) sink->onCameraError(error);
}

V4l2CameraSourceFactory::V4l2CameraSourceFactory(CameraCaptureSettings settings)
    : settings_(settings) {}

bool V4l2CameraSourceFactory::supports(const CameraDevice& device) const
{
    return !device.id.empty() && std::filesystem::path{device.id}.is_absolute();
}

std::unique_ptr<ICameraSource> V4l2CameraSourceFactory::create(const CameraDevice& device)
{
    if (!supports(device)) return nullptr;
    return std::make_unique<V4l2CameraSource>(device, settings_);
}

} // namespace micro_robot
