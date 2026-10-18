#include "camera/RecordedVideoSource.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace micro_robot {

class RecordedVideoSource::Impl {
public:
    explicit Impl(RecordedVideoConfig value) : config(std::move(value))
    {
        if (config.file.empty() || config.frameInterval <= Duration::zero()) {
            throw std::invalid_argument("Replay requires a file and positive frame interval");
        }
        config.file = std::filesystem::absolute(config.file).lexically_normal();
        cameraInfo = {"recorded-video:" + config.file.generic_string(),
                      config.file.filename().string()};
    }

    void changeState(CameraState value, Timestamp now)
    {
        if (currentState == value) {
            return;
        }
        currentState = value;
        if (auto recipient = sink.lock()) {
            recipient->onCameraStateChanged(value, now);
        }
    }

    void finish(CameraState value, Timestamp now)
    {
        capture.release();
        desiredActive = false;
        changeState(value, now);
    }

    void fail(CameraErrorCode code, std::string message, Timestamp now)
    {
        finish(CameraState::Fault, now);
        if (auto recipient = sink.lock()) {
            recipient->onCameraError({code, std::move(message), now});
        }
    }

    bool open(Timestamp now)
    {
        std::error_code error;
        if (!std::filesystem::is_regular_file(config.file, error)) {
            fail(CameraErrorCode::Unavailable, "Replay file is unavailable: "
                 + config.file.string(), now);
            return false;
        }
        if (!capture.open(config.file.string())) {
            fail(CameraErrorCode::BackendFailure, "Cannot decode replay file: "
                 + config.file.string(), now);
            return false;
        }
        return true;
    }

    void advance(Timestamp now)
    {
        if (advancing) {
            throw std::logic_error("Recursive recorded-video advancement");
        }
        if (lastAdvance && now < *lastAdvance) {
            throw std::invalid_argument("Replay time cannot go backward");
        }
        struct Guard {
            bool& active;
            explicit Guard(bool& value) : active(value) { active = true; }
            ~Guard() { active = false; }
        } guard{advancing};
        lastAdvance = now;

        if (!desiredActive) {
            if (cancelledStart) {
                cancelledStart = false;
                if (auto recipient = sink.lock()) recipient->onCameraStateChanged(currentState, now);
            }
            if (currentState == CameraState::Streaming) {
                finish(CameraState::Stopped, now);
            }
            return;
        }

        try {
            if (currentState != CameraState::Streaming) {
                if (!open(now)) {
                    return;
                }
                nextCapture = lastCapture ? std::max(now, *lastCapture + config.frameInterval) : now;
                framesInPass = 0;
                changeState(CameraState::Streaming, now);
            }
            // A callback can request stop without recursive dispatch.
            if (!desiredActive) {
                finish(CameraState::Stopped, now);
                return;
            }
            if (now < nextCapture) {
                return;
            }
            if (nextCapture > Timestamp::max() - config.frameInterval
                || nextSequence == std::numeric_limits<std::uint64_t>::max()) {
                fail(CameraErrorCode::BackendFailure, "Replay timeline exhausted", now);
                return;
            }

            cv::Mat decoded;
            if (!capture.read(decoded)) {
                if (framesInPass == 0) {
                    fail(CameraErrorCode::BackendFailure, "Replay contains no decodable frames", now);
                    return;
                }
                if (!config.loop) {
                    finish(CameraState::Stopped, now);
                    return;
                }
                // Reopen rather than relying on backend-specific seek behavior.
                capture.release();
                if (!open(now)) {
                    return;
                }
                framesInPass = 0;
                if (!capture.read(decoded)) {
                    fail(CameraErrorCode::BackendFailure, "Cannot restart replay", now);
                    return;
                }
            }

            if (decoded.empty() || decoded.depth() != CV_8U
                || (decoded.channels() != 1 && decoded.channels() != 3 && decoded.channels() != 4)) {
                fail(CameraErrorCode::InvalidFrame, "Unsupported decoded pixel layout", now);
                return;
            }
            cv::Mat rgb;
            if (decoded.channels() == 1) {
                cv::cvtColor(decoded, rgb, cv::COLOR_GRAY2RGB);
            } else {
                cv::cvtColor(decoded, rgb, decoded.channels() == 3
                             ? cv::COLOR_BGR2RGB : cv::COLOR_BGRA2RGB);
            }
            const auto stride = static_cast<std::size_t>(rgb.cols) * 3;
            if (static_cast<std::size_t>(rgb.rows) > std::numeric_limits<std::size_t>::max() / stride) {
                fail(CameraErrorCode::InvalidFrame, "Decoded frame is too large", now);
                return;
            }
            std::vector<std::byte> pixels(stride * static_cast<std::size_t>(rgb.rows));
            for (int row = 0; row < rgb.rows; ++row) {
                std::memcpy(pixels.data() + static_cast<std::size_t>(row) * stride,
                            rgb.ptr(row), stride);
            }
            auto frame = std::make_shared<const Frame>(Frame{
                nextSequence++, nextCapture, static_cast<std::uint32_t>(rgb.cols),
                static_cast<std::uint32_t>(rgb.rows), stride, PixelFormat::Rgb8,
                std::make_shared<const std::vector<std::byte>>(std::move(pixels))
            });
            ++framesInPass;
            lastCapture = nextCapture;
            nextCapture += config.frameInterval;
            if (auto recipient = sink.lock()) {
                recipient->onFrame(std::move(frame));
            }
        } catch (const cv::Exception& error) {
            fail(CameraErrorCode::BackendFailure, error.what(), now);
        }
    }

    RecordedVideoConfig config;
    CameraInfo cameraInfo;
    std::weak_ptr<ICameraEvents> sink;
    cv::VideoCapture capture;
    CameraState currentState{CameraState::Stopped};
    bool desiredActive{};
    bool cancelledStart{};
    bool advancing{};
    std::optional<Timestamp> lastAdvance;
    std::optional<Timestamp> lastCapture;
    Timestamp nextCapture{};
    std::uint64_t nextSequence{};
    std::uint64_t framesInPass{};
};

RecordedVideoSource::RecordedVideoSource(RecordedVideoConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}
RecordedVideoSource::~RecordedVideoSource() = default;
void RecordedVideoSource::setEventSink(std::weak_ptr<ICameraEvents> sink)
{
    impl_->sink = std::move(sink);
}
CameraInfo RecordedVideoSource::info() const { return impl_->cameraInfo; }
CameraState RecordedVideoSource::state() const { return impl_->currentState; }
void RecordedVideoSource::start() { impl_->desiredActive = true; impl_->cancelledStart = false; }
void RecordedVideoSource::stop()
{
    impl_->cancelledStart = impl_->cancelledStart
        || (impl_->desiredActive && impl_->currentState != CameraState::Streaming);
    impl_->desiredActive = false;
}
void RecordedVideoSource::advanceTo(Timestamp now) { impl_->advance(now); }

} // namespace micro_robot
