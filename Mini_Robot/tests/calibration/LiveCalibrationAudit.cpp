// Synthetic, offscreen performance evidence only. No physical camera or accuracy claim.
#include "app/CalibrationController.hpp"
#include "app/VisionController.hpp"
#include "calibration/OpenCvCalibrationDetector.hpp"
#include "calibration/OpenCvCalibrationCornerTracker.hpp"
#include "calibration/OpenCvCalibrationUndistorter.hpp"
#include "IntrinsicFixtures.hpp"
#include "calibration/OpenCvIntrinsicCalibrationSolver.hpp"
#include "ui/CalibrationWizard.hpp"
#include "vision/OpenCvAprilTagDetector.hpp"
#include <QApplication>
#include <QEventLoop>
#include <QTimer>
#include <algorithm>
#include <iostream>

using namespace micro_robot;
namespace {
struct Timing {
    int count{};
    double totalMs{}, maximumMs{};
    void add(Timestamp start) {
        const double ms = std::chrono::duration<double, std::milli>(MonotonicClock::now()-start).count();
        ++count; totalMs += ms; maximumMs = std::max(maximumMs, ms);
    }
};
class AuditStore final : public ICalibrationProfileStore {
public:
    ProfileLoadOutcome load(std::string_view id, std::uint32_t width, std::uint32_t height) override {
        auto c=test::validIntrinsicCalibration();
        c.sourceId=id; c.width=width; c.height=height;
        c.cameraMatrix={static_cast<double>(width),0,width/2.0,0,static_cast<double>(width),height/2.0,0,0,1};
        return {ProfileLoadState::Loaded,c,{}};
    }
    std::string save(const IntrinsicCalibration&) override { return "Audit never saves"; }
};
class Chess final : public ICalibrationDetector {
public:
    Timing timing; // Read only after the worker has stopped.
    OpenCvCalibrationDetector detector;
    CalibrationDetection detect(const Frame& f, const CalibrationTarget& t) override {
        const auto start = MonotonicClock::now();
        auto result = detector.detect(f,t); timing.add(start); return result;
    }
};
class Flow final : public ICalibrationCornerTracker {
public:
    Timing timing;
    OpenCvCalibrationCornerTracker tracker;
    std::vector<ImageCorner> track(const Frame& from, const Frame& to, const std::vector<ImageCorner>& corners) override {
        const auto start=MonotonicClock::now();
        auto result=tracker.track(from,to,corners); timing.add(start); return result;
    }
};
class Tags final : public IMarkerDetector {
public:
    Timing timing;
    OpenCvAprilTagDetector detector;
    MarkerDetections detect(const Frame& f) override {
        const auto start = MonotonicClock::now();
        auto result = detector.detect(f); timing.add(start); return result;
    }
};
}
int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    Logger logger;
    for (auto [width, height] : {std::pair{640,480}, {1280,720}, {1920,1080}}) {
        for (const auto overlay : {CalibrationLivePreviewMode::Hold, CalibrationLivePreviewMode::Track, CalibrationLivePreviewMode::Matched}) {
        const bool tracking = overlay == CalibrationLivePreviewMode::Track;
        for (int mode : {0, 1, 2}) {
            const bool visible=mode!=0, corrected=mode==2;
            auto chess = std::make_unique<Chess>(); auto* c = chess.get();
            auto tags = std::make_unique<Tags>(); auto* t = tags.get();
            CalibrationController calibration(std::move(chess),
                std::make_unique<OpenCvIntrinsicCalibrationSolver>(), logger.channel("audit"));
            VisionController vision(std::move(tags), logger.channel("tags"));
            auto flow=std::make_unique<Flow>(); auto* f=flow.get();
            calibration.setLiveTracker(std::move(flow));
            calibration.setLivePreviewMode(overlay);
            if (corrected) calibration.setProfileServices(std::make_unique<AuditStore>(),
                std::make_unique<OpenCvCalibrationUndistorter>());
            CalibrationWizard wizard(calibration);
            calibration.setInput(QStringLiteral("synthetic"), true, corrected);
            calibration.restart();
            vision.setInput(QStringLiteral("synthetic"), true);
            // Fixed 10x7-square board with white margins, scaled to each image.
            auto pixels = std::make_shared<std::vector<std::byte>>(width*height, std::byte{255});
            const int square = std::min(width/14, height/11);
            const int left=(width-10*square)/2, top=(height-7*square)/2;
            for (int y=0; y<7*square; ++y) for (int x=0; x<10*square; ++x)
                if ((x/square+y/square)%2 == 0) (*pixels)[(top+y)*width+left+x] = std::byte{0};
            auto shifted=std::make_shared<std::vector<std::byte>>(width*height,std::byte{255});
            for (int y=0; y<height; ++y)
                std::copy_n(pixels->begin()+y*width,width-2,shifted->begin()+y*width+2);
            if (visible) wizard.show();
            QEventLoop loop;
            QTimer frames, heartbeat;
            std::uint64_t sequence = 0, lastShown=0, lastHeld=0;
            int videoFrames=0, overlayFrames=0, heldFrames=0, uniqueHeld=0, rewinds=0;
            QObject::connect(&calibration, &CalibrationController::previewChanged, &loop,
                [&](FramePtr frame, const std::vector<ImageCorner>& corners, bool, bool held) {
                    if (!frame) return;
                    if (frame->sequence<lastShown) ++rewinds;
                    if (frame->sequence>lastShown) ++videoFrames;
                    lastShown=frame->sequence;
                    if (!corners.empty()) ++overlayFrames;
                    if (held) {
                        ++heldFrames;
                        if (frame->sequence!=lastHeld) ++uniqueHeld;
                        lastHeld=frame->sequence;
                    }
                });
            double maximumGapMs = 0;
            auto lastBeat = MonotonicClock::now();
            QObject::connect(&heartbeat, &QTimer::timeout, [&] {
                const auto now=MonotonicClock::now();
                maximumGapMs=std::max(maximumGapMs, std::chrono::duration<double,std::milli>(now-lastBeat).count());
                lastBeat=now;
            });
            QObject::connect(&frames, &QTimer::timeout, [&] {
                auto frame=std::make_shared<Frame>(Frame{++sequence, MonotonicClock::now(),
                    static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height),
                    static_cast<std::size_t>(width), PixelFormat::Gray8, sequence%2 ? pixels : shifted});
                calibration.receiveFrame(frame); vision.receiveFrame(frame);
                if (corrected && calibration.canUndistort() && !calibration.undistortedPreview())
                    calibration.setUndistortedPreview(true);
            });
            frames.start(33); heartbeat.start(5);
            QTimer::singleShot(2000, &loop, &QEventLoop::quit);
            loop.exec(); frames.stop(); heartbeat.stop();
            wizard.hide();
            calibration.shutdown(); vision.shutdown();
            while (!calibration.isStopped() || !vision.isStopped()) QApplication::processEvents();
            std::cout << width << 'x' << height << " overlay=" << static_cast<int>(overlay) << " tracking=" << tracking << " visible=" << visible << " corrected=" << corrected << " inputs=" << sequence
                      << " chess_calls=" << c->timing.count << " chess_mean_ms="
                      << (c->timing.count ? c->timing.totalMs/c->timing.count : 0)
                      << " chess_max_ms=" << c->timing.maximumMs << " tag_calls=" << t->timing.count
                      << " tag_mean_ms=" << (t->timing.count ? t->timing.totalMs/t->timing.count : 0)
                      << " tag_max_ms=" << t->timing.maximumMs
                      << " video_frames=" << videoFrames << " overlay_frames=" << overlayFrames
                      << " held_updates=" << heldFrames << " unique_held=" << uniqueHeld << " rewinds=" << rewinds
                      << " flow_calls=" << f->timing.count << " flow_mean_ms="
                      << (f->timing.count ? f->timing.totalMs/f->timing.count : 0)
                      << " heartbeat_max_gap_ms=" << maximumGapMs << '\n';
        }
        }
    }
}
