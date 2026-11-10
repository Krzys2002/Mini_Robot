#include "tracking/RobotTracker.hpp"
#include "common/Angle.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <stdexcept>
using namespace micro_robot;
namespace {
struct Sample { double t; Pose2D truth, measured; };
double squaredError(const Pose2D& a, const Pose2D& b, bool heading) {
    if (heading) return std::pow(normalizeAngle(Radians{a.heading.value-b.heading.value}).value, 2);
    return std::pow(a.x.value-b.x.value, 2)+std::pow(a.y.value-b.y.value, 2);
}
std::vector<Sample> synthetic(int scenario) {
    std::vector<Sample> result;
    for (int i=0; i<600; ++i) {
        const double t=i*.02;
        Pose2D truth;
        if (scenario == 1) { truth.x=Meters{.1*t}; truth.heading=Radians{.3*t}; }
        if (scenario == 2) {
            truth.x=Meters{.2*std::sin(t)}; truth.y=Meters{.1*std::sin(2*t)};
            truth.heading=Radians{3+.4*std::sin(t)};
        }
        auto measurement=truth;
        measurement.x.value += .01*std::sin(17*i)+.004*std::cos(7*i);
        measurement.y.value += .008*std::sin(13*i);
        measurement.heading=normalizeAngle(Radians{truth.heading.value+.02*std::sin(11*i)});
        result.push_back({t, truth, measurement});
    }
    return result;
}
std::vector<Sample> read(const std::filesystem::path& path) {
    std::ifstream in(path); if (!in) throw std::runtime_error("Cannot open replay CSV");
    std::vector<Sample> result; std::string line;
    std::getline(in,line);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line != "t,x,y,heading,true_x,true_y,true_heading")
        throw std::runtime_error("Invalid replay CSV header");
    while (std::getline(in,line)) {
        std::replace(line.begin(),line.end(),',',' ');
        std::istringstream row(line); Sample s{};
        if (!(row >> s.t >> s.measured.x.value >> s.measured.y.value >> s.measured.heading.value
            >> s.truth.x.value >> s.truth.y.value >> s.truth.heading.value))
            throw std::runtime_error("Malformed replay CSV row");
        row >> std::ws; if (!row.eof()) throw std::runtime_error("Extra CSV content");
        for (double v : {s.t,s.measured.x.value,s.measured.y.value,s.measured.heading.value,
                         s.truth.x.value,s.truth.y.value,s.truth.heading.value})
            if (!std::isfinite(v)) throw std::runtime_error("Non-finite replay value");
        if (s.t < 0 || s.t > 86400 || (!result.empty() && s.t <= result.back().t))
            throw std::runtime_error("Replay times must increase within one day");
        result.push_back(s);
    }
    if (result.size()<50) throw std::runtime_error("Need at least 50 replay samples");
    return result;
}
void compare(const std::string& name, const std::vector<Sample>& data, const std::filesystem::path& dir,
             std::ostream& metrics) {
    std::ofstream input(dir/(name+"-input.csv"));
    input << std::setprecision(17) << "t,x,y,heading,true_x,true_y,true_heading\n";
    for (const auto& s:data) input << s.t << ',' << s.measured.x.value << ',' << s.measured.y.value << ','
        << s.measured.heading.value << ',' << s.truth.x.value << ',' << s.truth.y.value << ',' << s.truth.heading.value << '\n';
    input.flush(); if (!input) throw std::runtime_error("Cannot write replay input");
    for (auto kind : {PoseFilterKind::Exponential,PoseFilterKind::Kalman}) {
        WorldModel world;
        RobotTracker tracker(world, {std::chrono::milliseconds{250},std::chrono::seconds{1}}, {.2,.2,kind});
        const std::string filter=kind==PoseFilterKind::Kalman ? "kalman" : "exponential";
        std::ofstream output(dir/(name+"-"+filter+".csv"));
        output << std::setprecision(17) << "t,x,y,heading\n";
        std::vector<Pose2D> poses;
        double positionError=0, headingError=0;
        constexpr std::size_t warmup=25;
        for (std::size_t i=0;i<data.size();++i) {
            const auto& s=data[i];
            const auto time=Timestamp{}+std::chrono::duration_cast<Duration>(std::chrono::duration<double>{s.t});
            if (tracker.observe(RobotId{1},s.measured,time,time)!=TrackerUpdate::Applied)
                throw std::runtime_error("Replay measurement rejected");
            const auto pose=*world.robot(RobotId{1})->pose;
            poses.push_back(pose);
            output << s.t << ',' << pose.x.value << ',' << pose.y.value << ',' << pose.heading.value << '\n';
            if (i>=warmup) { positionError+=squaredError(pose,s.truth,false); headingError+=squaredError(pose,s.truth,true); }
        }
        output.flush(); if (!output) throw std::runtime_error("Cannot write filtered output");
        // Best delayed-reference fit over 0..20 samples, evaluated after warmup.
        std::size_t lag=0; double best=1e300;
        for (std::size_t shift=0;shift<=20;++shift) {
            double error=0;
            for (std::size_t i=warmup;i<data.size();++i) error+=squaredError(poses[i],data[i-shift].truth,false);
            if (error<best) {best=error;lag=shift;}
        }
        double lagSeconds=0;
        for (std::size_t i=warmup;i<data.size();++i) lagSeconds+=data[i].t-data[i-lag].t;
        const double count=static_cast<double>(data.size()-warmup);
        metrics << name << ',' << filter << ',' << data.size() << ',' << std::sqrt(positionError/count)
            << ',' << std::sqrt(headingError/count) << ',' << lagSeconds/count << '\n';
    }
}
}
int main(int argc,char** argv) {
    try {
        if (argc<2 || argc>3) {std::cerr << "Usage: micro_robot_filter_comparison OUTPUT_DIRECTORY [REPLAY.csv]\n";return 2;}
        const std::filesystem::path dir{argv[1]}; std::filesystem::create_directories(dir);
        std::ofstream metrics(dir/"metrics.csv"); if (!metrics) throw std::runtime_error("Cannot write metrics");
        metrics << std::setprecision(10) << "scenario,filter,samples,position_rmse_m,heading_rmse_rad,position_lag_s\n";
        std::ofstream config(dir/"configuration.txt");
        config << "exponential alpha_position=0.2 alpha_heading=0.2\nkalman q_position=0.05 q_heading=0.2 r_position=0.0001 r_heading=0.0004\n"
            << "warmup_samples=25 lag_search_samples=0..20 thresholds_stale_s=0.25 lost_s=1\n"
            << "synthetic_dt_s=0.02 synthetic_samples=600 deterministic_sinusoidal_noise; lag is best delayed-reference position fit\n";
        if (argc==3) compare("replay",read(argv[2]),dir,metrics);
        else {compare("stationary",synthetic(0),dir,metrics);compare("constant-motion",synthetic(1),dir,metrics);compare("turning",synthetic(2),dir,metrics);}
        metrics.flush(); config.flush();
        if (!metrics || !config) throw std::runtime_error("Cannot write results/configuration");
        return 0;
    } catch (const std::exception& e) {std::cerr << e.what() << '\n';return 1;}
}
