#!/usr/bin/env python3
"""Compare bounded calibration storage at a Git revision and the working tree.

Requires Git and a POSIX-style C++20 compiler. RSS uses ps when available (macOS/
Linux); absent RSS does not invalidate the portable retained-pixel measurements.
Compilation/output live in a temporary directory unless --output is specified.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import tempfile

HARNESS = r'''
#include "calibration/CalibrationCapture.hpp"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
using namespace micro_robot;
void checkpoint(const std::string& json) {
    std::cout << json << std::endl;
    std::string acknowledge;
    if (!std::getline(std::cin, acknowledge)) std::exit(2);
}
int main(int argc, char** argv) {
    if (argc != 6) return 2;
    const auto width = std::strtoul(argv[1], nullptr, 10);
    const auto height = std::strtoul(argv[2], nullptr, 10);
    const auto channels = std::strtoul(argv[3], nullptr, 10);
    const auto padding = std::strtoul(argv[4], nullptr, 10);
    const int cycles = std::atoi(argv[5]);
    CalibrationCaptureConfig config;
    config.duplicateDistance = .00001; // Distinct synthetic storage views, not accuracy evidence.
    CalibrationCapture session(config);
    for (int cycle = 0; cycle < cycles; ++cycle) {
        session.restart("storage-audit");
        checkpoint("{\"stage\":\"before\",\"cycle\":" + std::to_string(cycle) + "}");
        std::string failure;
        for (std::uint64_t n = 1; n <= config.maximumViews + 1; ++n) {
            const auto stride = width * channels + padding;
            auto pixels = std::make_shared<const std::vector<std::byte>>(stride * height, std::byte{127});
            auto frame = std::make_shared<Frame>(Frame{n, Timestamp{std::chrono::seconds{n}},
                static_cast<unsigned>(width), static_cast<unsigned>(height), stride,
                channels == 1 ? PixelFormat::Gray8 : channels == 3 ? PixelFormat::Rgb8 : PixelFormat::Rgba8,
                pixels});
            CalibrationDetection detection;
            detection.sharpness = 100;
            for (int row = 0; row < 6; ++row) for (int column = 0; column < 9; ++column)
                detection.corners.push_back({width * .5 + (column - 4) * static_cast<double>(width) / 25 + n,
                    height * .5 + (row - 2.5) * static_cast<double>(height) / 20});
            const auto decision = session.add(frame, std::move(detection));
            if (!decision.accepted) { failure = decision.message; break; }
        }
        std::size_t retained = 0;
        std::vector<std::weak_ptr<const Frame>> references;
        for (const auto& view : session.views()) {
            retained += view.frame->pixels->capacity();
            references.push_back(view.frame);
        }
        checkpoint("{\"stage\":\"filled\",\"cycle\":" + std::to_string(cycle)
            + ",\"accepted\":" + std::to_string(session.views().size())
            + ",\"stored_pixel_capacity\":" + std::to_string(retained)
            + ",\"budget\":" + std::to_string(config.maximumStoredBytes)
            + ",\"max_views\":" + std::to_string(config.maximumViews)
            + ",\"failure\":\"" + failure + "\"}");
        session.cancel();
        const bool released = std::all_of(references.begin(), references.end(), [](const auto& p) { return p.expired(); });
        checkpoint("{\"stage\":\"cancelled\",\"cycle\":" + std::to_string(cycle)
            + ",\"remaining_views\":" + std::to_string(session.views().size())
            + ",\"all_review_frames_released\":" + (released ? "true" : "false") + "}");
    }
}
'''

CASES = [(1920, 1080, 1, 0), (1920, 1080, 3, 0), (1920, 1080, 4, 0),
         (1920, 1080, 4, 4096), (2000, 2000, 4, 0)]


def resident_bytes(pid: int) -> int | None:
    try:
        result = subprocess.check_output(["ps", "-o", "rss=", "-p", str(pid)], text=True)
        return int(result.strip()) * 1024
    except (OSError, subprocess.SubprocessError, ValueError):
        return None


def run_case(binary: Path, case: tuple[int, ...], cycles: int) -> dict:
    width, height, channels, padding = case
    events = []
    with subprocess.Popen([str(binary), *map(str, case), str(cycles)], text=True,
                          stdin=subprocess.PIPE, stdout=subprocess.PIPE) as process:
        assert process.stdin and process.stdout
        for line in process.stdout:
            event = json.loads(line)
            event["rss_bytes"] = resident_bytes(process.pid)
            events.append(event)
            process.stdin.write("continue\n")
            process.stdin.flush()
        if process.wait() != 0:
            raise RuntimeError(f"Audit process failed for {case}")
    return {"width": width, "height": height,
            "pixel_format": {1: "Gray8", 3: "Rgb8", 4: "Rgba8"}[channels],
            "stride": width * channels + padding,
            "input_buffer_bytes": (width * channels + padding) * height,
            "events": events}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, help="Git revision containing the pre-fix capture implementation")
    parser.add_argument("--compiler", default="c++")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 1 <= args.cycles <= 20:
        parser.error("--cycles must be between 1 and 20")
    root = Path(__file__).resolve().parents[1]
    report = {"baseline": args.baseline, "cycles": args.cycles,
              "rss_units": "bytes (ps RSS in KiB × 1024); allocator caches may retain residency after release"}
    with tempfile.TemporaryDirectory(prefix="calibration-storage-audit-") as temporary:
        directory = Path(temporary)
        harness = directory / "audit.cpp"
        harness.write_text(HARNESS)
        for phase in ["before", "after"]:
            include = directory / phase
            source = include / "calibration"
            source.mkdir(parents=True)
            for name in ["CalibrationCapture.hpp", "CalibrationCapture.cpp"]:
                path = f"Mini_Robot/src/calibration/{name}"
                content = (subprocess.check_output(["git", "show", f"{args.baseline}:{path}"], cwd=root)
                           if phase == "before" else (root / path).read_bytes())
                (source / name).write_bytes(content)
            binary = directory / f"audit-{phase}"
            subprocess.run([args.compiler, "-std=c++20", "-O2", "-I", str(include), "-I",
                            str(root / "Mini_Robot/src"), str(harness), str(source / "CalibrationCapture.cpp"),
                            "-o", str(binary)], check=True, cwd=root)
            report[phase] = [run_case(binary, case, args.cycles) for case in CASES]
    encoded = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.write_text(encoded)
    print(encoded, end="")


if __name__ == "__main__":
    main()
