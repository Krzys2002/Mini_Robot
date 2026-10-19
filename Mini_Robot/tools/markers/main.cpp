#include "MarkerPrint.hpp"

#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string_view>

int main(int argc, char** argv)
{
    try {
        if (argc == 3 && std::string_view{argv[1]} == "--generate") {
            const std::filesystem::path directory{argv[2]};
            std::filesystem::create_directories(directory);
            for (const int size : {20, 40}) {
                const auto path = directory / ("tag36h11-" + std::to_string(size) + "mm-a4.svg");
                std::ofstream output;
                output.exceptions(std::ios::failbit | std::ios::badbit);
                output.open(path);
                output << micro_robot::marker_print::sheetSvg(size);
                output.close();
            }
            return 0;
        }
        if (argc == 3 && std::string_view{argv[1]} == "--verify-sheet") {
            const auto image = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
            if (image.empty()) {
                throw std::runtime_error("Cannot read rasterized sheet");
            }
            const auto ids = micro_robot::marker_print::detectIds(image);
            if (ids.size() != micro_robot::marker_print::robotCount) {
                throw std::runtime_error("Sheet must decode exactly 16 markers");
            }
            for (int id = 0; id < micro_robot::marker_print::robotCount; ++id) {
                if (ids[static_cast<std::size_t>(id)] != id) {
                    throw std::runtime_error("Expected each ID 0..15 exactly once");
                }
            }
            std::cout << "Verified all 16 tag36h11 IDs (0..15)\n";
            return 0;
        }
        std::cerr << "Usage: micro_robot_print_markers --generate DIRECTORY\n"
                     "       micro_robot_print_markers --verify-sheet RASTER.png\n";
        return 2;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
