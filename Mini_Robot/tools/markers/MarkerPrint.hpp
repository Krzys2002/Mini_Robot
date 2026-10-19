#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace micro_robot::marker_print {

inline constexpr int robotCount = 16;
inline constexpr int blackModules = 8; // 6 data cells plus one black border per side
inline constexpr int footprintModules = 10; // one white quiet cell per side

// Offline tools, called synchronously by the CLI/tests, never from the GUI.
// Throws std::invalid_argument for IDs outside the allocated robot range.
[[nodiscard]] cv::Mat markerCells(int tagId);
[[nodiscard]] cv::Mat markerImage(int tagId, int pixelsPerCell);
// Sorted decoded dictionary IDs; this tool does not register robots or update poses.
[[nodiscard]] std::vector<int> detectIds(const cv::Mat& image);
// A4 portrait; footprint includes the quiet zone. Only the two specified sizes
// are accepted. Throws std::invalid_argument for other sizes.
[[nodiscard]] std::string sheetSvg(int footprintMm);

} // namespace micro_robot::marker_print
