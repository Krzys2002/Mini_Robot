#include "MarkerPrint.hpp"

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

#include <algorithm>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace micro_robot::marker_print {

cv::Mat markerCells(int tagId)
{
    if (tagId < 0 || tagId >= robotCount) {
        throw std::invalid_argument("Allocated robot tag IDs are 0..15");
    }
    const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::Mat cells;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    cv::aruco::generateImageMarker(dictionary, tagId, blackModules, cells, 1);
#else
    cv::aruco::drawMarker(dictionary, tagId, blackModules, cells, 1);
#endif
    return cells;
}

cv::Mat markerImage(int tagId, int pixelsPerCell)
{
    if (pixelsPerCell < 1 || pixelsPerCell > 100) {
        throw std::invalid_argument("Pixels per cell must be in 1..100");
    }
    cv::Mat bordered;
    cv::copyMakeBorder(markerCells(tagId), bordered, 1, 1, 1, 1,
                       cv::BORDER_CONSTANT, cv::Scalar{255});
    cv::Mat image;
    cv::resize(bordered, image, {}, pixelsPerCell, pixelsPerCell, cv::INTER_NEAREST);
    return image;
}

std::vector<int> detectIds(const cv::Mat& image)
{
    const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    cv::aruco::ArucoDetector detector{dictionary};
    detector.detectMarkers(image, corners, ids);
#else
    cv::aruco::detectMarkers(image, dictionary, corners, ids);
#endif
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::string sheetSvg(int footprintMm)
{
    if (footprintMm != 20 && footprintMm != 40) {
        throw std::invalid_argument("Print footprint must be 20 or 40 mm");
    }
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"210mm\" height=\"297mm\" viewBox=\"0 0 210 297\">\n"
        << "<rect width=\"210\" height=\"297\" fill=\"white\"/>\n"
        << "<g font-family=\"sans-serif\" fill=\"black\">\n"
        << "<text x=\"15\" y=\"12\" font-size=\"5\">Mini Robot - AprilTag 36h11</text>\n"
        << "<text x=\"15\" y=\"19\" font-size=\"3.2\">Footprint " << footprintMm
        << " mm; black edge " << footprintMm * 8 / 10 << " mm; IDs 0-15</text>\n"
        << "<text x=\"15\" y=\"25\" font-size=\"3.2\">Print at 100%; no fit-to-page. Keep white square. UP = front.</text>\n";
    const double cell = static_cast<double>(footprintMm) / footprintModules;
    for (int id = 0; id < robotCount; ++id) {
        const int x = 15 + (id % 4) * 45;
        const int y = 35 + (id / 4) * 55;
        out << "<g transform=\"translate(" << x << ' ' << y << ")\">\n"
            << "<rect x=\"0\" y=\"0\" width=\"" << footprintMm << "\" height=\""
            << footprintMm << "\" fill=\"white\"/>\n"
            << "<g transform=\"scale(" << cell << ")\" shape-rendering=\"crispEdges\">\n";
        const auto cells = markerCells(id);
        for (int row = 0; row < blackModules; ++row) {
            for (int col = 0; col < blackModules; ++col) {
                if (cells.at<unsigned char>(row, col) == 0) {
                    out << "<rect x=\"" << col + 1 << "\" y=\"" << row + 1
                        << "\" width=\"1\" height=\"1\"/>\n";
                }
            }
        }
        out << "</g>\n<text x=\"0\" y=\"" << footprintMm + 5
            << "\" font-size=\"3\">UP ^  tag " << id << " = RobotId " << id + 1
            << "</text>\n</g>\n";
    }
    out << "<path d=\"M15 268h50 M15 265v6 M65 265v6\" fill=\"none\" stroke=\"black\" stroke-width=\"0.3\"/>\n"
        << "<text x=\"15\" y=\"278\" font-size=\"3.5\">Scale check: 50 mm between end ticks</text>\n"
        << "<text x=\"15\" y=\"285\" font-size=\"3\">Cut outside white footprint; labels stay off the robot.</text>\n"
        << "</g>\n</svg>\n";
    return out.str();
}

} // namespace micro_robot::marker_print
