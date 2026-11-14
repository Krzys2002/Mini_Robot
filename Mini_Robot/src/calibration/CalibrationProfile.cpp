#include "calibration/CalibrationProfile.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace micro_robot {
namespace {
// Decimal syntax only, independently of locale and stream-library extensions.
// Some libc++ versions have no floating-point from_chars. Classic-locale streams
// still give correctly rounded subnormals but may set failbit for ERANGE.
bool parseProfileDouble(std::string_view word, double& value)
{
    std::size_t pos = !word.empty() && word.front() == '-' ? 1 : 0;
    bool nonzero = false;
    const auto digits = [&] {
        const auto begin = pos;
        while (pos < word.size() && word[pos] >= '0' && word[pos] <= '9') {
            nonzero = nonzero || word[pos] != '0';
            ++pos;
        }
        return pos - begin;
    };
    auto significandDigits = digits();
    if (pos < word.size() && word[pos] == '.') { ++pos; significandDigits += digits(); }
    if (significandDigits == 0) return false;
    if (pos < word.size() && (word[pos] == 'e' || word[pos] == 'E')) {
        ++pos;
        if (pos < word.size() && (word[pos] == '+' || word[pos] == '-')) ++pos;
        const auto exponentBegin = pos;
        while (pos < word.size() && word[pos] >= '0' && word[pos] <= '9') ++pos;
        if (pos == exponentBegin) return false;
    }
    if (pos != word.size()) return false;
    std::istringstream number{std::string{word}};
    number.imbue(std::locale::classic());
    number >> value;
    if (number.bad() || number.rdbuf()->sgetc() != std::char_traits<char>::eof()
        || !std::isfinite(value) || (value == 0 && nonzero)) return false;
    return !number.fail() || std::fpclassify(value) == FP_SUBNORMAL;
}
std::string profileError(const IntrinsicCalibration& c)
{
    if (c.sourceId.size() > 4096 || std::any_of(c.sourceId.begin(), c.sourceId.end(),
        [](unsigned char v) { return v < 32 || v == 127; })) return "Invalid camera identifier.";
    if (!supportedCalibrationResolution(c.width, c.height))
        return "Profile resolution exceeds the supported 4 megapixel / 4096 per dimension bound.";
    if (c.perViewErrors.size() > CalibrationMaximumViews) return "Profile has too many views.";
    for (const auto& v : c.perViewErrors)
        if (v.pointCount > 1'000'000) return "Invalid profile point count.";
    return validateIntrinsicCalibration(c);
}
}
std::string serializeCalibrationProfile(const IntrinsicCalibration& c)
{
    if (auto error = profileError(c); !error.empty()) throw std::invalid_argument(error);
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<double>::max_digits10)
        << "MICRO_ROBOT_INTRINSICS 1\nmodel pinhole_brown_conrady_5\ncamera "
        << std::quoted(c.sourceId) << "\nresolution " << c.width << ' ' << c.height << "\nmatrix";
    for (double v : c.cameraMatrix) out << ' ' << v;
    out << "\ndistortion";
    for (double v : c.distortion) out << ' ' << v;
    out << "\nrms " << c.rmsPixels << "\nviews " << c.perViewErrors.size() << '\n';
    for (const auto& v : c.perViewErrors)
        out << v.frameSequence << ' ' << v.pointCount << ' ' << v.rmsPixels << '\n';
    out << "end\n";
    return out.str();
}
IntrinsicCalibrationOutcome parseCalibrationProfile(std::string_view text)
{
    const auto reject = [](const std::string& message) -> IntrinsicCalibrationOutcome {
        return {{}, "Calibration profile rejected: " + message};
    };
    if (text.size() > MaximumCalibrationProfileBytes) return reject("file too large.");
    std::istringstream in{std::string{text}};
    in.imbue(std::locale::classic());
    auto token = [&](std::string_view expected) { std::string word; return (in >> word) && word == expected; };
    const auto number = [&](auto& value) {
        std::string word;
        if (!(in >> word)) return false;
        if constexpr (std::is_integral_v<std::remove_reference_t<decltype(value)>>) {
            const auto parsed = std::from_chars(word.data(), word.data() + word.size(), value);
            return parsed.ec == std::errc{} && parsed.ptr == word.data() + word.size();
        } else {
            return parseProfileDouble(word, value);
        }
    };
    int version{};
    if (!token("MICRO_ROBOT_INTRINSICS") || !(in >> version) || version != 1)
        return reject("unknown format or version.");
    if (!token("model") || !token("pinhole_brown_conrady_5")) return reject("unsupported lens model.");
    IntrinsicCalibration c;
    // Require quotes; std::quoted by itself also accepts unquoted input.
    if (!token("camera") || !(in >> std::ws) || in.peek() != '"' || !(in >> std::quoted(c.sourceId))
        || !token("resolution") || !number(c.width) || !number(c.height) || !token("matrix"))
        return reject("invalid camera metadata.");
    for (double& v : c.cameraMatrix) if (!number(v)) return reject("invalid matrix.");
    if (!token("distortion")) return reject("missing distortion.");
    c.distortion.resize(5);
    for (double& v : c.distortion) if (!number(v)) return reject("invalid distortion.");
    std::size_t count{};
    if (!token("rms") || !number(c.rmsPixels) || !token("views") || !number(count) || count > CalibrationMaximumViews)
        return reject("invalid reprojection metadata.");
    for (std::size_t i = 0; i < count; ++i) {
        ReprojectionError view;
        if (!number(view.frameSequence) || !number(view.pointCount) || !number(view.rmsPixels))
            return reject("truncated view.");
        c.perViewErrors.push_back(view);
    }
    if (!token("end") || !(in >> std::ws).eof()) return reject("unexpected trailing data.");
    if (auto error = profileError(c); !error.empty()) return reject(error);
    return {std::move(c), {}};
}
std::string calibrationCompatibilityError(const IntrinsicCalibration& c,
    std::string_view sourceId, std::uint32_t width, std::uint32_t height)
{
    if (c.sourceId != sourceId) return "Calibration belongs to a different camera/device.";
    if (c.width != width || c.height != height) return "Calibration resolution does not match the delivered frame.";
    return {};
}
} // namespace micro_robot
