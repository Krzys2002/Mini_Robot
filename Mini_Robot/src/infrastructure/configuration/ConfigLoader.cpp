#include "infrastructure/configuration/ConfigLoader.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cstdint>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <system_error>
#include <utility>

namespace micro_robot {
namespace {

using std::chrono::milliseconds;

std::string_view trim(std::string_view text)
{
    constexpr std::string_view whitespace = " \t\r\f\v";
    const auto first = text.find_first_not_of(whitespace);
    if (first == std::string_view::npos) return {};
    const auto last = text.find_last_not_of(whitespace);
    return text.substr(first, last - first + 1);
}

std::string toLower(std::string_view text)
{
    std::string lower{text};
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return lower;
}

template <typename Integer>
std::optional<Integer> parseInteger(std::string_view text)
{
    Integer value{};
    const auto* end = text.data() + text.size();
    const auto [parsedTo, error] = std::from_chars(text.data(), end, value);
    if (text.empty() || error != std::errc{} || parsedTo != end) return std::nullopt;
    return value;
}

std::optional<milliseconds> parseMilliseconds(std::string_view text, milliseconds min, milliseconds max)
{
    const auto value = parseInteger<std::int64_t>(text);
    if (!value || *value < min.count() || *value > max.count()) return std::nullopt;
    return milliseconds{*value};
}

std::optional<bool> parseBool(std::string_view text)
{
    const auto lower = toLower(text);
    if (lower == "true" || lower == "yes" || lower == "on" || lower == "1") return true;
    if (lower == "false" || lower == "no" || lower == "off" || lower == "0") return false;
    return std::nullopt;
}

constexpr std::array logLevels{LogLevel::Debug, LogLevel::Info, LogLevel::Warning, LogLevel::Error};

std::optional<LogLevel> parseLogLevel(std::string_view text)
{
    const auto lower = toLower(text);
    for (const auto level : logLevels) {
        if (lower == toLower(toString(level))) return level;
    }
    return std::nullopt;
}

void logAtLevel(const LogChannel& log, LogLevel level, std::string_view message)
{
    switch (level) {
    case LogLevel::Debug: log.debug(message); return;
    case LogLevel::Info: log.info(message); return;
    case LogLevel::Warning: log.warning(message); return;
    case LogLevel::Error: log.error(message); return;
    }
}

std::string pathToUtf8(const std::filesystem::path& path)
{
    const auto utf8 = path.u8string();
    return {utf8.begin(), utf8.end()};
}

// Bounds keep the demo timer responsive without allowing a busy loop.
constexpr milliseconds minInterval{1};
constexpr milliseconds maxInterval{10'000};

// The single table of recognised settings: file keys are mapped to typed fields
// here and nowhere else.
struct Setting {
    std::string_view key; // "section.name"
    std::string_view expected;
    bool (*apply)(std::string_view value, ApplicationConfig& config);
    std::string (*current)(const ApplicationConfig& config);
};

const std::array settings{
    Setting{"logging.level", "one of debug, info, warning, error",
        [](std::string_view value, ApplicationConfig& config) {
            const auto level = parseLogLevel(value);
            if (level) config.logging.minimumLevel = *level;
            return level.has_value();
        },
        [](const ApplicationConfig& config) { return toLower(toString(config.logging.minimumLevel)); }},
    Setting{"logging.file_enabled", "true or false",
        [](std::string_view value, ApplicationConfig& config) {
            const auto enabled = parseBool(value);
            if (enabled) config.logging.fileEnabled = *enabled;
            return enabled.has_value();
        },
        [](const ApplicationConfig& config) {
            return std::string{config.logging.fileEnabled ? "true" : "false"};
        }},
    Setting{"logging.file_path", "a file path, or empty for the default location",
        [](std::string_view value, ApplicationConfig& config) {
            config.logging.filePath = std::string{value};
            return true;
        },
        [](const ApplicationConfig& config) { return "'" + config.logging.filePath + "'"; }},
    Setting{"robot.id", "an integer from 0 to 4294967295",
        [](std::string_view value, ApplicationConfig& config) {
            const auto id = parseInteger<std::uint32_t>(value);
            if (id) config.robot.id = RobotId{*id};
            return id.has_value();
        },
        [](const ApplicationConfig& config) { return std::to_string(config.robot.id.value()); }},
    Setting{"simulation.update_interval_ms", "an integer from 1 to 10000",
        [](std::string_view value, ApplicationConfig& config) {
            const auto interval = parseMilliseconds(value, minInterval, maxInterval);
            if (interval) config.simulation.updateInterval = *interval;
            return interval.has_value();
        },
        [](const ApplicationConfig& config) {
            return std::to_string(config.simulation.updateInterval.count());
        }},
    Setting{"simulation.camera_frame_interval_ms", "an integer from 1 to 10000",
        [](std::string_view value, ApplicationConfig& config) {
            const auto interval = parseMilliseconds(value, minInterval, maxInterval);
            if (interval) config.simulation.cameraFrameInterval = *interval;
            return interval.has_value();
        },
        [](const ApplicationConfig& config) {
            return std::to_string(config.simulation.cameraFrameInterval.count());
        }},
};

const Setting* findSetting(std::string_view key)
{
    const auto found = std::find_if(settings.begin(), settings.end(),
                                    [key](const Setting& setting) { return setting.key == key; });
    return found == settings.end() ? nullptr : &*found;
}

bool isKnownSection(std::string_view section)
{
    return std::any_of(settings.begin(), settings.end(), [section](const Setting& setting) {
        return setting.key.substr(0, setting.key.find('.')) == section;
    });
}

class Parser {
public:
    explicit Parser(std::string_view sourceName) : sourceName_(sourceName) {}

    ConfigParseResult parse(std::string_view text)
    {
        constexpr std::string_view utf8Bom = "\xEF\xBB\xBF";
        if (text.substr(0, utf8Bom.size()) == utf8Bom) text.remove_prefix(utf8Bom.size());

        while (!text.empty()) {
            ++lineNumber_;
            const auto end = text.find('\n');
            parseLine(trim(text.substr(0, end)));
            text.remove_prefix(end == std::string_view::npos ? text.size() : end + 1);
        }
        return std::move(result_);
    }

private:
    void parseLine(std::string_view line)
    {
        if (line.empty() || line.front() == '#' || line.front() == ';') return;

        if (line.front() == '[') {
            const auto name = line.back() == ']' ? trim(line.substr(1, line.size() - 2))
                                                 : std::string_view{};
            if (name.empty()) {
                warn("expected '[section]', got '" + std::string{line} + "'; line ignored");
                section_.reset();
                return;
            }
            section_ = std::string{name};
            sectionKnown_ = isKnownSection(name);
            if (!sectionKnown_) {
                warn("unknown section [" + *section_ + "]; its entries are ignored");
            }
            return;
        }

        const auto separator = line.find('=');
        if (separator == std::string_view::npos) {
            warn("expected 'key = value', got '" + std::string{line} + "'; line ignored");
            return;
        }
        const auto name = trim(line.substr(0, separator));
        const auto value = trim(line.substr(separator + 1));
        if (!section_) {
            warn("setting '" + std::string{name} + "' is not inside a [section]; entry ignored");
            return;
        }
        if (!sectionKnown_) return; // Already reported once for the whole section.

        const auto key = *section_ + "." + std::string{name};
        const auto* setting = findSetting(key);
        if (!setting) {
            warn("unknown setting '" + key + "'; entry ignored");
            return;
        }
        if (const auto [previous, inserted] = firstLines_.try_emplace(key, lineNumber_); !inserted) {
            warn("duplicate setting '" + key + "' (first set on line "
                 + std::to_string(previous->second) + "); the later value is used if valid");
        }
        if (!setting->apply(value, result_.config)) {
            warn("invalid value '" + std::string{value} + "' for '" + key + "' (expected "
                 + std::string{setting->expected} + "); keeping " + setting->current(result_.config));
        }
    }

    void warn(const std::string& message)
    {
        result_.diagnostics.push_back({LogLevel::Warning, std::string{sourceName_} + ":"
            + std::to_string(lineNumber_) + ": " + message});
    }

    std::string_view sourceName_;
    ConfigParseResult result_;
    std::map<std::string, int> firstLines_;
    std::optional<std::string> section_;
    bool sectionKnown_{};
    int lineNumber_{};
};

} // namespace

ConfigParseResult parseConfig(std::string_view text, std::string_view sourceName)
{
    return Parser{sourceName}.parse(text);
}

ConfigLoadResult loadConfigFile(const std::filesystem::path& path, ConfigFilePolicy policy)
{
    ConfigLoadResult result;
    result.path = path;
    const auto displayPath = pathToUtf8(path);
    const auto fail = [&](ConfigFileStatus status, LogLevel level, const std::string& reason) {
        result.status = status;
        result.diagnostics.push_back({level, reason + "; using built-in defaults"});
        return std::move(result);
    };

    std::error_code error;
    const auto fileStatus = std::filesystem::status(path, error);
    if (fileStatus.type() == std::filesystem::file_type::not_found) {
        if (policy == ConfigFilePolicy::Optional) {
            result.status = ConfigFileStatus::NotFound;
            return result;
        }
        return fail(ConfigFileStatus::NotFound, LogLevel::Error,
                    "Configuration file " + displayPath + " does not exist");
    }
    if (error) {
        return fail(ConfigFileStatus::Unreadable, LogLevel::Error,
                    "Cannot access configuration file " + displayPath + ": " + error.message());
    }
    if (fileStatus.type() != std::filesystem::file_type::regular) {
        return fail(ConfigFileStatus::Unreadable, LogLevel::Error,
                    "Configuration path " + displayPath + " is not a regular file");
    }

    std::ifstream file{path, std::ios::binary};
    if (!file) {
        return fail(ConfigFileStatus::Unreadable, LogLevel::Error,
                    "Cannot open configuration file " + displayPath);
    }
    std::ostringstream contents;
    contents << file.rdbuf();

    auto parsed = parseConfig(contents.str(), displayPath);
    result.config = parsed.config;
    result.diagnostics = std::move(parsed.diagnostics);
    result.status = ConfigFileStatus::Loaded;
    return result;
}

void logConfigLoadResult(const ConfigLoadResult& result, const LogChannel& log)
{
    const auto displayPath = pathToUtf8(result.path);
    if (result.status == ConfigFileStatus::Loaded) {
        if (result.diagnostics.empty()) {
            log.info("Configuration loaded from " + displayPath);
        } else {
            log.warning("Configuration loaded from " + displayPath + " with "
                        + std::to_string(result.diagnostics.size())
                        + " problem(s); affected settings keep their defaults");
        }
    } else if (result.status == ConfigFileStatus::NotFound && result.diagnostics.empty()) {
        log.info("No configuration file at " + displayPath + "; using built-in defaults");
    }
    for (const auto& diagnostic : result.diagnostics) {
        logAtLevel(log, diagnostic.level, diagnostic.message);
    }
}

} // namespace micro_robot
