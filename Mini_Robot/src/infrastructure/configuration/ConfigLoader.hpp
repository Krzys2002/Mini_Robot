#pragma once

#include "infrastructure/configuration/ApplicationConfig.hpp"
#include "infrastructure/logging/Logger.hpp"

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace micro_robot {

// One problem found while loading configuration. Messages are complete and
// actionable, e.g. "micro_robot.ini:7: invalid value 'abc' for robot.id ...".
struct ConfigDiagnostic {
    LogLevel level{LogLevel::Warning};
    std::string message;
};

struct ConfigParseResult {
    ApplicationConfig config;
    std::vector<ConfigDiagnostic> diagnostics;
};

// Parses INI-style text:
//
//   # full-line comment (also ';')
//   [section]
//   key = value
//
// Section and key names are case-sensitive; enumeration and boolean values are
// not. Values are taken literally after trimming surrounding whitespace (no inline
// comments or quoting). Malformed lines, unknown keys, and invalid values produce
// Warning diagnostics and leave the affected setting at its default; valid entries
// are still applied. A repeated key is reported and its last valid value wins.
// `sourceName` prefixes each diagnostic. Throws only std::bad_alloc.
[[nodiscard]] ConfigParseResult parseConfig(std::string_view text, std::string_view sourceName);

enum class ConfigFileStatus { Loaded, NotFound, Unreadable };

// Optional: a missing file silently selects defaults (first run).
// Required: a missing file is reported as an Error (e.g. a path given explicitly).
enum class ConfigFilePolicy { Optional, Required };

struct ConfigLoadResult {
    ApplicationConfig config;
    std::filesystem::path path;
    ConfigFileStatus status{ConfigFileStatus::NotFound};
    std::vector<ConfigDiagnostic> diagnostics;
};

// Reads and parses a configuration file. Never fails: if the file is missing or
// unreadable, defaults are returned and the problem is described in diagnostics.
// Throws only std::bad_alloc.
[[nodiscard]] ConfigLoadResult loadConfigFile(const std::filesystem::path& path,
                                              ConfigFilePolicy policy = ConfigFilePolicy::Optional);

// Logs a summary of the load (source file or defaults) and every diagnostic at
// its own level. Kept separate from loading so that the logger itself can be
// configured from the result before anything is logged.
void logConfigLoadResult(const ConfigLoadResult& result, const LogChannel& log);

} // namespace micro_robot
