# Application configuration

Source: [ApplicationConfig.hpp](../../src/infrastructure/configuration/ApplicationConfig.hpp),
[ConfigLoader.hpp](../../src/infrastructure/configuration/ConfigLoader.hpp),
[ConfigLoader.cpp](../../src/infrastructure/configuration/ConfigLoader.cpp), and the
[example file](../../config/micro_robot.example.ini).

`micro_robot_configuration` is a Qt-free cross-cutting M1 service that depends on
common types and logging. It builds and is tested with
`MICRO_ROBOT_BUILD_DESKTOP=OFF`.

## Ownership and module access

`ApplicationConfig` is a plain struct of typed sections (`logging`, `robot`,
`simulation`). Member initializers are the built-in defaults. The composition
root ([main.cpp](../../src/app/main.cpp)) loads one instance at startup and
injects the individual values that each module needs, for example the robot ID
into `DesktopController` and the frame interval into `FakeCameraConfig`. There
is no global settings object; modules do not read files or know key names, and
no GUI widget stores settings. All file keys are mapped to typed fields in one
table in `ConfigLoader.cpp`.

Durations are `std::chrono::milliseconds` in code; file keys carry an `_ms`
suffix. New settings (camera selection, transport parameters, controller gains,
and so on) are added by extending a section struct, the key table, the example
file, and the tests.

## File format

The initial format is INI-style text, chosen because it is human-editable, needs
no additional dependency, and keeps the parser Qt-free and unit-testable:

```ini
# full-line comment ('#' or ';')
[robot]
id = 1
```

Section and key names are case-sensitive; enumeration and boolean values are not.
Values are taken literally after trimming surrounding whitespace, so there are no
inline comments or quoting. A UTF-8 byte order mark and CRLF line endings are
accepted. Every setting is optional.

| Key | Accepted values | Default |
| --- | --- | --- |
| `logging.level` | `debug`, `info`, `warning`, `error` | `info` |
| `logging.file_enabled` | `true`/`false` (also `yes`/`no`, `on`/`off`, `1`/`0`) | `true` |
| `logging.file_path` | UTF-8 path; empty selects the [default log location](logging.md#desktop-application) | empty |
| `robot.id` | integer 0 to 4294967295 | `1` |
| `simulation.update_interval_ms` | integer 1 to 10000 | `100` |
| `simulation.camera_frame_interval_ms` | integer 1 to 10000 | `33` |

Calibration and workspace profiles are not part of this file; they will use
their own per-camera profile format when the calibration issues are implemented.

## File location

Without arguments the application reads `micro_robot.ini` from Qt's per-user
`AppConfigLocation`:

| Platform | Configuration file |
| --- | --- |
| macOS | `~/Library/Preferences/Micro Robot/micro_robot.ini` |
| Ubuntu | `~/.config/Micro Robot/micro_robot.ini` (honours `XDG_CONFIG_HOME`) |
| Windows | `%LOCALAPPDATA%/Micro Robot/micro_robot.ini` |

`--config <file>` selects another file, for example an experiment-specific one:

```sh
./build/src/micro_robot_desktop --config config/micro_robot.example.ini
```

The application never creates or rewrites the configuration file. Copy the
[example file](../../config/micro_robot.example.ini) and edit it.

## Failure behavior

Loading never prevents startup; every problem falls back to defaults:

| Situation | Result | Log |
| --- | --- | --- |
| Valid file | Values applied | Info: file loaded |
| Default file absent | Defaults | Info: no file, using defaults |
| `--config` file absent | Defaults | Error naming the file |
| Path is not a regular file or cannot be opened | Defaults | Error naming the file |
| Invalid value, unknown key/section, malformed line | Setting keeps its default; other valid entries still apply | Warning summary plus one warning per problem |
| Repeated key | Last valid value wins | Warning naming both lines |

Diagnostics name the file, line, key, rejected value, accepted range, and the
value that remains in effect, for example:

```text
[WARNING] [config] /home/me/.config/Micro Robot/micro_robot.ini:2: invalid value 'seven' for 'robot.id' (expected an integer from 0 to 4294967295); keeping 1
```

Configuration is loaded before the logger exists, because the logger's level
and file come from it. The loader therefore returns diagnostics instead of
logging; `logConfigLoadResult` writes them through the `config` log channel
once all log sinks are attached, so they reach both stderr and the log file.

## Verification

[Configuration tests](../../tests/configuration/ConfigLoaderTest.cpp) cover
defaults, every key, whitespace/comments/BOM/CRLF, boundary and invalid values,
unknown and malformed entries, duplicates, missing and unreadable files, log
output, and that the shipped example file is valid and matches the defaults.
See the [testing guide](../development/testing.md).
