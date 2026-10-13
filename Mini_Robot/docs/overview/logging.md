# Application logging

Source: [LogRecord.hpp](../../src/infrastructure/logging/LogRecord.hpp),
[Logger.hpp](../../src/infrastructure/logging/Logger.hpp),
[Logger.cpp](../../src/infrastructure/logging/Logger.cpp),
[LogSinks.hpp](../../src/infrastructure/logging/LogSinks.hpp), and
[LogSinks.cpp](../../src/infrastructure/logging/LogSinks.cpp).

`micro_robot_logging` is a Qt-free cross-cutting M1 service that depends only on
common types. It builds and is tested with `MICRO_ROBOT_BUILD_DESKTOP=OFF`.

## Ownership and module access

There is no global logger. The composition root owns one `Logger` and injects
it, or per-module `LogChannel` handles derived from it, into modules that need
to log. A channel fixes the module name (currently `app` and `controller`;
later, for example, `camera` or `transport`). Channels are cheap, copyable, and
non-owning; the logger must outlive every channel.

## Records and format

Each `LogRecord` holds a wall-clock time, a `LogLevel` (Debug, Info, Warning,
Error), the module name, a message, and an optional `RobotId`. Records below the
logger's minimum level are dropped. The provided sinks write one line per record:

```text
2026-10-12T09:41:07.250Z [ERROR] [transport] robot=3 Link lost
```

Timestamps are UTC with millisecond precision, formatted with standard C++20
calendar types rather than platform-specific `gmtime` variants. Line breaks in
messages are escaped so each record stays on one line. Errors are identified by
the `[ERROR]` level tag; warnings use `[WARNING]`.

Log times come from `std::chrono::system_clock` so that operational logs can be
correlated with real time across runs. They are not suitable for latency or
freshness calculations, which keep using monotonic `Timestamp` values. The logger
accepts an injected clock function for deterministic tests.

## Threading and failure behavior

`Logger` is thread-safe, so future camera, vision, and transport workers may log
directly. Sinks run one at a time under the logger's lock and must not log
through the same logger. `Logger::log` is `noexcept`: exceptions from the clock,
formatting, or any sink are discarded, and one failing sink does not stop the
others. Logging must never crash the application.

## Sinks

- `StreamLogSink` writes to a caller-owned `std::ostream`.
- `FileLogSink` appends to a file, creating missing parent directories and
  flushing each record so that the log survives a crash. Opening never throws;
  `isOpen()` reports failure. There is no size-based rotation yet.

## Desktop application

[main.cpp](../../src/app/main.cpp) logs to stderr and appends to
`logs/micro_robot.log` under Qt's per-user `AppLocalDataLocation`. The path is
resolved with `QStandardPaths`, not hard-coded per platform, for example:

| Platform | Log file |
| --- | --- |
| macOS | `~/Library/Application Support/Micro Robot/logs/micro_robot.log` |
| Ubuntu | `~/.local/share/Micro Robot/logs/micro_robot.log` (honours `XDG_DATA_HOME`) |
| Windows | `%LOCALAPPDATA%/Micro Robot/logs/micro_robot.log` |

The resolved path is logged at startup. If the file cannot be opened, a warning
is logged to stderr and the application continues. The minimum level, whether a
log file is written, and its path come from the `[logging]` section of the
[configuration](configuration.md); the table above lists the default location.

The composition root logs `Application starting`, `Application started`,
`Application shutting down`, and `Application shutdown complete`.
[DesktopController](desktop-demo.md) logs initialization, shutdown requests,
camera and connection state changes (Fault states at Error level), and camera
and transport errors.

## Operational logs versus measurements

These logs are human-readable operational diagnostics. Thesis measurements,
such as latency, tracking error, or controller output time series, need
structured, machine-readable experiment records with monotonic timestamps. Those
belong to the later measurement issues and should not be parsed from these logs.

## Verification

[Logging tests](../../tests/logging/LoggerTest.cpp) cover formatting, level
filtering, channels from multiple modules, failing sinks and clocks, and file
append/failure behavior without Qt. [UI tests](../../tests/ui/MainWindowTest.cpp)
check controller log records with a capturing sink. See the
[testing guide](../development/testing.md).
