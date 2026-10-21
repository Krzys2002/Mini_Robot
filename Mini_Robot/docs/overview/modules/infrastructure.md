# Configuration, logging and portability

Milestone scope: M1 / M8. [Module index](README.md).

## Capability and boundaries

Configuration loads typed INI settings at startup; composition injects them.
The Qt-free logger supplies channels, timestamps and stream/file sinks, without a
hidden global logger. CMake targets/boundary checks isolate OS/device dependencies.
Human-readable logging is not a completed measurement recorder. Calibration and
workspace profile persistence remain separate.

## Behavior guides

- [Configuration](../configuration.md), [logging](../logging.md)
- [Platform boundaries](../platform-boundaries.md)
- [Build setup](../../development/build.md), [Ubuntu](../../development/linux-setup.md)

## Verification and remaining acceptance

Configuration/filesystem tests cover invalid input, paths, ownership and failures.
Boundary tests reject forbidden dependencies. CI owns build evidence; setup guides
own recipes. Windows, real devices and physical compositor input still need acceptance. The
[GUI guide](../../development/gui-compatibility.md) owns recorded automated Linux
results and their limits.
