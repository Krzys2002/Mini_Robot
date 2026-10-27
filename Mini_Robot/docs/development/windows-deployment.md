# Windows deployment

Source: [Windows CI workflow](../../../.github/workflows/windows-build-test.yml),
[CMake presets](../../CMakePresets.json), and
[deployment script](../../tools/windows/Deploy-WindowsDesktop.ps1).

This guide covers packaging the desktop executable and its runtime dependencies
on Windows after a successful `windows-msvc` build.

## Prerequisites

- A build configured with the `windows-msvc` preset.
- Qt 6 (same toolchain/architecture as the build), with `windeployqt`.
- OpenCV package used at configure time.
- A Visual Studio developer shell matching the build toolchain.

## Build

From `Mini_Robot`, add the Qt and matching OpenCV runtime directories before
building (GoogleTest discovery can execute tests during the build):

```powershell
$env:PATH = "C:\Qt\6.8.3\msvc2022_64\bin;C:\tools\opencv\build\x64\vc16\bin;$env:PATH"
cmake --preset windows-msvc -DMICRO_ROBOT_BUILD_DESKTOP=ON -DMICRO_ROBOT_BUILD_REPLAY=ON `
  -DCMAKE_PREFIX_PATH="C:\Qt\6.8.3\msvc2022_64;C:\path\to\OpenCV\cmake"
cmake --build --preset windows-msvc --parallel 4
ctest --preset windows-msvc
```

## Package runtime files

Use the script to copy `micro_robot_desktop.exe`, invoke `windeployqt`, and add
OpenCV runtime DLLs into one deploy folder:

```powershell
.\tools\windows\Deploy-WindowsDesktop.ps1 `
  -QtPrefix C:\Qt\6.8.3\msvc2022_64 `
  -OpenCvBin C:\tools\opencv\build\x64\vc16\bin
```

By default this creates `build\windows-msvc\deploy\micro_robot_desktop\`.
Run the app from that folder:

```powershell
.\build\windows-msvc\deploy\micro_robot_desktop\micro_robot_desktop.exe
```

## Verification checklist

- Build and automated tests: workflow authored; successful Windows execution
  remains **pending**. See [Windows CI](testing.md#windows-continuous-integration).
- Camera discovery/preview on physical Windows hardware: **not run in this issue**
  (hardware access not available in CI).
- BLE path on physical Windows hardware: **not run in this issue** (hardware and
  BLE environment not available in CI).

Record those two hardware checks explicitly when executing local bench validation.
