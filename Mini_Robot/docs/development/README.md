# Development guides

- [Optional Graphify navigation](graphify.md): locked local tools, worktree freshness, pilot results, limitations, and recovery.

- [Build setup](build.md): dependencies, platform/compiler configuration, CMake presets, platform recipes, and troubleshooting.
- [Ubuntu setup](linux-setup.md): clean Ubuntu 24.04 LTS quick start, compilers, Qt 6, OpenCV, GoogleTest, camera access, BlueZ, build/test commands, troubleshooting, and the Linux builds executed so far.
- [Windows deployment](windows-deployment.md): Qt/OpenCV runtime packaging with `windeployqt`, deploy script usage, and hardware-check reporting limits.
- [Testing](testing.md): GoogleTest/CTest setup, the hardware-free regression suite run by CI, desktop and Qt-free builds.
- [Hardware tests](hardware-tests.md): tests and manual checks that need cameras, robots, or a real display, kept out of the regression suite.
- [GUI compatibility](gui-compatibility.md): Qt GUI checks on Wayland, X11, and macOS, executed Linux results, limitations, and the manual checklist.
- [GitHub blocker automation](GITHUB_BLOCKER_AUTOMATION.md): Project status
  synchronization, authentication, local tests, and troubleshooting.
