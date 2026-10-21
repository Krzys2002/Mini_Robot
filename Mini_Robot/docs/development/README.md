# Development guides

- [Build setup](build.md): dependencies, platform/compiler configuration, CMake presets, platform recipes, and troubleshooting.
- [Ubuntu setup](linux-setup.md): Linux build requirements and GCC/Clang/headless recipes, with the Linux builds executed so far.
- [Testing](testing.md): GoogleTest/CTest setup, the hardware-free regression suite run by CI, desktop and Qt-free builds.
- [Hardware tests](hardware-tests.md): tests and manual checks that need cameras, robots, or a real display, kept out of the regression suite.
- [GUI compatibility](gui-compatibility.md): Qt GUI checks on Wayland, X11, and macOS, executed Linux results, limitations, and the manual checklist.
- [GitHub blocker automation](GITHUB_BLOCKER_AUTOMATION.md): Project status
  synchronization, authentication, local tests, and troubleshooting.

The Ubuntu recipe does not close [LNX-07 / #131](https://github.com/Krzys2002/Mini_Robot/issues/131): clean-Ubuntu build validation and future hardware-package details remain outstanding.
