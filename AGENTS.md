# Repository agent instructions

## Scope and sources of truth

The main application is `Mini_Robot/`. Read `Mini_Robot/AGENTS.md` before editing
that subtree; its architecture, safety, C++ and issue-workflow rules also apply
when changing the root desktop build integration. Preserve unrelated user edits.
For a direct maintenance request without an assigned issue, implement that request
without inventing issue dependencies or publishing a PR unless requested.

Use the root `README.md` for repository layout and root commands, and
`Mini_Robot/docs/README.md` for module guides. Current implementation status is in
`Mini_Robot/docs/overview/capabilities.md`; planning catalogs, archived reports,
and `core arhiteture.md` do not establish implemented functionality.

## Build ownership

- Root `CMakeLists.txt` enables CTest and adds `Mini_Robot/`. Keep it thin.
- `Mini_Robot/CMakeLists.txt` owns desktop options, dependencies, and targets.
  Preserve standalone configuration from that directory and existing CI commands.
- Root `CMakePresets.json` includes `Mini_Robot/CMakePresets.json`; edit shared
  presets there rather than duplicating configurations.
- Keep project-relative paths valid for both entry points. Within Mini_Robot,
  use `PROJECT_SOURCE_DIR` or `CMAKE_CURRENT_SOURCE_DIR` as appropriate rather
  than assuming `CMAKE_SOURCE_DIR` is the application directory.
- Root builds place application artifacts under `build/<preset>/Mini_Robot/`;
  standalone builds place them under `Mini_Robot/build/<preset>/`.
- `T1-ESP-C6/`, `T1-ESP-S3/`, and `Mini_Robot/firmware/ble-lifecycle/` are separate
  PlatformIO projects. Do not link embedded sources into desktop production
  targets. Existing host-side firmware safety tests deliberately compile the
  legacy controller against fakes.
- Do not commit build trees, local CMake presets, secrets, or IDE state.
  Do not modify tracked IDE files just to make the project load on one machine.

## Validation

Run from the repository root for root integration changes:

```sh
cmake --preset core
cmake --build --preset core --parallel
ctest --preset core
```

For desktop changes use the host preset (`macos`, `linux-gcc`, `linux-clang`,
`windows-msvc`) and build/test it. Root integration changes must also configure
and test the standalone Mini_Robot entry point, since CI uses it. Use separate
build directories. Follow `Mini_Robot/docs/development/testing.md` for additional
affected suites, including browser-control tests when touching legacy firmware.

The core preset avoids Qt/OpenCV and physical hardware, but tests require an
installed GTest package or the pinned download. Supply an existing GoogleTest
checkout with `FETCHCONTENT_SOURCE_DIR_GOOGLETEST` for offline builds.

Report commands actually run and distinguish build/test results from GUI or
hardware validation. Do not claim a completed perception-to-motion loop or
production motor safety based on component tests or the BLE connection fixture.
