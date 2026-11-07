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
- The shared `desktop` preset builds the GUI without tests. Use host/core presets
  for validation. Optional `firmware_*` targets delegate build-only work to nested
  PlatformIO manifests; never make them desktop/default-build dependencies.
- Keep project-relative paths valid for both entry points. Within Mini_Robot,
  use `PROJECT_SOURCE_DIR` or `CMAKE_CURRENT_SOURCE_DIR` as appropriate rather
  than assuming `CMAKE_SOURCE_DIR` is the application directory.
- Root builds place application artifacts under `build/<preset>/Mini_Robot/`;
  standalone builds place them under `Mini_Robot/build/<preset>/`.
- Firmware lives under `Mini_Robot/firmware/`. `codecell-c6/` is the new C6
  production build skeleton; `ble-lifecycle/` is a separate connection fixture.
  The outdated T1 projects and HTTP/browser controller have been retired.
  Never link embedded production sources into desktop targets. Firmware host
  tests compile portable headers/logic through an independent CMake entry point.
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
affected suites, including firmware host tests when touching firmware contracts.

The core preset avoids Qt/OpenCV and physical hardware, but tests require an
installed GTest package or the pinned download. Supply an existing GoogleTest
checkout with `FETCHCONTENT_SOURCE_DIR_GOOGLETEST` for offline builds.

Report commands actually run and distinguish build/test results from GUI or
hardware validation. Do not claim a completed perception-to-motion loop or
production motor safety based on component tests or the BLE connection fixture.

## Optional code graph

Graphify is a derived navigation aid. Applicable instructions and assigned issues
define scope; source and tests establish implementation; live GitHub data
establishes project state; recorded execution establishes validation. Use
`Mini_Robot/docs/overview/capabilities.md` for current integration limits, not
planning catalogs, archives, or the historical `core arhiteture.md` sketch.

- Read applicable instructions and the assigned issue/dependencies first.
- For structural exploration, check `python3 tools/graphify/project_graph.py status`
  from the Git worktree root. A missing/stale graph must not block work: use `rg`
  and source inspection, or explicitly build/update the graph when worthwhile.
- Use scoped query/path/explain/affected commands to locate relevant code/tests.
  Exact strings and errors can use `rg` directly. Verify source before editing;
  an absent edge does not prove independence. Pass this guidance to exploring
  subagents; they share this worktree's graph and must not refresh concurrently.
- After coherent edits, refresh with `update`; use `build` for first use,
  recovery, or changed extraction policy. Every worktree owns its ignored
  `graphify-out/`. Never reuse another checkout's output or commit generated graphs.
- Keep backend `none` and code-only extraction. Semantic indexing, hosted upload,
  saved-answer ingestion, hooks, and global agent changes are outside this setup.
- Do not infer issue completion, calibrated closed-loop operation, physical
  stopping, or firmware safety from graph results. Read live GitHub blockers;
  graph impact suggestions do not replace required build/test suites.

Commands, coverage limits, and recovery: [Graphify guide](Mini_Robot/docs/development/graphify.md).
For the reusable structural-exploration procedure, use the repository skill
[mini-robot-graph](.agents/skills/mini-robot-graph/SKILL.md).
