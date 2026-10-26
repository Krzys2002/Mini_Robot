# Documentation ownership and merge workflow

Feature documentation should change in the same PR as its behavior, without
requiring every PR to rewrite shared status tables.

## Which page to update

| Change | Owning location |
| --- | --- |
| Public behavior/API, examples, limits, manual procedure | Feature behavior guide linked from its module |
| Capability or remaining acceptance | Relevant page under [overview/modules](../overview/modules/README.md) |
| Shared dependency direction, units or threading rules | [Architecture](../overview/architecture.md) |
| Sources, targets, direct dependencies | CMake declarations; regenerate the [inventory](../overview/source-map.md) |
| Reusable build/test command or CI mechanics | Development build/testing guide |
| Test coverage specific to a feature | Module/behavior guide, alongside the feature |
| Run-specific results/counts | PR validation or a dated record in the owning guide |
| Live issue status, blockers, priorities and dates | GitHub issues/project |

Indexes are navigation, not duplicate capability reports. Add a module link when
introducing a new module; an ordinary change within that module should not touch
every index. Keep entries/sections separate instead of rewriting one long shared
paragraph or milestone-table row. The roadmap owns future milestone scope.

Do not replace a feature's detailed guide with a short module summary. Module
pages own the current capability summary; guides own the detailed API/procedure.
Preserve dated evidence and distinguish automated, physical and unrun checks.

## Generated inventory

Enable `MICRO_ROBOT_BUILD_DOCUMENTATION=ON` in the desired CMake configuration.
The report is written to `<build>/docs/source-map.md` from actual CMake target
properties. It includes first-party libraries/executables declared under `src`,
`tests` and `tools`, including interface sources and header file sets. Fetched
dependencies and utility/autogen targets are excluded.

The report lists direct link and interface declarations, not a reconstructed
transitive graph. It preserves generator expressions as declarations and labels
generated/external entries. Checkout/build path prefixes are replaced with
`@source`/`@build`, and absolute external paths use `@external/<basename>` so
reports remain relocatable. A disabled target is absent; inspect desktop, core
or optional-feature configurations as appropriate. Do not commit generated
snapshots or add manual per-source/per-target tables elsewhere.

## Local checks

The link checker needs only Python 3.9+ and its standard library. Tooling tests
also use CMake and the project's C++ compiler to configure a small fixture. No
Python package install or network access is needed. Ordinary builds keep the
documentation option OFF and gain no Python requirement.

```sh
python3 tools/docs/check_links.py --root .
python3 -m unittest discover -s tools/docs/tests -v
cmake --preset core -DMICRO_ROBOT_BUILD_DOCUMENTATION=ON
cmake --build --preset core --target documentation
ctest --test-dir build/core -L documentation --output-on-failure --no-tests=error
```

The checker validates local file links, Markdown heading/explicit anchors and
conflict markers in maintained guides: root README/AGENTS, the documentation index,
overview, development, vision, protocol and documentation-tool guides. It skips
code examples and external URLs; it does not claim external-link availability or
full Markdown syntax validation. Historical planning/archive reports are not
scanned as current guidance, but links to them still need valid destinations.
The build-local report is also checked when supplied with `--inventory`.

The scheduled/master CI suite enables the option for macOS, Linux GCC/Clang and
the core job. These full build workflows do not run on pull requests. The cheap
`Documentation links` workflow checks maintained local links/anchors on PRs
without configuring CMake or installing dependencies. CTest in full builds runs
`Documentation.Links` and `Documentation.Tooling` under the `documentation` label.
Core documentation checks require neither Qt nor OpenCV and do not build hardware
adapters. Read each CI result before claiming platform success.

## Merging overlapping work

1. State real PR dependencies and stack only work that needs an unmerged
   prerequisite. Do not combine unrelated features just to avoid doc conflicts.
2. Review functionality/interface changes first. Follow with dependent regression
   changes; consolidate broad setup/documentation rewrites after those inputs settle.
3. After each human merge, refresh remaining branches from the new base, preserve
   both sides' valid content and recheck their build/docs. A clean PR against today's
   base does not guarantee it will remain clean after another feature merges.
4. Resolve conflicts in the owning module/guide. Avoid blanket “ours/theirs” choices,
   which can silently remove another feature or restore obsolete capability claims.
5. Keep normal pushes and human review. This workflow does not authorize automatic
   PR merges, force pushes or bypassing acceptance criteria.

This reduces shared edits; it cannot remove conflicts between changes to the same
behavior or the same build/composition code.

## Adopting this layout in existing PRs

After this change merges, refresh older feature branches once. Transfer their
capability and coverage updates from the old shared tables into the owning module
page, preserve their detailed behavior guides, and keep their CMake declarations.
For example, BLE work updates the communication page, while detector work updates
vision; composition changes may also update desktop. Regenerate inventories instead
of restoring the deleted manual source/target lists. This migration requires human
content review; it is not an automatic merge strategy.

## Current migration review

The refresh against master preserves the detailed BLE connection/message guides,
AprilTag detector/robot-observation guides and intrinsic-profile guide. Their
capability and test summaries now live in communication, vision and calibration
module pages; generated inventories replace the manual source/target tables.
Runtime implementation and existing dated hardware evidence are unchanged.

Workspace and RobotManager PRs (#208/#210) are now merged into master and
incorporated in this refresh. Merge this broad documentation change after the
remaining debug-overlay PR (#212), then refresh once more: retain their newer module pages and behavior guides, migrate their
shared architecture/testing additions into those owners, and rerun inventories,
links and tests. The current master refresh does not claim the debug overlay is merged or
eliminate conflicts against a future base.
