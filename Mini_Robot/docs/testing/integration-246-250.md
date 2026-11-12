# Local integration test branch: #246–#250

Prepared 2026-11-12 on `test/246-250-integration`, in an isolated worktree.
This combines the five implementation tasks with current master for integration
and manual testing. It does not merge or retarget their GitHub PRs, update issue
state, or establish physical acceptance.

## Exact inputs

| Input | Commit | Implementation PR |
| --- | --- | --- |
| Master, including merged #242–#245 | `3a3c8700ba28c34a82c20111d8c3ccdd2b692429` | Existing default branch |
| #246 signed corners/origin | `ea81b7a1f10b1c2469c7970368814bd79bf28923` | #260 |
| #247 oblique validation | `fce00aa166512e14aadc073f98bda1a2b78f42f5` | #262 |
| #248 board acquisition | `cdeb2d10e33c0d1f6194cc7df9f4a1e9c66cfb6d` | #264 |
| #249 maps/uncertainty | `1d147fd41fd6f9b791dd07c063fe69e34b14676e` | #267 |
| #250 two-point distance | `d63babbf7bf5bfa1b72b1823ba67883d441138fe` | #269 |

#250 already contains #246, #248 and #249. The branch first merges that stack
into master, then merges independent #247. Both are local merge commits; the
feature branches remain unchanged. This integration branch is not a replacement
for reviewing the individual PRs and their eventual bases/diffs.

## Integration decisions

- Preserve the detector tests/includes from master's intrinsic calibration/storage
  work alongside #248's rendered-board acquisition test.
- Retain #248's 4096-per-role spatial duplicate index, thin SVD for large datasets
  and complete four-point nullspace, together with #247's named conditioning
  threshold, recovery advice and fit-coverage reporting.
- Retain worker-based Check verification/acquisition snapshots, with feedback that
  distinguishes recoverable acquisition references from legacy profiles without
  provenance. Avoid restoring the old 64-point GUI-thread verification path.
- Preserve explicit corner ordering, signed origin and 8192-row table workflow;
  add side-camera visibility/plane/parallax guidance without restoring the old
  fixed lower-left UI or 64-point instructions.
- Normalize #247's coverage coordinates relative to the signed workspace minimum.
  The additional `TabletopObliqueTest.InteriorOriginPreservesObliqueCoverageAndHeldOutChecks`
  regression combines an oblique H, interior origin and held-out references.
- Keep tests from both branches. Adapt the oblique UI fixture to confirm ordered
  corners before adding references, retain solver-output mutation tests and use
  proper homogeneous translation when a fixture has a non-affine denominator.
- Reconcile maintained side-camera documentation with linked acquisition records
  and the separate spatial uncertainty model. Existing dated evidence remains
  evidence for its original source revision, not a claim about this combined tree.

## Executed validation

Host: macOS arm64, AppleClang 21, Qt 6.12.0, OpenCV 5.1.0. Configure commands used
an existing offline GoogleTest checkout via `FETCHCONTENT_SOURCE_DIR_GOOGLETEST`.
Build directories are local to this worktree; no other worktree's build cache or
generated graph was reused. The local graph was missing; source/`rg` inspection
was used for merge resolution.

From the worktree root:

```sh
cmake --preset core -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/path/to/googletest
cmake --build --preset core --parallel 3
ctest --preset core --parallel 3
cmake --preset macos -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/path/to/googletest
cmake --build --preset macos --parallel 3
ctest --preset macos --parallel 3
```

Root core: **332 passed**. Production desktop built; root macOS suite:
**583 passed, two opt-in audits skipped (585 selected), zero failures**.
The intentional skips are `CalibrationDetectorTest.PerformanceAudit` and
`WorkspaceSolverMemoryAudit.BoundedThinDecomposition`.

The standalone `Mini_Robot/` entry point was independently configured, built and
tested with the same core preset commands and a separate build directory:
**332 passed**.

```sh
QT_QPA_PLATFORM=offscreen QT_SCALE_FACTOR=1.5 build/macos/Mini_Robot/tests/micro_robot_tests \
  '--gtest_filter=Workspace*'
QT_QPA_PLATFORM=offscreen QT_SCALE_FACTOR=2 build/macos/Mini_Robot/tests/micro_robot_tests \
  '--gtest_filter=Workspace*'
```

Each scale run passed **40/40** workspace UI/service cases, including corners,
acquisition, oblique feedback, spatial maps and two-point measurement.

## Manual use and limits

Configure/open this worktree's root CMake project using the `macos` or `desktop`
preset. The tested root macOS application is at
`build/macos/Mini_Robot/src/micro_robot_desktop.app`. Build/run from this worktree,
so a prior feature checkout does not masquerade as the integrated application.

Physical camera, independent references/lengths, actual model coverage, native
macOS input/display, Linux, Windows and OpenCV 4 acceptance are **NOT RUN** here.
Existing physical owners #259/#261/#263/#265/#268 and #224/#63 retain their scope.
Offscreen/component results establish neither physical accuracy nor a completed
motion loop or production robot safety.
