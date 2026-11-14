# Architecture review for PRs #260–#269

Implementation worktree: `fix/requested-architecture-260-269`.
Baseline: existing integration `57534f4` (no PR merge, retarget or history rewrite).
Integrated source revisions:

| PR | Revision |
| --- | --- |
| #260 | ea81b7a1f10b1c2469c7970368814bd79bf28923 |
| #262 | fce00aa166512e14aadc073f98bda1a2b78f42f5 |
| #264 | cdeb2d10e33c0d1f6194cc7df9f4a1e9c66cfb6d |
| #267 | 1d147fd41fd6f9b791dd07c063fe69e34b14676e |
| #269 | d63babbf7bf5bfa1b72b1823ba67883d441138fe |

Other numbers in the requested range are issues, not PRs.

The patch preserves the signed-frame/profile migration and normalized horizon
fix. It adds optional board-surface elevation/thickness provenance with a v2
writer and backward v1 reader, tabletop/frame/canonical revision labels, observed
signed per-axis comparisons, and documentation of successor ownership and limits.
A null vision batch now clears raw/tracked coordinates and reports Unknown;
non-null invalid batches still mark Lost. Late generation results remain fenced,
and pre-invalidation capture timestamps cannot recreate a track. Robot identities
remain registered. The current integrated tracker has no filter state.

Regression coverage includes the 1,525-point capacity dataset with elevation
round trip, v1 unknown-value migration and schema mismatch rejection, signed-origin
replacement during a blocked detector, stale bindings, JSON provenance and per-axis
presentation. Existing immutable sidecar mismatch checks and original evidence
are retained. The review patch was first validated above the existing combined tree, then split
into append-only local commits on the five owning PR branches. No history was rewritten.

Validation uses the existing GoogleTest checkout via
`-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/Users/krzysztofsawicki/Documents/Put/sem6/Term_Design/build/macos/_deps/googletest-src`.
The first core configure attempted the pinned download and was interrupted;
subsequent core and host configuration used that offline checkout.

Physical checks #259/#261/#263/#265/#268: **NOT RUN**. No extrinsic estimation,
height compensation, static reference monitoring, body uncertainty propagation,
or physical ±5 mm acceptance is claimed. Follow-ups #270/#271/#272 remain open.

## Executed validation

- `cmake --preset core` with the offline source override above: configured.
- `cmake --build --preset core --parallel 8`: passed.
- `ctest --preset core`: 332/332 passed after the reset-contract change.
- `cmake --preset macos` with the same offline override: configured.
- `cmake --build --preset macos --parallel 8`: passed, including desktop application.
- `QT_QPA_PLATFORM=offscreen ctest --preset macos`: no failures across 585 entries;
  performance and solver-memory audits were skipped (583 passed, 2 skipped).
- After adding final label assertions, rebuilt and ran
  `QT_QPA_PLATFORM=offscreen ctest --preset macos -R 'WorkspaceDiagnosticsUiTest|WorkspaceDistanceUiTest'`:
  14/14 passed.
- `git diff --check`: passed.

The initial reset-contract run exposed two expectations that retained Lost state
across a cleared input. They were updated to require Unknown and absent pose/tracking
state, while retaining timestamp fencing and recovery assertions; final runs above
passed. No physical camera, BLE or motor validation was performed.

Graph refresh was attempted once with `python3 tools/graphify/project_graph.py update`;
it reported a missing worktree-local snapshot. Source inspection was used instead;
no other worktree's derived graph was reused.

## Local PR commits (not pushed)

| PR | Branch | Commit |
| --- | --- | --- |
| #260 | `feat/246-workspace-corners-origin` | `d0f97961399dd30435d9ebc87b299e2c0676012d` |
| #262 | `fix/247-oblique-tabletop-validation` | `951158d047b5f39de4780449590b58fe0bc1aa9d` |
| #264 | `feat/248-workspace-chessboard-acquisition` | `05f121425b8b42cf6e3e490093a0105da95b1386` |
| #267 | `feat/249-workspace-scale-error-maps` | `fd5510d9910772590515e4f8a083670bc160cffb` |
| #269 | `feat/250-workspace-distance` | `7c2188d30096226c9ae1ec29de270a334c5c9516` |

Each PR commit contains identical added/removed lines to its corresponding commit
on the combined review branch. Existing branch documentation was retained when
resolving two append-context conflicts. Later PR feature changes were not merged
into earlier PR branches; existing PR bases and ancestry remain unchanged.

During the split, guide additions were moved to their owning module guides. The
distance plane now uses its existing explicit result default instead of reading
a newly added diagnostics field, so #269 does not require #267's new field before
upstream integration. After this adjustment, `cmake --build --preset macos --parallel 8`
and `QT_QPA_PLATFORM=offscreen ctest --preset macos -R 'WorkspaceDistance'` passed
(17/17). Earlier full-suite results above apply to the combined implementation;
individual PR branch suites were not rerun during commit distribution.
