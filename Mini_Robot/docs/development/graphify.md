# Optional local code graph

Graphify helps locate source relationships and tests. It is independent of CMake,
PlatformIO, and application runtime dependencies. Source remains authoritative;
GitHub issues/native dependencies and Project #4 remain authoritative for work
scope and progress. Graph impact does not replace the required test suites.

## Setup and commands

The portable runbook is [`.codex/workflows/graphify.md`](../../../.codex/workflows/graphify.md).
It is also integrated into the explicitly invoked
[issue implementation workflow](../../../.codex/workflows/github-issue-implementation.md).
Both are regular repository files, not symlinks to a user's home directory.
Workflow Markdown is a project convention that agents read explicitly; it is
not an automatic Codex execution hook. Root `AGENTS.md` links the runbooks.

The repository includes the `mini-robot-graph` skill under
`.agents/skills/mini-robot-graph/SKILL.md`. In a new Codex session opened in this
worktree, invoke it with `$mini-robot-graph`, for example:
"Use $mini-robot-graph to find WorldModel consumers and candidate regression
tests." Its description also allows automatic selection for structural queries.
If the client does not discover repository skills, ask it to read that file
explicitly. The skill uses the wrapper below; no upstream skill installation,
global configuration, or model key is required.

Use Python 3.12 and uv 0.13.7 (used for the current lockfile and diagnostics). From the Git worktree
root, install the isolated, locked environment and build the graph:

```sh
uv sync --project tools/graphify --locked
python3 tools/graphify/project_graph.py build
python3 tools/graphify/project_graph.py status
python3 tools/graphify/project_graph.py audit
python3 tools/graphify/project_graph.py find 'WorldModel'
python3 tools/graphify/project_graph.py query 'WorldModel'
python3 tools/graphify/project_graph.py affected 'WorldModel'
python3 tools/graphify/project_graph.py explain 'mini_robot_src_app_robotmanager_robotmanager'
python3 tools/graphify/project_graph.py path 'mini_robot_src_app_robotmanager_robotmanager' 'mini_robot_src_transport_irobottransport_irobottransport'
python3 tools/graphify/project_graph.py update
```

From `Mini_Robot/`, use `python3 ../tools/graphify/project_graph.py ...`.
The script location determines the worktree, regardless of the caller's current
directory. Terms containing spaces must be quoted. Extra upstream options are
deliberately not forwarded. The wrapper pins `graphifyy==0.9.106`, checks the
installed version, disables semantic backends and global query logging, and uses
an explicit graph path. Initial installation downloads packages; extraction and
queries operate locally. No Git hook or global agent configuration is installed.
Do not use the upstream project installer: it writes broader agent instructions.

`build` creates a clean code graph (archiving the old graph locally), then creates
the report and HTML with model labeling disabled. `update` refreshes an existing
compatible graph. Both mark output stale on failure or concurrent source changes.
After changing extraction settings/exclusions, use `build` to remove old content.
Both commands then add explicit technical-document links and regenerate clusters,
the report and HTML. Outputs, cache, HTML, report and provenance live under ignored `graphify-out/`.

Before document linking, `source_checks.py` rejects C++ call edges that bind
application/embedded code to host-test symbols without a configured build
context, or whose target spelling is unsupported at the recorded call site.
Known rejections are recorded in `graphify-out/source-checks.json` with source
hashes. Updates retain earlier omissions while the source is unchanged;
`rejected_this_operation` distinguishes new removals from retained records.
Rejected edges remain unresolved: this pass does not fabricate replacement calls.
The real controller is compiled against fakes by the host safety-test target;
that target-specific fact does not make its fakes dependencies of PlatformIO builds.

## Freshness and worktrees

`status` returns 0 for fresh and 2 for missing/stale/corrupt output. Queries refuse
stale output with code 2; fall back immediately to `rg` and source inspection if
refreshing is inconvenient. Upstream ambiguity (3) and no matching node (4) pass
through unchanged. Retry ambiguous names with the full node ID printed by Graphify.

Use `find 'Symbol'` to discover up to 20 exact symbol matches and their IDs;
otherwise it searches labels/source paths. Follow with `explain '<id>'` for one
bounded neighborhood. `query '<exact id>'` also uses that exact neighborhood,
avoiding upstream token expansion into unrelated seeds. Use `affected '<id>'`
for candidate consumers/tests and `path '<id A>' '<id B>'` for a relationship.
Read edge direction, confidence and source lines before acting. A graph path is
not necessarily an execution path. Exact-string/error searches still use `rg`.

`audit` returns JSON and exits 0 when the implemented checks pass, 2 otherwise.
It checks unique IDs/edges, edge endpoints, source and definition file/line
locations, excluded paths, explicit documentation-reference evidence and the C++
call guard. It also reports source-less nodes and confidence counts. It does not
measure precision/recall or certify complete call coverage. `audit` and `find`
need Python/Git and a fresh graph; they do not import Graphify's dependencies.

The repository shares the wrapper, policy, tests, skill and instructions.
`tools/graphify/.venv/` and `graphify-out/` are ignored, per-worktree artifacts;
they are not distributed through GitHub commits. The manual diagnostics workflow
can upload temporary artifacts when explicitly invoked. Normal local navigation
does not invoke it or upload the graph. Root `AGENTS.md` carries the durable agent
workflow; no machine path or global agent configuration is required.

The provenance file records the worktree's absolute root, HEAD, included file
hashes, pinned Graphify version, profile, graph digest and timestamp. Hashing
deliberately includes nonignored documentation/build contracts as well as code;
these can invalidate navigation even though they are not graph content. Ignored
tracked files and build/cache output are excluded. A source symlink's target text
is recorded without reading the target. The inventory is conservative, so an
unrelated nonignored asset may also require a refresh. No source files are changed.

Check after checkout, pull, rebase or merge; refresh after coherent code changes.
Each worktree needs its own environment and graph. A copied provenance file is
rejected. Wrapper operations serialize with an exclusive local lock. If a process
is killed, verify it exited before removing `graphify-out/.project-operation.lock`.
Do not run upstream writers concurrently: they do not honor this wrapper lock.
Source can still change after a freshness check; verify cited lines before use.

## Corpus and limits

The scan root is the whole Git worktree. `.graphifyignore` and existing Git ignore
rules exclude builds, dependencies, environments, IDE state, local agent files,
generated output, GitHub automation/administration, plans, archives, evidence
records, thesis reports and presentations. The code-only flag skips remaining
media. The graph includes desktop code, `T1-ESP-C6/`, `T1-ESP-S3/`, the separate
`Mini_Robot/firmware/ble-lifecycle/` fixture, tests and maintained local tools.
A `.h=cpp` override handles Arduino and fake headers. CAD, STL and other binary
physical-design assets are not parsed. Do not bypass Git ignores to index local credentials/config.

The `code-and-linked-technical-docs` profile adds documentation locally, without a
model or semantic backend. `tools/graphify/doc_links.py` allowlists technical
overview, protocol, vision and development guides, application/firmware READMEs,
and the hardware/connection test procedures. GitHub automation guides, planning,
archives, reviews, reports and recorded test results are not candidates. A doc
must reference an extracted source file directly or through another selected doc;
disconnected docs and doc-only cycles are omitted. Markdown links and backtick
repository paths create `documents`/`references` edges. External URLs are ignored.
These edges record a reference, not verified behavior or implemented requirements.
The overlay is replaced on every build/update so removed links and docs disappear.

The first compatibility pilot on master `8800ec0` plus the initial wrapper indexed
193 code files: 3,938 nodes, 7,654 edges and 199 communities. Counts change as this
tooling changes. `affected WorldModel` located DesktopController, RobotManager,
world/transport/UI tests; class queries required disambiguating constructors.
Fourteen scanned files yielded no symbols, including QtBleBackend.hpp; its file
node exists, but this does not establish method/dispatch coverage.

- The graph is an AST/navigation approximation, not compiler or linker truth.
- Qt signal/slot dispatch, overloads, templates, build conditionals and injected
  concrete implementations may be missing or inferred. Inspect source and CMake.
- CMake/PlatformIO contracts are not represented as configured target graphs.
- Source-less/external concepts and cross-target same-name matches need review.
- Confidence tags describe extraction/inference; they do not prove correctness.
- The 0.9.106 upgrade restores cross-header field/member calls and separates
  accessor nodes from backing fields in this repository. C++ class/free-function
  case collisions and local lambda calls can still be unresolved. The source
  guard suppresses observed false bindings; it is not a full C++ resolver.
- A test edge does not prove test execution, physical behavior, safety, or full
  perception-to-motion integration. Firmware remains three independent projects.

No token/time saving is claimed: the pilot established useful navigation, not a
controlled productivity benchmark. Use normal source tools when coverage is poor.

## Verification and diagnostics

### Agent usefulness assessment and changes — 2026-11-11

Engineering judgment, not a measured productivity benchmark: before this pass,
**7/10 overall for agent navigation**. The strongest uses are finding source/docs
and callers (8/10); candidate test/impact discovery is useful but incomplete
(6/10); compiler/build/runtime correctness is weaker (4/10). These ratings are
about fitness for a task, not percentages of correct edges.

Six local command probes took approximately 0.16–0.48 seconds each. `WorldModel`
correctly required disambiguation, and `affected` found source/test consumers.
However, querying its full ID expanded to 339 nodes and truncated at 57. A no-op
update also erased the diagnostic history of 47 suppressed calls. The exact-ID
route and retained diagnostics below address those observed usability problems.
With the changes, the overall navigation rating is **8/10**, conditional on using
the source-verification workflow. No controlled token/time saving is claimed.

Changes across this maintenance session:

1. Installed an isolated locked environment; upgraded Graphify 0.9.102 to
   0.9.106 and regenerated its lock. Diagnostics use uv 0.13.7 for that lockfile.
2. Included desktop/test/local-tool code and all three firmware projects.
   Excluded GitHub administration/automation, plans, archives, reports,
   presentations, evidence records, generated files and private local state.
3. Added allowlisted technical-document nodes and explicit local reference edges;
   removed disconnected docs and stale links without using a semantic backend.
4. Restored tracker-to-WorldModel member calls and correct accessor identities
   through upstream C++ fixes. Added a conservative guard suppressing 46
   application/firmware-to-test bindings and one case-collision call in the audited
   corpus. Suppression leaves these calls unresolved rather than inventing targets.
5. Added `find`, exact-ID `query` routing and `audit`, plus source-hashed rejection
   history across updates and regression tests for the new behavior.
6. Updated root/subtree agent guidance, the repository skill, this guide and
   manual diagnostics; generated outputs remain ignored and local by default.
7. Replaced the local-only home-directory workflow link with a portable issue
   workflow and added a graph lifecycle runbook under `.codex/workflows/`, covering
   new worktrees, offline package-cache reuse, incremental refresh, clean rebuilds,
   corruption/version/lock recovery and post-commit freshness.

Remaining improvements, in order of likely value: upstream case-sensitive
function/type IDs and local lambda extraction; target-aware compiler/SCIP input
for independent firmware/desktop/host-test builds; Qt signal/slot evidence; and
more representative source-grounded recall fixtures. These require further
evaluation and are not implemented here. Do not enable broader semantic indexing,
hosted services or automatic hooks just to improve the rating.

Current local validation (macOS, Python 3.12, Graphify 0.9.106):

- `python3 -m unittest discover -s tools/graphify -p 'test_*.py'`: 38 passed.
- Wrapper `build`, `update`, `status`, `audit`, `find WorldModel`, exact-ID
  `query`, `explain`, `affected` and tracker-to-WorldModel `path`: passed.
- Audit: 4,428 nodes, 9,012 edges, 49 technical documents, 484 explicit doc
  edges checked, no integrity errors. The 1,335 nodes without source definitions
  are unresolved/external names; their presence limits definition/impact claims.
- Regression checks cover duplicate/dangling nodes, invalid source locations,
  excluded sources, exact ID routing and retention/expiry of rejection evidence.
- `git diff --check` and manual-workflow YAML parsing passed. The hosted workflow,
  application builds, GUI and physical hardware were not run for these tooling edits.

### Fresh worktree verification — 2026-11-11

Verified against committed implementation `35abf74` in a disposable detached Git
worktree, with neither the original graph nor its environment copied:

- Initial `status` returned 2 for missing graph/provenance, as expected.
- `uv sync --project tools/graphify --locked --offline` with a shared package
  cache created a separate worktree-local environment successfully.
- All 38 offline wrapper tests passed in the new worktree.
- Real `build`, `status`, `audit`, `find WorldModel` and the exact-ID
  RobotTracker-to-WorldModel path passed. Provenance recorded the new root.
- Both `.codex/workflows/` files arrived as regular tracked files with no
  home-directory paths. They require explicit reading; no hook was installed.
- Adding a temporary C++ class, running `update`, then finding it succeeded.
  Deleting that file and updating removed its nodes. Final audit passed with
  the baseline 4,428 nodes, 9,012 edges and all 49 technical documents restored.
- The temporary source was removed and the worktree was clean. No application,
  GUI, hardware, hosted-runner or Windows/Linux execution was part of this check.

After committing, re-run `update`, `status` and `audit` in the active checkout:
HEAD changes invalidate provenance even when source bytes are unchanged.

Offline wrapper tests require only Python and Git:

```sh
python3 -m unittest discover -s tools/graphify -p 'test_*.py' -v
```

They cover content/HEAD/policy invalidation, addition/rename/deletion, ignored
files, graph corruption/replacement, worktree isolation, lock contention, failed
and concurrent builds, backend policy and query error propagation. A manual
`Graphify diagnostics` Actions workflow runs these checks and a code-only build
with read-only permissions. It uploads only the graph/report and count
summary and rejected-call diagnostics for three days; private source names remain visible to artifact readers.
It writes no Project state/comments/commits and is not a required PR check.

Missing environment: rerun the locked `uv sync`. Corrupt/incompatible graph: run
`build`. For a Graphify upgrade, deliberately change the pin, regenerate the
lock, repeat the pilot and tests, then rebuild; never silently upgrade.

Local verification on 2026-11-04 (macOS, Python 3.12):

- `python3 -m unittest discover -s tools/graphify -p 'test_*.py' -v`: 20 passed.
- `python3 -m unittest discover -s .github/scripts/tests -v`: 28 passed.
- Real wrapper build/update/status/query/path/explain/affected smoke checks:
  passed, including root/subdirectory invocation and temporary source addition,
  edit, rename and deletion. The temporary sources were removed afterward.
- Workflow YAML parsed; manual-only trigger, read-only permissions and timeout
  checked. The hosted Actions job has not run.
- `git diff --check`: passed. No application/CMake/firmware sources changed;
  application builds, GUI, hardware, Linux and Windows execution were not run.

## Deferred extensions

Semantic documentation indexing (distinct from the explicit links above), saved-answer ingestion, hosted services, MCP,
watchers/hooks, PR triage and automatic issue snapshots are not enabled. Evaluate
them separately after the local workflow is useful. Documentation would need a
separate refresh policy and explicit current/planned/historical provenance;
archives/catalogs cannot become implementation evidence. MCP would need explicit
per-worktree paths and optional dependencies, preserving source-search fallback.

## Upstream references

- [Evaluated release](https://github.com/Graphify-Labs/graphify/releases/tag/v0.9.106)
- [README and CLI setup](https://github.com/Graphify-Labs/graphify/blob/v0.9.106/README.md)
- [Architecture and confidence](https://github.com/Graphify-Labs/graphify/blob/v0.9.106/ARCHITECTURE.md)
- [Default agent rules reviewed but not installed](https://github.com/Graphify-Labs/graphify/blob/v0.9.106/graphify/always_on/agents-md.md)
