# Optional local code graph

Graphify helps locate source relationships and tests. It is independent of CMake,
PlatformIO, and application runtime dependencies. Source remains authoritative;
GitHub issues/native dependencies and Project #4 remain authoritative for work
scope and progress. Graph impact does not replace the required test suites.

## Setup and commands

The repository includes the `mini-robot-graph` skill under
`.agents/skills/mini-robot-graph/SKILL.md`. In a new Codex session opened in this
worktree, invoke it with `$mini-robot-graph`, for example:
"Use $mini-robot-graph to find WorldModel consumers and candidate regression
tests." Its description also allows automatic selection for structural queries.
If the client does not discover repository skills, ask it to read that file
explicitly. The skill uses the wrapper below; no upstream skill installation,
global configuration, or model key is required.

Use Python 3.12 and uv (the local pilot used uv 0.10.11). From the Git worktree
root, install the isolated, locked environment and build the graph:

```sh
uv sync --project tools/graphify --locked
python3 tools/graphify/project_graph.py build
python3 tools/graphify/project_graph.py status
python3 tools/graphify/project_graph.py query 'WorldModel'
python3 tools/graphify/project_graph.py affected 'WorldModel'
python3 tools/graphify/project_graph.py explain 'mini_robot_src_app_robotmanager_robotmanager'
python3 tools/graphify/project_graph.py path 'mini_robot_src_app_robotmanager_robotmanager' 'mini_robot_src_transport_irobottransport_irobottransport'
python3 tools/graphify/project_graph.py update
```

From `Mini_Robot/`, use `python3 ../tools/graphify/project_graph.py ...`.
The script location determines the worktree, regardless of the caller's current
directory. Terms containing spaces must be quoted. Extra upstream options are
deliberately not forwarded. The wrapper pins `graphifyy==0.9.102`, checks the
installed version, disables semantic backends and global query logging, and uses
an explicit graph path. Initial installation downloads packages; extraction and
queries operate locally. No Git hook or global agent configuration is installed.
Do not use the upstream project installer: it writes broader agent instructions.

`build` creates a clean code graph (archiving the old graph locally), then creates
the report and HTML with model labeling disabled. `update` refreshes an existing
compatible graph. Both mark output stale on failure or concurrent source changes.
After changing extraction settings/exclusions, use `build` to remove old content.
Outputs, cache, HTML, report and provenance live under ignored `graphify-out/`.

## Freshness and worktrees

`status` returns 0 for fresh and 2 for missing/stale/corrupt output. Queries refuse
stale output with code 2; fall back immediately to `rg` and source inspection if
refreshing is inconvenient. Upstream ambiguity (3) and no matching node (4) pass
through unchanged. Retry ambiguous names with the full node ID printed by Graphify.

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
generated output, Markdown/text, thesis reports and presentations. The code-only
flag skips remaining media. `Mini_Robot/firmware/` is excluded from the desktop graph.
The graph includes desktop code, the separate `Mini_Robot/firmware/ble-lifecycle/`
fixture, tests, maintained tools and GitHub automation. A `.h=cpp` override handles Arduino
and fake headers. Do not bypass Git ignores to index local credentials/config.

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
- A test edge does not prove test execution, physical behavior, safety, or full
  perception-to-motion integration. Firmware remains three independent projects.

No token/time saving is claimed: the pilot established useful navigation, not a
controlled productivity benchmark. Use normal source tools when coverage is poor.

## Verification and diagnostics

Offline wrapper tests require only Python and Git:

```sh
python3 -m unittest discover -s tools/graphify -p 'test_*.py' -v
```

They cover content/HEAD/policy invalidation, addition/rename/deletion, ignored
files, graph corruption/replacement, worktree isolation, lock contention, failed
and concurrent builds, backend policy and query error propagation. A manual
`Graphify diagnostics` Actions workflow runs these checks and a code-only build
with read-only permissions. It uploads only the graph/report and count
summary for three days; private source names remain visible to artifact readers.
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

Semantic documentation indexing, saved-answer ingestion, hosted services, MCP,
watchers/hooks, PR triage and automatic issue snapshots are not enabled. Evaluate
them separately after the local workflow is useful. Documentation would need a
separate refresh policy and explicit current/planned/historical provenance;
archives/catalogs cannot become implementation evidence. MCP would need explicit
per-worktree paths and optional dependencies, preserving source-search fallback.

## Upstream references

- [Evaluated release](https://github.com/Graphify-Labs/graphify/releases/tag/v0.9.102)
- [README and CLI setup](https://github.com/Graphify-Labs/graphify/blob/v0.9.102/README.md)
- [Architecture and confidence](https://github.com/Graphify-Labs/graphify/blob/v0.9.102/ARCHITECTURE.md)
- [Default agent rules reviewed but not installed](https://github.com/Graphify-Labs/graphify/blob/v0.9.102/graphify/always_on/agents-md.md)
