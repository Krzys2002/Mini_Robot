---
name: mini-robot-graph
description: Explore Mini_Robot code relationships and change impact using the repository's local Graphify wrapper. Use for callers, interface consumers, dependency paths, and locating candidate tests. Exact-string searches and live GitHub issue/status questions can use their normal tools directly.
---

# Mini Robot code graph

Use the graph to locate relevant source and tests, then inspect those files to
answer or implement the user's request. Read applicable `AGENTS.md` instructions
and any assigned issue first; this skill does not expand implementation scope.

## Select the current worktree

Locate `tools/graphify/project_graph.py` in the task's Git worktree. Run commands
from that root, or use its absolute script path from a subdirectory. Do not use a
wrapper or output directory from a different checkout. There is no hardcoded
machine path and no global `graphify` executable requirement.

```sh
python3 tools/graphify/project_graph.py status
```

Exit 0 means fresh. Exit 2 means missing/stale/corrupt output or a tool error:
read the message. If graph exploration is worthwhile, use `build` for first use,
corruption, foreign output, or changed extraction policy; use `update` for source
or HEAD changes with compatible provenance. If setup/refresh is unavailable or
fails, continue with `rg` and source inspection. Do not repeatedly retry a failing
refresh. Setup and recovery details are in the
[development guide](../../../Mini_Robot/docs/development/graphify.md); read it
only when needed. Installation downloads packages and remains subject to the
environment's network permissions.

## Choose a scoped operation

All commands use `python3 tools/graphify/project_graph.py` followed by:

| Intent | Arguments |
| --- | --- |
| Find exact symbols/IDs or narrow by source path | `find 'WorldModel'` |
| Check integrity, source locations and explicit doc references | `audit` |
| Find relevant concepts | `query 'WorldModel'` |
| Inspect a specific symbol and its neighbors | `explain '<symbol or full node ID>'` |
| Trace a connection | `path '<node A>' '<node B>'` |
| Find consumers and candidate regression tests | `affected 'WorldModel'` |

Quote each term; `find`/`query` accept one string and `path` accepts two. The wrapper
does not forward upstream flags. Prefer short queries using actual project
identifiers: the matcher does not provide general semantic/synonym search. For a
question about changing the robot transport boundary, start with `IRobotTransport`
and its consumers rather than a long prose question. When vocabulary does not
match, locate a likely symbol with scoped source search and query that symbol.

- Exit 3 lists ambiguous candidates. Select the candidate by source path and
  line, then retry using its full node ID. Constructors/destructors and classes
  can share labels. For example, the RobotManager class currently has ID
  `mini_robot_src_app_robotmanager_robotmanager`; prefer freshly returned IDs
  over assuming this example remains unchanged. Prefer `find` then `explain`;
  `query '<exact id>'` now routes to that one symbol's neighborhood instead of
  tokenizing the ID into unrelated search seeds.
- Exit 4 means no matching node. Try a source-grounded name if readily available,
  otherwise fall back to source search. It does not establish that the feature or
  dependency is absent.
- For relationships, inspect edge direction, confidence, and source locations.
  An inferred edge needs source confirmation; a missing edge is inconclusive.
- Use `affected` to find test candidates, then read test bodies and CMake
  registrations. Graph impact is not a complete test selection or test result.

## Verify and refresh

Verify the relevant source lines before editing or making behavioral claims.
Run `audit` after graph tooling changes. Inspect `graphify-out/source-checks.json`
for known rejected call edges; these remain unresolved. The graph/environment
are ignored per-worktree artifacts, while the wrapper/instructions are repository
files. Do not invoke the manual GitHub diagnostics workflow for local navigation.
Qt signals/slots, templates, injected concrete implementations, CMake target
ownership and conditional builds may be incompletely modeled. Distinguish paths
found in the graph from relationships established by source inspection. Report
useful source/test locations and material gaps, rather than dumping the graph.

After coherent source changes, refresh once with `update` when useful. Let the
main agent own refreshes when subagents share a worktree. Use the wrapper only:
direct upstream writers bypass its freshness/locking checks. Preserve the
backend `none`, code extraction plus explicit technical-document links, ignored
output, and `.graphifyignore`. Desktop and standalone firmware are included;
GitHub administration, plans, reports and presentations are excluded. Do not install upstream agent rules,
enable semantic extraction, save answers into the graph, add hooks, or upload
content as part of this exploration workflow.

For issue dependencies and progress, consult live GitHub data. Neither a graph
edge nor a passing component test establishes calibrated closed-loop operation
or physical motor/watchdog/E-stop safety.
