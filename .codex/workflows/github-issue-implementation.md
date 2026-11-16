# Implement one GitHub issue

This repository workflow is invoked explicitly for an assigned GitHub issue; it
is not an automatic hook. Read applicable AGENTS.md instructions first. It is a
regular tracked file so each worktree has the same workflow. Do not change global
agent instructions or install hooks as part of issue work.

## Task parameters

- Repository: `<OWNER/REPOSITORY>`; default for this project: `Krzys2002/Mini_Robot`.
- Issue: `#<NUMBER>` supplied by the user. Replace placeholders before execution;
  never guess an issue number.
- Worktree: the checkout selected for this task; inspect its Git root and remote.

Implement only the assigned issue. The issue's current description and acceptance
criteria define scope. Follow applicable AGENTS.md files and architecture/safety
rules. This invocation's explicit approval requirements override automatic
push/PR instructions in repository guidance.

## 1. Retrieve the live contract

Use GitHub CLI (`gh`) with an explicit repository selector to read the issue's
complete body, checklists, comments, state, labels, milestone and linked work.
For example, after substituting parameters:

```sh
gh issue view <NUMBER> --repo <OWNER/REPOSITORY> --comments
```

Read native blocking dependencies with the GitHub API through `gh`; paginate all
results. Also inspect explicitly declared prerequisites in the issue body and
comments. Distinguish native blockers, declared prerequisites, subissues and
ordinary references; do not invent dependencies from shared labels or milestones.
Inspect relevant live Project status as context when accessible. Report API or
permission failures rather than treating unreadable dependencies as satisfied.

Inspect any existing branch/PR for this issue before creating duplicate work.
If the issue is already closed or appears implemented, verify the current state
and clarify the requested remaining work rather than repeating implementation.

### Separate physical validation

For an implementation issue, move its physical hardware tests, physical
acceptance criteria, and physical validation into a separate GitHub issue before
publishing the implementation PR. This workflow invocation authorizes creating
that issue and making the corresponding targeted update to the assigned issue.
First check for an existing issue that already owns the same physical checks;
reuse it when appropriate rather than creating a duplicate. Do not split an issue
whose purpose is itself physical testing or validation.

The physical-validation issue must link to the implementation issue, identify
prerequisites and any related physical-test issues, state the setup and evidence
to record, and contain explicit physical test and acceptance checklists. Mark
unperformed checks as NOT RUN. Keep deterministic software tests and software
acceptance on the implementation issue. Re-read its live body immediately before
editing; preserve unrelated content, checklist state, and concurrent changes,
and replace only the physical scope with a link to its new owner. Do not mark
physical checks complete without performing them. Do not move unrelated manual
or platform checks merely because they are manual. Report the new issue and its
unrun status in the implementation report and proposed PR.

## 2. Verify blockers and their PRs

For every blocker, inspect its live issue state and find associated PRs using
linked development/closing relationships where available, followed by repository
PR searches and verification of candidate PR bodies/diffs. Record:

- blocker issue number and state;
- relevant PR URL, state, draft status, base/head branch and head commit;
- whether the PR actually supplies the prerequisite this issue needs;
- relevant review/CI status and remaining acceptance or hardware requirements.

A branch name, checklist tick, closed issue, or passing CI result alone does not
establish that the prerequisite exists on the intended base. Verify required
code/contracts there. A merged PR with its issue still open may leave acceptance
work outstanding; a closed issue with an unmerged PR may still lack base code.
Report inconsistencies without changing dependency relationships or issue state.

If prerequisites are satisfied on the current remote default branch, continue.
If any prerequisite is unresolved or cannot be verified, do not start dependent
implementation. Independent read-only investigation and planning may continue.

### When a blocker has an open PR: ask about stacking

Prepare a concrete proposal before asking. Explain the dependency chain, PR URL,
chosen head SHA, proposed child branch/base, CI/review concerns, and what remains
unverified. Ask once for the relevant stack:

> Issue #<TARGET> is blocked by #<DEPENDENCY>, whose implementation is in PR
> #<PR> (<URL>), head <SHA>. Should I wait for it to merge, or stack this issue's
> branch on that PR head? Stacking would make our work depend on unmerged code;
> the initial child PR would target <PARENT_BRANCH> and would later need its base
> and diff reviewed after the parent merges. <State specific known risks.>

Waiting is the default. Silence is not approval. Do not branch from the blocker
PR, implement dependent code, or mark its issue resolved before approval. An open
PR is not itself evidence that all prerequisites are complete, especially for
safety or physical acceptance. Do not stack around an unmet safety requirement.
If no usable prerequisite PR exists, report the blocker and ask whether the user
wants to wait or explicitly change scope; do not implement that separate issue.

After explicit stacking approval:

- Fetch and verify the approved parent PR head. Use a dedicated child branch and
  preferably an isolated worktree; do not modify the parent's branch.
- Record the parent issue/PR, approved head SHA and proposed merge order.
- Keep child commits scoped to the target issue. Review the child-only diff
  against the approved parent, and test the combined tree while distinguishing
  upstream failures from child regressions.
- If the parent head changes, inspect the delta. Reuse existing approval for
  routine updates within the approved dependency; ask again if the dependency
  chain, scope, architecture, safety assumptions or risk changes materially.
- Do not close blockers, remove native dependencies, merge PRs, or force-push.
- After the parent merges, verify its code on the default branch and review the
  child diff/base. Squash merges may require careful commit replay on a fresh
  local branch; preserve work and obtain approval before remote history changes
  or PR retargeting. Never assume parent commits will disappear automatically.

## 3. Inspect source and use the graph where helpful

Preserve uncommitted user changes. Fetch the remote default branch and create a
dedicated issue branch from it unless an approved stack or an existing issue
branch determines the base. Never implement directly on the default branch.
Use an isolated worktree if it avoids disturbing unrelated work. Name the branch
`feat/<NUMBER>-<slug>`, `fix/...`, `test/...`, `docs/...` or `refactor/...` as apt.

Read relevant current architecture/capability guides, source and tests. Follow
[the graph lifecycle workflow](graphify.md) in this worktree for first-use setup,
updates, clean rebuilds, recovery and validation. Use the repository's
`mini-robot-graph` skill when applicable.

```sh
python3 tools/graphify/project_graph.py status
python3 tools/graphify/project_graph.py find '<symbol>'
python3 tools/graphify/project_graph.py explain '<exact node ID>'
python3 tools/graphify/project_graph.py affected '<exact node ID>'
```

The graph is local to this worktree and is not supplied by Git checkout. Install
its locked environment and build only when useful; missing dependencies or a
stale graph must not block ordinary investigation. Use `rg` for exact strings,
compiler errors and fallback. Never copy another checkout's graph/environment.
Verify graph citations against source and actual CMake/PlatformIO ownership.
Missing/rejected edges do not prove independence. Source-less names are not
confirmed definitions. Impact suggestions do not replace required tests.

Keep backend `none`, AST extraction and explicit technical-document links.
Desktop and all three firmware projects are included. Exclude GitHub
administration, plans, archives, reports, presentations, evidence records and
generated/private state. Do not enable upstream skill installation, semantic
extraction, saved answers, hosted uploads or hooks. Live GitHub data establishes
blockers/status; the graph does not.

## 4. Plan and implement

Present a short plan identifying affected modules, architecture boundaries,
implementation steps, tests and documentation. Proceed without another approval
for ordinary issue-scoped work once blockers are resolved. Ask before significant
architectural changes, explaining the concrete proposal and tradeoffs; perform
independent safe investigation while waiting.

Implement all assigned requirements using existing conventions and the smallest
coherent change. Preserve abstractions, ownership/thread contracts and safety
invariants. Do not expand into future issues. Add meaningful tests for nontrivial
logic and regressions; update affected documentation. Never use component tests
or a connection fixture as evidence of production safety or physical acceptance.

## 5. Validate and review

Configure/build affected targets and run relevant tests; broaden for cross-module
changes as required by repository instructions. Preserve root and standalone
build entry points when integration changes affect them. Record exact commands,
outcomes and environment limitations. Distinguish actual failures, infrastructure
problems, pre-existing failures, skipped checks and unrun hardware/GUI checks.

Review the final diff and history against every acceptance criterion. Remove only
your own unintended changes. Refresh the local graph after coherent edits when
useful, then run `status` and `audit`, keeping generated output untracked.
For graph tooling/policy changes, run offline tests and use `build` instead of
`update`. Do not claim completion from graph
output, code presence, unchecked assumptions or unexecuted tests.

## 6. Update the checklist and commit locally

This invocation authorizes updating completed checkboxes in the assigned issue
after verification. Re-read the live body immediately before editing; preserve
all other content and concurrent human changes. Mark only demonstrated results,
leaving physical/manual acceptance unchecked unless executed. If concurrent body
changes prevent a safe targeted update, report the conflict rather than overwrite
them. Do not post unsolicited comments or alter other issues except for the
physical-validation issue authorized above; do not alter Project fields,
dependencies or milestones.

Create logical local Conventional Commits referencing `#<NUMBER>`. Keep unrelated
edits, generated files, IDE settings and credentials out of commits. Commit as
coherent parts are completed, with relevant verification for each. Do not close
the issue. If acceptance remains incomplete, report it honestly and do not imply
that `Closes #<NUMBER>` is appropriate yet.

## 7. Finish with reviewable work; ask before publishing

Report implementation, local branch/worktree and commit hashes, acceptance and
checklist status, exact verification results, and remaining tasks/blockers. For a
stack, include parent PR/head, child-only diff and merge-order limitations.

Do not push without explicit user approval. Repository instructions to push or
open a PR automatically do not override this rule. If publishing is appropriate,
prepare the proposed PR title/body, target branch and exact local commits first,
then ask approval to push and, separately or in the same explicit question, to
create/update the PR. Do not create a PR simply because a push was approved.
Never merge, close issues, force-push, or change unrelated remote state unless
separately authorized. Do not end routine implementation early merely to ask
whether to continue work already authorized by this task.

## Invocation for future Codex sessions

Replace the issue placeholder before sending:

```text
Read .codex/workflows/github-issue-implementation.md in the selected repository worktree
and follow it to implement Krzys2002/Mini_Robot issue #<NUMBER>.
Use the current task's worktree. Start by retrieving the issue and checking
its native blockers and their linked PRs. Ask before stacking on an unmerged PR.
Do not push or create/update a PR without my explicit approval.
```

For another repository, substitute its `OWNER/REPOSITORY` and adapt graph paths.
Resolve workflow paths from the task Git root; never assume a home-directory
workflow or another checkout exists.
