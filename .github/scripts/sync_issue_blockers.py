#!/usr/bin/env python3
"""Synchronize Todo/Blocked in Project #4 using native issue dependencies.

Requires Python 3.10+ and an authenticated GitHub CLI. No third-party Python
packages are required. The only mutation is updateProjectV2ItemFieldValue for
the existing Status field. Project configuration is never changed.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
import re
import subprocess
from typing import Any


FIELDS_QUERY = """query SyncProjectFields($owner:String!, $number:Int!, $cursor:String) {
  user(login:$owner) { projectV2(number:$number) {
    id fields(first:100, after:$cursor) {
      nodes { __typename
        ... on ProjectV2Field { id name }
        ... on ProjectV2SingleSelectField { id name options { id name } }
        ... on ProjectV2IterationField { id name }
      } pageInfo { hasNextPage endCursor }
    }
  } }
}"""

ITEMS_QUERY = """query SyncProjectItems($project:ID!, $cursor:String) {
  node(id:$project) { ... on ProjectV2 {
    id items(first:100, after:$cursor) { nodes {
      id type content { __typename ... on Issue {
        id number repository { nameWithOwner }
      } }
      fieldValueByName(name:"Status") {
        ... on ProjectV2ItemFieldSingleSelectValue { optionId name }
      }
    } pageInfo { hasNextPage endCursor } }
  } }
}"""

RECHECK_QUERY = """query RecheckProjectItem($item:ID!) {
  node(id:$item) { ... on ProjectV2Item {
    id project { id } content { __typename ... on Issue {
      id number repository { nameWithOwner }
    } }
    fieldValueByName(name:"Status") {
      ... on ProjectV2ItemFieldSingleSelectValue { optionId name }
    }
  } }
}"""

STATUS_MUTATION = """mutation SetProjectStatus(
  $project:ID!, $item:ID!, $field:ID!, $option:String!
) {
  updateProjectV2ItemFieldValue(input:{
    projectId:$project, itemId:$item, fieldId:$field,
    value:{singleSelectOptionId:$option}
  }) { projectV2Item { id } }
}"""


class ApiError(RuntimeError):
    """An API operation failed or returned incomplete/unexpected data."""


def redact(message: str) -> str:
    for key in ("GH_TOKEN", "GITHUB_TOKEN", "PROJECT_TOKEN"):
        token = os.environ.get(key)
        if token:
            message = message.replace(token, "[REDACTED]")
    return re.sub(r"(?:github_pat_|gh[pousr]_)[A-Za-z0-9_]+", "[REDACTED]", message)


def log(event: str, **values: Any) -> None:
    # JSON escapes untrusted names/newlines and cannot become workflow commands.
    print(json.dumps({"event": event, **values}, ensure_ascii=True), flush=True)


class GitHubApi:
    def request(self, endpoint: str, payload: dict | None = None) -> Any:
        command = [
            "gh", "api", "--hostname", "github.com",
            "-H", "Accept: application/vnd.github+json",
            "-H", "X-GitHub-Api-Version: 2026-03-10", endpoint,
        ]
        if payload is not None:
            command.extend(["--method", "POST", "--input", "-"])
        try:
            result = subprocess.run(
                command, input=json.dumps(payload) if payload is not None else None,
                text=True, capture_output=True, timeout=45, check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise ApiError(redact(f"gh api failed: {exc}")) from exc
        if result.returncode:
            raise ApiError(redact(result.stderr.strip() or "gh api exited unsuccessfully"))
        try:
            return json.loads(result.stdout)
        except json.JSONDecodeError as exc:
            raise ApiError("gh api returned invalid JSON") from exc

    def graphql(self, query: str, variables: dict) -> dict:
        result = self.request("graphql", {"query": query, "variables": variables})
        if not isinstance(result, dict):
            raise ApiError("GraphQL returned an unexpected response")
        if result.get("errors"):
            raise ApiError(redact("GraphQL errors: " + json.dumps(result["errors"])))
        if not isinstance(result.get("data"), dict):
            raise ApiError("GraphQL returned no data; check PROJECT_TOKEN permissions")
        return result["data"]

    def blockers(self, repository: str, number: int) -> list[dict]:
        output = []
        page = 1
        while True:
            endpoint = f"repos/{repository}/issues/{number}/dependencies/blocked_by?per_page=100&page={page}"
            result = self.request(endpoint)
            if not isinstance(result, list):
                raise ApiError("Native dependency API returned an unexpected response")
            if any(not isinstance(x, dict) or x.get("state") not in ("open", "closed") for x in result):
                raise ApiError("Native dependency API returned a blocker without a valid state")
            output.extend(result)
            if len(result) < 100:
                return output
            page += 1


@dataclass(frozen=True)
class StatusConfig:
    project_id: str
    field_id: str
    todo_id: str
    blocked_id: str


@dataclass
class Summary:
    scanned: int = 0
    skipped: int = 0
    changed: int = 0
    would_change: int = 0
    failed: int = 0

    @property
    def exit_code(self) -> int:
        return 1 if self.failed else 0


def next_cursor(connection: dict, previous: set[str]) -> str | None:
    info = connection.get("pageInfo")
    if not isinstance(info, dict) or not isinstance(info.get("hasNextPage"), bool):
        raise ApiError("GraphQL pagination metadata is missing")
    if not info["hasNextPage"]:
        return None
    cursor = info.get("endCursor")
    if not cursor or cursor in previous:
        raise ApiError("GraphQL pagination returned a missing or repeated cursor")
    previous.add(cursor)
    return cursor


def load_config(api: GitHubApi, owner: str, number: int) -> StatusConfig:
    cursor = None
    cursors: set[str] = set()
    fields = []
    project_id = None
    while True:
        data = api.graphql(FIELDS_QUERY, {"owner": owner, "number": number, "cursor": cursor})
        project = (data.get("user") or {}).get("projectV2")
        if not project or not project.get("id"):
            raise ApiError(f"Cannot read user Project {owner}/{number}; check PROJECT_TOKEN access")
        if project_id and project_id != project["id"]:
            raise ApiError("Project identity changed while reading fields")
        project_id = project["id"]
        connection = project["fields"]
        fields.extend(connection["nodes"])
        cursor = next_cursor(connection, cursors)
        if cursor is None:
            break
    matches = [f for f in fields if f.get("name") == "Status"]
    if len(matches) != 1 or matches[0].get("__typename") != "ProjectV2SingleSelectField":
        raise ApiError("Project must have exactly one existing single-select Status field")
    field = matches[0]
    options = {}
    for option in field.get("options", []):
        key = " ".join(option["name"].split()).casefold()
        if key in options or not option.get("id"):
            raise ApiError("Status options have duplicate names or missing identifiers")
        options[key] = option["id"]
    missing = {"todo", "blocked", "in progress", "done"} - options.keys()
    if missing:
        raise ApiError("Missing required Status options: " + ", ".join(sorted(missing)) + "; restore options in the existing field")
    return StatusConfig(project_id, field["id"], options["todo"], options["blocked"])


def load_items(api: GitHubApi, project_id: str) -> list[dict]:
    cursor = None
    cursors: set[str] = set()
    items = []
    seen = set()
    while True:
        project = api.graphql(ITEMS_QUERY, {"project": project_id, "cursor": cursor}).get("node")
        if not project or project.get("id") != project_id:
            raise ApiError("Cannot read Project items")
        connection = project["items"]
        for item in connection["nodes"]:
            if not item or not item.get("id") or item["id"] in seen:
                raise ApiError("Project returned a missing or duplicate item")
            seen.add(item["id"])
            items.append(item)
        cursor = next_cursor(connection, cursors)
        if cursor is None:
            return items


def desired_status(current: str | None, has_open_blockers: bool, config: StatusConfig) -> str | None:
    if current == config.todo_id and has_open_blockers:
        return config.blocked_id
    if current == config.blocked_id and not has_open_blockers:
        return config.todo_id
    return None


def synchronize(api: GitHubApi, repository: str, owner: str, number: int, dry_run: bool = False) -> Summary:
    config = load_config(api, owner, number)
    # Finish the paginated Project scan before any mutation. A failed scan is
    # incomplete, not an invitation to process a partial/invisible Project.
    items = load_items(api, config.project_id)
    summary = Summary()
    for item in items:
        summary.scanned += 1
        label = {"item": item["id"]}
        try:
            content = item.get("content")
            if not content:
                if item.get("type") == "ISSUE":
                    raise ApiError("Project issue content is inaccessible; check token repository access")
                summary.skipped += 1
                log("skip", **label, reason="draft or inaccessible non-issue item")
                continue
            if content.get("__typename") != "Issue":
                summary.skipped += 1
                log("skip", **label, reason="not an issue")
                continue
            issue_repository = (content.get("repository") or {}).get("nameWithOwner")
            if not issue_repository:
                raise ApiError("Project issue repository is inaccessible")
            label["issue"] = content["number"]
            if issue_repository.casefold() != repository.casefold():
                summary.skipped += 1
                log("skip", **label, reason="issue belongs to another repository")
                continue
            status = item.get("fieldValueByName") or {}
            current = status.get("optionId")
            if current not in (config.todo_id, config.blocked_id):
                summary.skipped += 1
                log("skip", **label, status=status.get("name", "unset"), reason="status is protected or outside Todo/Blocked")
                continue
            blockers = api.blockers(repository, content["number"])
            has_open = any(blocker["state"] == "open" for blocker in blockers)
            target = desired_status(current, has_open, config)
            if target is None:
                summary.skipped += 1
                log("skip", **label, status=status.get("name"), reason="status already agrees with native blockers")
                continue
            fresh = api.graphql(RECHECK_QUERY, {"item": item["id"]}).get("node")
            if not fresh or (fresh.get("project") or {}).get("id") != config.project_id:
                raise ApiError("Project item disappeared or changed Project before update")
            fresh_content = fresh.get("content") or {}
            if fresh_content.get("id") != content.get("id"):
                raise ApiError("Project item changed issue before update")
            fresh_status = fresh.get("fieldValueByName") or {}
            target = desired_status(fresh_status.get("optionId"), has_open, config)
            if target is None:
                summary.skipped += 1
                log("skip", **label, status=fresh_status.get("name", "unset"), reason="fresh status is protected or already correct")
                continue
            target_name = "Blocked" if target == config.blocked_id else "Todo"
            if dry_run:
                summary.would_change += 1
                log("would_change", **label, previous=fresh_status.get("name"), status=target_name)
                continue
            result = api.graphql(STATUS_MUTATION, {
                "project": config.project_id, "item": item["id"],
                "field": config.field_id, "option": target,
            })
            updated = (result.get("updateProjectV2ItemFieldValue") or {}).get("projectV2Item")
            if not updated or updated.get("id") != item["id"]:
                raise ApiError("Status mutation did not confirm the expected Project item")
            summary.changed += 1
            log("changed", **label, previous=fresh_status.get("name"), status=target_name)
        except (ApiError, KeyError, TypeError, ValueError) as exc:
            summary.failed += 1
            log("error", **label, message=redact(str(exc)))
    log("summary", dry_run=dry_run, **vars(summary))
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository", default="Krzys2002/Mini_Robot")
    parser.add_argument("--project-owner", default="Krzys2002")
    parser.add_argument("--project-number", type=int, default=4)
    parser.add_argument("--dry-run", action="store_true", help="read and log transitions without mutations")
    args = parser.parse_args(argv)
    if not re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", args.repository) or args.project_number < 1:
        parser.error("repository must be OWNER/REPO and Project number must be positive")
    try:
        return synchronize(GitHubApi(), args.repository, args.project_owner, args.project_number, args.dry_run).exit_code
    except (ApiError, KeyError, TypeError, ValueError) as exc:
        log("error", message=redact(str(exc)), reason="Project preflight or inventory failed; no statuses were changed")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
