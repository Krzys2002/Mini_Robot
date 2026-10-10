"""Deterministic tests: no GitHub credentials or live mutations required."""
import contextlib
import copy
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sync_issue_blockers as sync

REPOSITORY = "Krzys2002/Mini_Robot"
OPTIONS = {"todo": "Todo", "blocked": "Blocked", "active": "In progress", "done": "Done", "review": "Review"}


def item(number, status="todo", repository=REPOSITORY):
    return {"id": f"item-{number}", "type": "ISSUE",
            "content": {"__typename": "Issue", "id": f"issue-{number}", "number": number,
                        "repository": {"nameWithOwner": repository}},
            "fieldValueByName": {"optionId": status, "name": OPTIONS.get(status, status)}}


def connection(nodes, cursor=None):
    return {"nodes": nodes, "pageInfo": {"hasNextPage": cursor is not None, "endCursor": cursor}}


class FakeApi:
    def __init__(self, items, blockers=None, page_size=100):
        self.items = copy.deepcopy(items)
        self.dependencies = blockers or {}
        self.page_size = page_size
        self.fields = [{"__typename": "ProjectV2SingleSelectField", "id": "status-field", "name": "Status",
                        "options": [{"id": key, "name": name} for key, name in OPTIONS.items()]}]
        self.mutations = []
        self.reads = []
        self.calls = []
        self.fresh_status = {}
        self.fail_mutations = set()
        self.fail_item_cursor = None

    def graphql(self, query, variables):
        self.calls.append((query, variables))
        if query == sync.FIELDS_QUERY:
            start = int(variables["cursor"] or 0)
            end = start + self.page_size
            return {"user": {"projectV2": {"id": "project", "fields": connection(
                self.fields[start:end], str(end) if end < len(self.fields) else None)}}}
        if query == sync.ITEMS_QUERY:
            if variables["cursor"] and variables["cursor"] == self.fail_item_cursor:
                raise sync.ApiError("inventory failed")
            start = int(variables["cursor"] or 0)
            end = start + self.page_size
            return {"node": {"id": "project", "items": connection(
                copy.deepcopy(self.items[start:end]), str(end) if end < len(self.items) else None)}}
        selected = next(x for x in self.items if x["id"] == variables["item"])
        if query == sync.RECHECK_QUERY:
            result = copy.deepcopy(selected)
            result["project"] = {"id": "project"}
            if selected["id"] in self.fresh_status:
                status = self.fresh_status[selected["id"]]
                result["fieldValueByName"] = {"optionId": status, "name": OPTIONS[status]}
            return {"node": result}
        if query == sync.STATUS_MUTATION:
            if selected["id"] in self.fail_mutations:
                raise sync.ApiError("write denied")
            self.mutations.append(variables)
            selected["fieldValueByName"] = {"optionId": variables["option"], "name": OPTIONS[variables["option"]]}
            return {"updateProjectV2ItemFieldValue": {"projectV2Item": {"id": selected["id"]}}}
        raise AssertionError("Unexpected GraphQL operation")

    def blockers(self, repository, number):
        self.reads.append((repository, number))
        value = self.dependencies.get(number, [])
        if isinstance(value, Exception):
            raise value
        return [{"state": state} for state in value]


class SynchronizationTests(unittest.TestCase):
    def run_sync(self, api, dry_run=False):
        with contextlib.redirect_stdout(io.StringIO()) as output:
            result = sync.synchronize(api, REPOSITORY, "Krzys2002", 4, dry_run)
        self.logs = [json.loads(line) for line in output.getvalue().splitlines()]
        return result

    def test_open_blocker_sets_blocked_and_repeat_is_idempotent(self):
        api = FakeApi([item(2)], {2: ["open"]})
        self.assertEqual(self.run_sync(api).changed, 1)
        self.assertEqual(api.items[0]["fieldValueByName"]["optionId"], "blocked")
        self.assertEqual(self.run_sync(api).changed, 0)
        self.assertEqual(len(api.mutations), 1)
        self.assertEqual(api.mutations[0], {"project": "project", "item": "item-2", "field": "status-field", "option": "blocked"})

    def test_closed_blocker_restores_todo(self):
        api = FakeApi([item(2, "blocked")], {2: ["closed"]})
        self.assertEqual(self.run_sync(api).changed, 1)
        self.assertEqual(api.mutations[0]["option"], "todo")

    def test_any_open_blocker_keeps_blocked(self):
        api = FakeApi([item(2, "blocked")], {2: ["closed", "open"]})
        self.assertEqual(self.run_sync(api).changed, 0)
        self.assertFalse(api.mutations)

    def test_protected_and_other_statuses_are_skipped_without_dependency_reads(self):
        for status in ("active", "done", "review", None):
            with self.subTest(status=status):
                api = FakeApi([item(2, status)], {2: ["open"]})
                self.assertEqual(self.run_sync(api).skipped, 1)
                self.assertFalse(api.mutations)
                self.assertFalse(api.reads)

    def test_todo_without_blockers_stays_todo(self):
        api = FakeApi([item(2)])
        self.assertEqual(self.run_sync(api).changed, 0)
        self.assertFalse(api.mutations)

    def test_added_then_removed_blocker_is_detected_on_next_scan(self):
        api = FakeApi([item(2)])
        self.assertEqual(self.run_sync(api).changed, 0)
        api.dependencies[2] = ["open"]
        self.assertEqual(self.run_sync(api).changed, 1)
        api.dependencies[2] = []
        self.assertEqual(self.run_sync(api).changed, 1)
        self.assertEqual(api.items[0]["fieldValueByName"]["optionId"], "todo")

    def test_dry_run_has_no_mutations(self):
        api = FakeApi([item(2)], {2: ["open"]})
        result = self.run_sync(api, True)
        self.assertEqual((result.would_change, result.changed, result.exit_code), (1, 0, 0))
        self.assertFalse(api.mutations)

    def test_fresh_status_preserves_newly_started_work(self):
        for status in ("active", "done", "blocked"):
            with self.subTest(status=status):
                api = FakeApi([item(2)], {2: ["open"]})
                api.fresh_status["item-2"] = status
                self.assertEqual(self.run_sync(api).changed, 0)
                self.assertFalse(api.mutations)

    def test_individual_read_or_write_failure_continues_and_returns_failure(self):
        for failure in ("read", "write"):
            with self.subTest(failure=failure):
                api = FakeApi([item(2), item(3)], {2: ["open"], 3: ["open"]})
                if failure == "read":
                    api.dependencies[2] = sync.ApiError("read denied")
                else:
                    api.fail_mutations.add("item-2")
                result = self.run_sync(api)
                self.assertEqual((result.failed, result.changed, result.exit_code), (1, 1, 1))
                self.assertEqual(api.mutations[0]["item"], "item-3")

    def test_other_repository_pull_request_and_draft_are_skipped(self):
        foreign = item(2, repository="someone/another")
        pull_request = item(3)
        pull_request["content"]["__typename"] = "PullRequest"
        draft = {"id": "draft", "type": "DRAFT_ISSUE", "content": None}
        api = FakeApi([foreign, pull_request, draft])
        self.assertEqual(self.run_sync(api).skipped, 3)
        self.assertFalse(api.reads)
        self.assertFalse(api.mutations)

    def test_inaccessible_issue_is_failure(self):
        api = FakeApi([{"id": "hidden", "type": "ISSUE", "content": None}])
        self.assertEqual(self.run_sync(api).exit_code, 1)

    def test_fields_and_items_paginate(self):
        api = FakeApi([item(2), item(3)], {2: ["open"], 3: ["open"]}, page_size=1)
        api.fields.insert(0, {"__typename": "ProjectV2Field", "id": "title", "name": "Title"})
        self.assertEqual(self.run_sync(api).changed, 2)
        self.assertTrue(any(query == sync.FIELDS_QUERY and variables["cursor"] == "1" for query, variables in api.calls))
        self.assertTrue(any(query == sync.ITEMS_QUERY and variables["cursor"] == "1" for query, variables in api.calls))

    def test_failed_inventory_does_not_partially_update(self):
        api = FakeApi([item(2), item(3)], {2: ["open"]}, page_size=1)
        api.fail_item_cursor = "1"
        with self.assertRaises(sync.ApiError):
            self.run_sync(api)
        self.assertFalse(api.reads)
        self.assertFalse(api.mutations)

    def test_missing_field_or_option_fails_without_writes(self):
        for configuration in ("field", "option"):
            with self.subTest(configuration=configuration):
                api = FakeApi([item(2)], {2: ["open"]})
                if configuration == "field":
                    api.fields = []
                else:
                    api.fields[0]["options"] = [o for o in api.fields[0]["options"] if o["id"] != "blocked"]
                with self.assertRaises(sync.ApiError):
                    self.run_sync(api)
                self.assertFalse(api.mutations)

    def test_repeated_cursor_fails(self):
        with self.assertRaises(sync.ApiError):
            sync.next_cursor(connection([], "same"), {"same"})


class ApiTests(unittest.TestCase):
    def test_rest_pagination_checks_open_blocker_on_later_page(self):
        api = sync.GitHubApi()
        with patch.object(api, "request", side_effect=[[{"state": "closed"}] * 100, [{"state": "open"}]]) as request:
            blockers = api.blockers(REPOSITORY, 2)
        self.assertEqual(len(blockers), 101)
        self.assertTrue(any(b["state"] == "open" for b in blockers))
        self.assertIn("page=2", request.call_args_list[1].args[0])

    def test_invalid_or_incomplete_blocker_state_fails_safe(self):
        api = sync.GitHubApi()
        for response in ([{}], [{"state": "unknown"}], {"message": "denied"}):
            with patch.object(api, "request", return_value=response), self.assertRaises(sync.ApiError):
                api.blockers(REPOSITORY, 2)

    def test_graphql_partial_error_is_not_accepted(self):
        api = sync.GitHubApi()
        with patch.object(api, "request", return_value={"data": {}, "errors": [{"message": "denied"}]}), self.assertRaises(sync.ApiError):
            api.graphql("query", {})

    def test_mutation_payload_passed_on_stdin(self):
        response = subprocess.CompletedProcess([], 0, '{"data":{}}', "")
        with patch.object(sync.subprocess, "run", return_value=response) as run:
            sync.GitHubApi().graphql("mutation", {"option": "blocked"})
        self.assertEqual(json.loads(run.call_args.kwargs["input"])["variables"], {"option": "blocked"})
        self.assertIn("--input", run.call_args.args[0])
        self.assertFalse(run.call_args.kwargs.get("shell", False))

    def test_api_failure_redacts_credentials(self):
        with patch.dict(os.environ, {"GH_TOKEN": "test-secret"}), patch.object(sync.subprocess, "run", return_value=subprocess.CompletedProcess([], 1, "", "denied test-secret github_pat_sensitive")):
            with self.assertRaises(sync.ApiError) as caught:
                sync.GitHubApi().request("graphql")
        self.assertNotIn("test-secret", str(caught.exception))
        self.assertNotIn("github_pat_sensitive", str(caught.exception))

    def test_main_returns_nonzero_on_global_failure(self):
        with patch.object(sync, "synchronize", side_effect=sync.ApiError("denied")), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(sync.main(["--dry-run"]), 1)


if __name__ == "__main__":
    unittest.main()
