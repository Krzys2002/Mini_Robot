import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


PROJECT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("check_links", PROJECT / "tools/docs/check_links.py")
links = importlib.util.module_from_spec(spec)
spec.loader.exec_module(links)


class LinkTests(unittest.TestCase):
    def test_github_headings_duplicates_unicode_and_explicit_anchors(self):
        text = """# Hello, `API`!
## Hello, API!
## Hello-api-1
## Zażółć gęślą
<a id="manual"></a>
<span class="note" id = "details"></span>
<span data-id="not-an-anchor"></span>
```
# Hidden
```
"""
        self.assertEqual(links.anchors(text), {"hello-api", "hello-api-1", "hello-api-1-1", "zażółć-gęślą", "manual", "details"})

    def test_inline_reference_images_titles_and_fenced_examples(self):
        text = """[one](file%20name.md#hello)
[two](nested(a).md "title")
![image](<image with spaces.png>)
[reference]: other.md#section "title"
`[literal](missing.md)`
~~~md
[example](missing.md)
~~~
"""
        self.assertEqual(list(links.destinations(text)), [
            (1, "file%20name.md#hello"), (2, "nested(a).md"),
            (3, "image with spaces.png"), (4, "other.md#section")])

    def test_missing_files_and_anchors_are_failures_and_external_links_are_offline(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            guide = root / "guide.md"
            target = root / "file name.md"
            target.write_text("# API\n", encoding="utf-8")
            guide.write_text("[ok](file%20name.md#api)\n[web](https://invalid.example/a)\n[missing](lost.md)\n[anchor](file%20name.md#lost)\n", encoding="utf-8")
            errors = links.check(root, [guide])
            self.assertEqual(len(errors), 2)
            self.assertIn("missing local target", errors[0])
            self.assertIn("missing Markdown anchor", errors[1])
            # Check the command's exit status too: CI must fail on a broken link.
            cli = subprocess.run([sys.executable, str(PROJECT / "tools/docs/check_links.py"),
                                  "--root", str(root), "--inventory", str(guide)],
                                 text=True, capture_output=True)
            self.assertEqual(cli.returncode, 1, cli.stdout + cli.stderr)
            empty = subprocess.run([sys.executable, str(PROJECT / "tools/docs/check_links.py"),
                                    "--root", str(root)], text=True, capture_output=True)
            self.assertEqual(empty.returncode, 2, empty.stdout + empty.stderr)
            guide.write_text("[ok](file%20name.md#api)\n", encoding="utf-8")
            self.assertEqual(links.check(root, [guide]), [])

    def test_conflict_markers_fail_but_history_is_not_scanned_as_current_guidance(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "docs/overview").mkdir(parents=True)
            (root / "docs/archive").mkdir()
            guide = root / "docs/overview/guide.md"
            guide.write_text("<<<<<<< HEAD\n=======\n>>>>>>> base\n", encoding="utf-8")
            (root / "docs/archive/history.md").write_text("[old](missing.md)\n", encoding="utf-8")
            files = links.maintained_files(root)
            self.assertEqual(files, [guide.resolve()])
            self.assertEqual(len(links.check(root, files)), 2)
            guide.write_text("Heading\n=======\n\n[valid](#heading)\n", encoding="utf-8")
            self.assertEqual(links.check(root, files), [])


class InventoryTests(unittest.TestCase):
    def test_cmake_graph_includes_interfaces_sources_links_and_configuration_changes(self):
        with tempfile.TemporaryDirectory(prefix="robot docs ") as directory:
            root = Path(directory) / "project"
            build = Path(directory) / "build"
            (root / "src").mkdir(parents=True)
            (root / "vendor").mkdir()
            for name in ("api (ü) #.hpp", "set.hpp", "expr.hpp", "optional.hpp"):
                (root / "src" / name).write_text("#pragma once\n", encoding="utf-8")
            (root / "src/library.cpp").write_text("int value() { return 1; }\n", encoding="utf-8")
            module = (PROJECT / "cmake/MicroRobotDocumentation.cmake").as_posix()
            (root / "CMakeLists.txt").write_text(f"""cmake_minimum_required(VERSION 3.24)
project(InventoryFixture LANGUAGES CXX)
option(MICRO_ROBOT_OPTIONAL "Fixture feature" OFF)
add_subdirectory(src)
add_subdirectory(vendor)
include("{module}")
micro_robot_add_documentation()
""", encoding="utf-8")
            (root / "vendor/CMakeLists.txt").write_text("add_library(vendor_dependency INTERFACE)\n", encoding="utf-8")
            (root / "src/CMakeLists.txt").write_text("""add_library(micro_robot_api INTERFACE)
target_sources(micro_robot_api INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/api (ü) #.hpp")
target_sources(micro_robot_api INTERFACE FILE_SET HEADERS FILES set.hpp)
target_sources(micro_robot_api INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/expr.hpp>")
add_library(micro_robot_library STATIC library.cpp)
target_link_libraries(micro_robot_library PRIVATE micro_robot_api)
if(MICRO_ROBOT_OPTIONAL)
    add_library(micro_robot_optional INTERFACE)
    target_sources(micro_robot_optional INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/optional.hpp")
endif()
""", encoding="utf-8")

            def configure(*extra):
                result = subprocess.run(["cmake", "-S", str(root), "-B", str(build), *extra],
                                        text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                return (build / "docs/source-map.md").read_text(encoding="utf-8")

            report = configure()
            self.assertIn("### micro_robot_api", report)
            self.assertIn("### micro_robot_library", report)
            self.assertIn("`INTERFACE_LIBRARY`", report)
            self.assertIn("LINK_LIBRARIES:\n\n- `micro_robot_api`", report)
            self.assertIn("src/set.hpp", report)
            self.assertIn("api%20%28ü%29%20%23.hpp", report)
            self.assertIn("$<BUILD_INTERFACE:@source/src/expr.hpp>", report)
            self.assertNotIn(str(root), report)
            self.assertNotIn("### vendor_dependency", report)
            self.assertNotIn("### micro_robot_optional", report)
            self.assertEqual(links.check(root, [build / "docs/source-map.md"]), [])
            self.assertEqual(configure(), report)
            # A newly enabled target appears from CMake alone, with no source-map edit.
            optional = configure("-DMICRO_ROBOT_OPTIONAL=ON")
            self.assertIn("### micro_robot_optional", optional)
            self.assertIn("src/optional.hpp", optional)


if __name__ == "__main__":
    unittest.main()
