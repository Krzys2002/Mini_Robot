# Documentation tooling

The [maintenance guide](../../docs/development/documentation.md) describes ownership,
generation and CI commands. `check_links.py` uses only Python's standard library
and never fetches external links. Tests cover missing files/anchors, encoded paths,
code examples, conflict markers and configuration-dependent CMake inventories.

The inventory exporter is
[MicroRobotDocumentation.cmake](../../cmake/MicroRobotDocumentation.cmake), which
reads CMake target properties instead of parsing CMake source text. Generated
reports live only in the build directory.
