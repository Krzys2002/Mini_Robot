# Source and documentation map

[Module pages](modules/README.md) connect behavior guides to system areas.
Source ownership and target dependencies come from the configured CMake graph,
including interface targets, header file sets and enabled test targets. There is
no committed generated snapshot for each feature PR to edit.

## Generate the inventory

From the CMake project directory:

```sh
cmake --preset core -DMICRO_ROBOT_BUILD_DOCUMENTATION=ON
cmake --build --preset core --target documentation
```

Open `build/core/docs/source-map.md` locally. To include desktop/replay targets,
configure the usual desktop build with `MICRO_ROBOT_BUILD_DOCUMENTATION=ON`
and build its `documentation` target. Application compilation is not required
just to generate/check the inventory. Python 3.9+ is required only when enabled.
For production targets only, configure with `BUILD_TESTING=OFF`; this also avoids
GoogleTest setup.

The report is created at configure time and reflects **that configuration only**:
a core report intentionally omits Qt/OpenCV adapters. Reconfigure after changing
CMake declarations/options. Generated files stay in the ignored build directory.

## Target boundaries

The report lists target type, declaration directory, direct link/interface
dependencies and declared sources. Generator expressions remain declarations,
not an evaluated transitive graph. Generated/external entries are labelled;
source links point into the checkout. Files not declared in CMake are not claimed
as target inputs. The [platform-boundary guide](platform-boundaries.md) owns
enforcement. A target's presence does not establish hardware/platform validation.

## Implementation limits

Module pages record capability, remaining acceptance and behavior guides. The
[roadmap](../planning/MILESTONES.md) defines planned scope. Avoid duplicating
those status lists in the inventory.

## Documentation maintenance

Update CMake when sources/dependencies change and regenerate. Update the module
and behavior guide when behavior changes. Do not edit/commit generated reports.
See the [maintenance workflow](../development/documentation.md).
