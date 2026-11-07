# Authoritative world model

Source: [WorldModel.hpp](../../src/world/WorldModel.hpp),
[WorldModel.cpp](../../src/world/WorldModel.cpp),
[RobotState.hpp](../../src/world/RobotState.hpp), and
[WorldSnapshot.hpp](../../src/world/WorldSnapshot.hpp).

The composition root owns one `WorldModel` and injects references. It is neither
copyable nor movable. Access is serialized on its owning thread; no mutex,
callbacks, background work, or clock reads are supplied. The private ordered
registry stores independent state by RobotId. Camera/transport connection does
not implicitly register a robot or update its pose.

## API and update rules

| API | Result |
| --- | --- |
| `registerRobot(id, at)` | True for a new record; false for a duplicate, without resetting state. |
| `removeRobot(id)` | True only when a record was removed. |
| `updatePose(id, pose, at)` | Explicit PoseUpdateResult; rejected updates leave state unchanged. |
| `robot(id)` | Optional value copy; absent for unknown IDs. |
| `robotCount()` | Number of registered records. |
| `snapshot()` | Detached registry and optional workspace-bounds copy, ordered by RobotId. |
| `setWorkspaceBounds(optionalBounds)` | Set measured metric bounds or clear them; does not change robot poses. |

Registration records `registeredAt` with both pose fields absent. Epoch is a valid
timestamp, not a missing-value sentinel. Pose and `poseUpdatedAt` become present
together. The first update may equal registration time; later updates must be
strictly newer for that robot. Re-registration after removal starts with unknown
pose and a new registration time.

Updates are checked in this order: unknown ID returns `UnknownRobot`; a non-finite
x, y, or heading returns `InvalidPose`; time before registration or at/before the
last accepted pose returns `StaleTimestamp`; otherwise the result is `Applied`.
Heading is stored unchanged, without filtering or normalization. Coordinates use
metres and radians in the [common world frame](architecture.md#domain-conventions).

```cpp
#include "world/WorldModel.hpp"
#include <chrono>
#include <cassert>

int main()
{
    using namespace micro_robot;
    using namespace std::chrono_literals;
    WorldModel world;
    const RobotId id{1};
    assert(world.registerRobot(id, Timestamp{1s}));
    assert(world.updatePose(id, {Meters{0.25}, Meters{0.15}, Radians{0.0}},
                            Timestamp{2s}) == PoseUpdateResult::Applied);
    const auto state = world.robot(id);
    assert(state && state->pose && state->poseUpdatedAt == Timestamp{2s});
    auto snapshot = world.snapshot();
    snapshot.robots.clear();
    assert(world.robotCount() == 1);
}
```

Tracked measurements expose status and age through `tracking(id, now)`; see
[robot tracking](robot-tracking.md). Consumers must apply explicit safety rules. [Workspace bounds](workspace.md) are optional
authoritative state; they do not establish calibrated localization. Obstacle, task storage, velocity,
quality, filtering, and connection state are not yet WorldModel features. Future
issue-scoped domain storage can extend private state and the detached snapshot.

[World tests](../../tests/world/WorldModelTest.cpp) verify insert/update/query,
rejection, independent robot ordering, snapshots, and the ownership contract.
