# Robot marker observations

Issue [#51](https://github.com/Krzys2002/Mini_Robot/issues/51) converts raw
[AprilTag detections](apriltag-detection.md) into per-robot pixel measurements,
before workspace calibration or tracking exists. Source:
[RobotObservation.hpp](../../src/vision/RobotObservation.hpp) and
[RobotObservation.cpp](../../src/vision/RobotObservation.cpp), target
`micro_robot_vision_observation` (Qt/OpenCV-free, platform-independent).

A `RobotObservation` is a measurement for one frame, not robot state. Nothing in
this stage undistorts pixels, converts to world coordinates, updates `WorldModel`,
filters, tracks or commands robots.

## Identity

`RobotMarkerMap` applies the [selection profile](apriltag-selection.md#robot-identity-and-mounting-convention):
by default `tag36h11` IDs **0–15 map to `RobotId` 1–16**. `RobotMarkerProfile`
holds the family, first dictionary ID, count and first RobotId in one place; the
constructor throws `std::invalid_argument` for an unsupported family, an empty
range, a range outside the 587-ID dictionary, or RobotIds that would overflow
`uint32`. Only `MarkerFamily::AprilTag36h11` is supported; selecting an unknown
family in a custom profile cannot enable observations from that family.

`observeRobots(detections, map)` sorts every raw marker into exactly one of:

| Outcome | Condition |
| --- | --- |
| `robots` (ordered by RobotId) | Profile family, allocated ID, exactly one copy in the frame, valid geometry |
| `UnsupportedFamily` | Family differs from the profile |
| `UnassignedId` | Dictionary ID outside the allocated range (including negative) |
| `DuplicateRobot` | Two or more markers in the frame map to the same RobotId; **all** copies are rejected, none is chosen |
| `InvalidGeometry` | Non-finite, collapsed, collinear, self-intersecting, concave or mirrored corners |

Rejected markers keep the raw observation for diagnostics, in detection order.
Unknown or ambiguous markers therefore never carry a RobotId and cannot reach
robot state. A detection error is propagated with its frame metadata and no
partial robots or rejections. An empty successful frame stays a negative observation.

## Center and provisional image-space heading

Corners keep the detector's canonical **TL, TR, BR, BL** order; image origin is
top-left with +u right and +v down.

- **Center** is the intersection of diagonals TL–BR and TR–BL. Under perspective
  this is the image of the printed square's center; the vertex mean is not.
  It moves exactly with any translation of the corners.
- **Front** is the vector from the bottom-edge midpoint (BL, BR) to the top-edge
  midpoint (TL, TR), i.e. the printed `UP ^` direction mounted toward robot front.
- **Image heading** (`Radians`) is zero along +u and increases
  **counterclockwise as displayed** (from +u toward image up, −v):
  `atan2(-front.v, front.u)`. Front toward image right, up, left and down gives
  0, π/2, π and −π/2.
- All angles are normalized with `normalizeAngle` from
  [common/Angle.hpp](../../src/common/Angle.hpp) to the half-open range
  **(−π, π]**; −π and π both become π.

The displayed-counterclockwise sign matches the world convention (zero along +X,
counterclockwise with +Y up) for an unmirrored top-down camera, but the heading is
still provisional: lens distortion, camera roll and the world transform are not
applied. Derive world heading only after calibrated image-to-world conversion.

Geometry must be finite and strictly convex with a positive (clockwise as
displayed) turn at every corner, the winding produced by an unmirrored canonical
marker. A mirrored camera feed is therefore rejected instead of reporting a
reversed rotation sense.

## Timestamp and quality metadata

Each observation copies the frame sequence and original monotonic capture
timestamp, so later freshness rules can be applied per robot. Quality fields:

- `minimumEdgePixels`: the shortest projected edge, for comparison with the
  [sampling budget](apriltag-selection.md#camera-constraints-independent-of-camera-model)
  (provisional minimum 24 px, preferred 40 px). No threshold is enforced here.
- `decisionMargin`: passed through when the detector supplies one; OpenCV's
  ArUco backend leaves it empty. No confidence score is fabricated.

## Verification

`micro_robot_vision_observation_tests` (labels `core`, `vision`) runs in every build,
including core-only builds without Qt or OpenCV. It covers the default and custom
mappings, invalid profiles, every dictionary ID, unsupported families, duplicates,
error propagation, translation, perspective centers, cardinal headings including
the ±π boundary, one-degree steps through −720°…720°, and invalid geometry.
`micro_robot_common_tests` covers `normalizeAngle`. With the OpenCV detector
enabled, `AprilTagDetectorTest.DecodedCanonicalCornersGiveRobotHeadingInEveryQuarterTurn`
checks the heading and center of a generated tag in all four quarter-turns.

Not yet verified with a physical printed tag or camera: front-edge orientation
of a mounted robot remains part of the commissioning steps in
[#191](https://github.com/Krzys2002/Mini_Robot/issues/191).
