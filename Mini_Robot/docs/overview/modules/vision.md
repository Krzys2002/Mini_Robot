# Image-space robot perception

Milestone scope: M2. [Module index](README.md).

## Capability and boundaries

Tag36h11 print profiles, mounting and ID allocation have reproducible offline
sheets/tools. Timestamped multi-tag detection runs through `IMarkerDetector`;
OpenCV conversion stays in its adapter. `VisionController` uses a serial worker
with one active and one newest pending frame, discarding old generations on
source/resolution changes and shutdown. Source timestamps and pixel coordinates
remain separate from metric world state.

`RobotMarkerMap` associates the allocated tag range with RobotIds. Robot
observations reject unknown/duplicate markers and invalid geometry, calculate
projective centers and provisional image-space headings. These measurements do
not update WorldModel; calibrated world conversion, tracking and freshness-based
control remain separate issues.

## Behavior guides

- [Selection, print tools and commissioning](../../vision/apriltag-selection.md)
- [Detection API, backend limits and bounded worker](../../vision/apriltag-detection.md)
- [Robot association and image geometry](../../vision/robot-observation.md)
- [Vision assets and guide index](../../vision/README.md)

## Verification and remaining acceptance

Deterministic tests cover all packed formats/stride, rotations, multiple IDs,
negative scenes and generated-video replay. Observation tests cover identity
ranges, duplicate/invalid rejection, projective geometry and headings. Worker
checks cover bounded buffering, invalidation, cancellation and shutdown.
Physical printed-tag acceptance remains #191; synthetic inputs do not prove
physical range, lighting, throughput or localization accuracy. Native Linux and
Windows runtime need their own executed evidence.
