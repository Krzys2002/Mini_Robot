# Camera calibration

Milestone scope: M2. [Architecture](../architecture.md).

## Capability and boundaries

Chessboard capture/review, intrinsic solving, result validation and reprojection
quality reporting are implemented. Versioned intrinsic profiles persist per exact
camera identity and delivered resolution; incompatible models are rejected.
The calibration dialog supports explicit save, automatic reload and optional
undistorted verification preview. Domain types/validation stay Qt/OpenCV-free;
filesystem I/O and OpenCV remapping run through injected adapters on the existing
worker. Captures retain raw pixels. Workspace calibration remains separate.

## Behavior guides

- [Target capture/review](../calibration-capture.md)
- [Intrinsic model and quality](../intrinsic-calibration.md)
- [Profiles, compatibility and undistorted preview](../calibration-profiles.md)

## Verification and remaining acceptance

Calibration suites cover capture/diversity, analytic model recovery, profile
round trips/validation, camera-resolution isolation, atomic replacement and
Brown–Conrady remap geometry. Offscreen dialog tests cover persistence across
controller recreation, asynchronous file/remap work, raw capture preservation,
bounded buffering, cancellation, invalidation and shutdown. Physical optical
accuracy and native-platform interaction remain manual checks in the guides.
