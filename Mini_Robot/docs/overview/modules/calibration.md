# Camera calibration

Milestone scope: M2. [Module index](README.md).

## Capability and boundaries

Chessboard capture/review, intrinsic solving, result validation and reprojection
quality reporting are implemented. Domain contracts/validation remain Qt/OpenCV-free;
replaceable OpenCV detection/solving runs on a worker. The wizard presents results
and forwards requests. Persistence and workspace calibration remain separate.

## Behavior guides

- [Target capture/review](../calibration-capture.md)
- [Intrinsic model and quality](../intrinsic-calibration.md)

## Verification and remaining acceptance

Calibration suites cover capture/diversity, result validation and analytic camera
recovery. Offscreen wizard tests cover asynchronous work, cancellation and shutdown.
The guides retain physical acceptance, quality limits and dated records.
