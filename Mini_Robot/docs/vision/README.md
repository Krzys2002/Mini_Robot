# Vision

- [AprilTag selection and printing](apriltag-selection.md): family comparison,
  physical dimensions, robot identity, camera sampling constraints, reproducible
  marker generation, and outstanding physical checks for Issue #49.
- Printable A4 sheets: [40 mm footprint](markers/tag36h11-40mm-a4.svg) and
  [20 mm footprint](markers/tag36h11-20mm-a4.svg), each containing IDs 0–15.

- [AprilTag frame detection](apriltag-detection.md): timestamped multi-tag observations,
  OpenCV adapter, bounded worker, replay tests and remaining physical acceptance.

- [Robot marker observations](robot-observation.md): tag-to-RobotId mapping,
  unknown/duplicate rejection, projective center, provisional image-space heading
  and angle normalization, with timestamp and quality metadata (#51).

- [Physical printed-tag validation #191](https://github.com/Krzys2002/Mini_Robot/issues/191):
  outstanding live-camera and real-recording acceptance for #50.

Selection/print tooling is separate from runtime detection. Image-to-world pose
conversion and tracking remain future work.
