# Vision

- [AprilTag selection and printing](apriltag-selection.md): family comparison,
  physical dimensions, robot identity, camera sampling constraints, reproducible
  marker generation, and outstanding physical checks for Issue #49.
- Printable A4 sheets: [40 mm footprint](markers/tag36h11-40mm-a4.svg) and
  [20 mm footprint](markers/tag36h11-20mm-a4.svg), each containing IDs 0–15.

- [AprilTag frame detection](apriltag-detection.md): timestamped multi-tag observations,
  OpenCV adapter, bounded worker, replay tests and remaining physical acceptance.

Selection/print tooling is separate from runtime detection. Image-to-world pose
conversion, robot association and tracking remain future work.
