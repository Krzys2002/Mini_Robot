# Image-space robot perception

Milestone scope: M2. [Module index](README.md).

## Capability and boundaries

Tag36h11, print profiles, mounting and ID allocation are specified, with
reproducible offline tools/sheets. These tools do not implement live perception.
Timestamped detector integration is separate scope (#50). Association, calibrated pose conversion and tracking remain
separate. Raw pixels are not control coordinates.

## Behavior guides

- [Selection and commissioning](../../vision/apriltag-selection.md)
- [Vision/print assets](../../vision/README.md)

## Verification and remaining acceptance

Opt-in offline tests cover IDs, rotations, margins and sheet regeneration.
Physical print/detection range, lighting, throughput and localization accuracy
need commissioning evidence. Synthetic decoding does not prove a physical envelope.
