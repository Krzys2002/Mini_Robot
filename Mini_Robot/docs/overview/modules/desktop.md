# Desktop composition and presentation

Milestone scope: M1. [Module index](README.md).

## Capability and boundaries

Composition injects services into Qt Core controllers; widgets present state and
emit requests. Native camera selection/streaming and calibration capture/solving
are wired into the desktop. Transport defaults to simulation; the world view has
an explicit static demo pose. Frames/telemetry do not independently assign poses.
Calibration uses a worker; shutdown drains active services before destruction.

## Behavior guides

- [Composition and controls](../desktop-demo.md)
- [Selection](../camera-selection.md), [wizard](../calibration-capture.md)

## Verification and remaining acceptance

Offscreen tests check integration, selection, lifecycle, calibration and teardown.
Production startup uses a private config and --exit-after-startup. Device/display
checks are in the [hardware guide](../../development/hardware-tests.md). Motion
controls, safety UI, overlays and broader GUI acceptance remain issue-scoped.

Automated [GUI compatibility](../../development/gui-compatibility.md) checks cover
resizing, mouse/keyboard input and scale factors 1.5/2. The guide retains executed
Wayland and X11 (Xwayland) results and the limits of synthetic input. Physical
input through the compositor remains a separate manual check.
