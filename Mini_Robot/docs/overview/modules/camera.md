# Camera acquisition

Milestone scope: M1. [Module index](README.md).

## Capability and boundaries

Discovery, selection, native start/stop and deterministic replay are implemented
behind ICameraSource. Acquisition publishes timestamped immutable packed frames;
it does not detect markers or own robot poses. DesktopController owns the active
source and stops it before replacement. Platform APIs remain in adapters.
Replay I/O belongs on its acquisition/test thread, not the GUI thread.

## Behavior guides

- [Frame/source contract](../camera-source.md)
- [Discovery](../camera-discovery.md), [selection](../camera-selection.md)
- [Streaming](../camera-streaming.md), [backends](../camera-backends.md)
- [Replay](../recorded-video.md)

## Verification and remaining acceptance

Frame/selection, replay and offscreen source/controller tests run without devices.
V4L2 tests use temporary device trees; physical capture uses the hardware label
and skips without its device variable. Backend guides own actual device/platform
evidence. USB visibility, hotplug/capture and automatic recovery need their
issue-specific checks; interface recovery support is not hardware proof.
