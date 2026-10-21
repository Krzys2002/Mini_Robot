# Authoritative world state

Milestone scope: M1. [Module index](README.md).

## Capability and boundaries

One non-copyable WorldModel owns registered robots and timestamped world poses.
Registration creates unknown pose. Updates reject unknown IDs, non-finite values
and stale timestamps; queries/snapshots return detached values. Filtering,
prediction, tracking freshness, obstacles and automatic poses are not registry roles.

## Behavior guides

- [WorldModel contract](../world-model.md)

## Verification and remaining acceptance

World tests cover registry changes, unknown poses, finite coordinates, timestamp
ordering and snapshot ownership without GUI/hardware. The world guide owns test
selection commands and detailed acceptance.
