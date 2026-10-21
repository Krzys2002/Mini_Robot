# Deterministic simulation and fakes

Milestone scope: M1. [Module index](README.md).

## Capability and boundaries

Fake camera/source-factory and simulation transport implement domain boundaries.
Tests/composition advance supplied monotonic time explicitly. Pending operations
are deferred, buffering is bounded and freshness is checked at dispatch. These
backends do not integrate real motion, implement a physical watchdog or establish
collision-safe swarm behavior.

## Behavior guides

- [Simulation and virtual time](../simulation.md)

## Verification and remaining acceptance

Simulation, substitution and shared contract tests are deterministic and
hardware-free. Physical camera/radio and control acceptance remain with adapters
and experiments.
