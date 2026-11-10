# CodeCell C6 hardware evidence record — issue #154

Status: **UNFILLED — no physical acceptance established by this record.**
Copy this record for each actual board/configuration under test. Fill fields from
observations or identified sources and link retained evidence. Leave unavailable
values UNRESOLVED and unperformed checks NOT RUN; a prepared record is not a pass.
Use the [hardware profile and approved bench procedure](codecell-c6-hardware-154.md)
as the authority for sequencing and gates. This record supplies no numerical
supply, actuator or stopping limits and authorizes no powered test.

## Identity and provenance

| Field | Recorded value / evidence |
| --- | --- |
| Date, operator and reviewer | UNRESOLVED |
| Board identifier and physical revision | UNRESOLVED |
| Top/bottom photographs and boot/reset pin evidence | UNRESOLVED |
| Firmware commit, test image and build environment | UNRESOLVED |
| PlatformIO, framework/compiler and CodeCell library resolved versions | UNRESOLVED |
| Host OS, tools and serial device | UNRESOLVED |
| Hardware profile version and differences from it | UNRESOLVED |
| Raw evidence location (photos, logs, measurements) | UNRESOLVED |

## Unpowered inspection and source records

The existing profile records previously supplied component facts. Reference those
sources and confirm applicability to this physical assembly; do not relabel a
listing or source-code observation as an electrical measurement.

| Item | Actual observation / source | Result |
| --- | --- | --- |
| Board/module and motor/driver identification | UNRESOLVED | NOT RUN |
| Left/right motor wiring and channel labels | UNRESOLVED | NOT RUN |
| Wheel diameter and wheel-center track width, units and uncertainty | UNRESOLVED | NOT RUN |
| Battery model, protection components and documented limits | UNRESOLVED | NOT RUN |
| IMU mounting axes relative to robot forward/up | UNRESOLVED | NOT RUN |
| Verified wheel feedback, if any; otherwise explicit open-loop classification | UNRESOLVED | NOT RUN |

## Power and actuator gate record

Complete the evidence required by [profile section 5.1](codecell-c6-hardware-154.md#51-required-supply-and-output-evidence-before-power-is-applied)
before its powered sequence. Missing evidence leaves the corresponding gate
BLOCKED. Approval must identify the actual configuration, rationale and reviewer.

| Gate | Value, units, rationale and evidence | Reviewer / result |
| --- | --- | --- |
| Approved supply voltage and current limit for this assembly | UNRESOLVED | BLOCKED |
| Supply/limiter model, settings and independently measured limiting behavior | UNRESOLVED | BLOCKED |
| Reviewed power wiring, battery/USB/charging paths and limiter bypass prevention | UNRESOLVED | BLOCKED |
| Approved final actuator output bound for each channel, including both signs | UNRESOLVED | BLOCKED |
| Exact firmware enforcement after all mixing, heading correction and startup transformations | UNRESOLVED | BLOCKED |
| Final-write traces and electrical output evidence from the approved measurement setup | UNRESOLVED | BLOCKED |
| Restraint, wheels-raised fixture, monitoring and immediate disconnect arrangement | UNRESOLVED | BLOCKED |
| Authorized test command values/durations and applicable acceptance bounds | UNRESOLVED | BLOCKED |

## Physical results

Execute only the existing procedure whose gates have actual approval/evidence.
Do not perform powered checks merely to fill this table. Distinguish software
setpoint, actuator API write, measured electrical output and mechanical response.

| Check | Configuration, observations, units / uncertainty and evidence | Result |
| --- | --- | --- |
| Boot/reset zero-output electrical levels and wheel behavior | UNRESOLVED | NOT RUN |
| Left/right signs for forward/reverse and both rotation directions | UNRESOLVED | NOT RUN |
| Zero, E-stop latch, explicit clear and fresh-intent behavior | UNRESOLVED | NOT RUN |
| Command-expiry timing and observed stopping, against separately established bounds | UNRESOLVED | NOT RUN |
| IMU mounting/heading direction against independent reference | UNRESOLVED | NOT RUN |
| Battery API raw value, documented conversion and simultaneous multimeter reading | UNRESOLVED | NOT RUN |
| Task/setup failure observations, if included in an approved procedure | UNRESOLVED | NOT RUN |

Battery samples must retain the raw API value, conversion source, reference
voltage, condition, timestamp and measurement uncertainty. Neither this table nor
the prototype's conversion establishes the API's units or accuracy.

## Acceptance and handoff

| #154 criterion | Evidence / unresolved action | Status |
| --- | --- | --- |
| Motion-relevant assumptions have a verified source or blocking unknown | Reconcile each row with the versioned hardware profile | PENDING |
| Actual board/build identity, polarity and zero-output behavior are reconciled | Attach actual board and electrical evidence | PENDING |
| Available feedback is explicitly classified | Confirm physical feedback and IMU mounting; do not presume wheel-speed PID | PENDING |
| Supply/output/geometry configuration and bench setup are documented | All applicable power/actuator gates need evidence | BLOCKED |

Record failures, owner and follow-up issue for each unresolved item. Update the
versioned hardware profile only with actual evidence and review. The existing
[keep/adapt/disable matrix](codecell-c6-hardware-154.md#6-legacy-behavior-matrix-keepadaptdisable)
remains a profile decision subject to its stated validation gates; this empty
record does not confirm its assumptions. Physical follow-up #238 and downstream powered-test gates
remain open until the required acceptance evidence exists. Build success, host
fakes and the separate BLE connection fixture do not establish motor safety.
