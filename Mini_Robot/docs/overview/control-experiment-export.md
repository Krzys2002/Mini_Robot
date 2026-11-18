# Control experiment JSON Lines export

Issue #79 records each explicit live visual-control run as one JSON Lines file.
The default is the per-user application data directory in `control-runs`; set
`logging.control_run_directory` to choose another destination. The live-control
status shows the path only after the worker opens the file successfully. File
creation and writes run on ControlTraceWriter's worker, never in the control
policy or cycle callback.

Each UTF-8 line is one JSON object. Schema version 1 defines three record types:

1. run_start: the run ID, robot/source/frame, steady-clock start time, target,
   units, controller name and the effective control and goal-controller settings.
2. sample: one completed control cycle or explicit stop attempt, with the
   measured and target pose snapshots captured by ControlLoop, tracking age,
   original camera timestamp/frame sequence, pose errors, requested and bounded
   command, submission outcomes, operational state and timing.
3. run_end: the stop/inhibition reason, final operational state, number of
   samples accepted by the queue, samples dropped under queue pressure, and a
   queue_complete flag.

## Version 1 fields

run_start contains the required identity and configuration fields:

| Field | Type / unit | Meaning |
| --- | --- | --- |
| type, schema_version, run_id | string, integer, string | Record discriminator, version 1, and unique run/file stem |
| robot_id, source, frame_id | integer, string, string | Selected robot, camera source and metric workspace frame |
| started_monotonic_ns, clock | decimal string, string | Run start time and clock domain |
| coordinate_units, controller | string, string | Unit declaration and goal-point policy name |
| target_x_m, target_y_m, target_heading_rad | number | Applied metric target pose |
| control_period_ms, command_validity_ms | integer | Scheduler period and command freshness window |
| maximum_observation_age_ns | decimal string | Strict upper bound for accepted pose age |
| maximum_linear_m_s, maximum_angular_rad_s | number | Independent configured command limits |
| position_gain_per_s, bearing_gain_per_s, final_heading_gain_per_s | number | Effective controller gains |
| position_tolerance_m, heading_tolerance_rad, turn_in_place_rad | number | Arrival and rotate-first policy thresholds |
| align_final_heading | boolean | Whether arrival includes final heading alignment |

Each sample contains type, schema_version, run_id, sample_index, robot_id,
source, frame_id, operational_state, result, command_type and stop_command.
Frame identity and operational state are snapshots captured with the control
cycle, rather than values read later by the exporter. `skipped_periods` records
timer periods omitted after lateness. When available, raw tracker input is
recorded as `raw_pose_x_m`, `raw_pose_y_m` and `raw_pose_heading_rad`, alongside
filtered pose; `measurement_quality` and age-weighted `tracking_confidence` are
separate fields and are not probabilities.
It records scheduled_time_ns, control_time_ns, submission_time_ns, lateness_ns,
camera_time_ns, tracking_age_ns, and frame_sequence. Timing, age and sequence
integers are decimal strings; unavailable camera time, age, frame sequence,
domain sequences, and submission results are JSON null.

Pose fields are pose_x_m, pose_y_m, pose_heading_rad; target fields are
target_x_m, target_y_m, target_heading_rad. Error fields are error_x_m,
error_y_m, error_heading_rad and error_distance_m. The requested control is
requested_v_m_s/requested_omega_rad_s; bounded values actually passed to
RobotManager are v_m_s/omega_rad_s. command_submission and stop_submission
describe manager handoff results. domain_sequence and stop_sequence are
application IDs; wire_sequence is always null in this desktop export.

Operational states are READY, RUNNING, PAUSED, LOST_TRACKING, LOST_CONNECTION,
ESTOP and FAULT. Cycle results use stable strings: submitted, stopped,
invalid_clock, tracking_unavailable, session_unavailable, frame_changed,
controller_unavailable, controller_failure, invalid_output and
submission_rejected. Submission values are submitted, unknown_robot,
not_connected, shutting_down, emergency_stopped, safety_fault and stale_session.
The run-end reason identifies the actual termination, including operator stop,
arrival, manual takeover/stop, camera change/stop, disconnect, scan, robot
selection, workspace change, invalid measurement, tracking loss, E-stop, safety
fault, clock reversal, controller failure/output, stale session, shutdown and
scheduler start failure.
camera_timestamp_source states that Frame.capturedAt is used and exposure time
is unverified.

run_end contains type, schema_version, run_id, ended_monotonic_ns, final_state,
reason, samples_queued, dropped_samples and queue_complete. Count and timestamp
values are decimal strings. queue_complete is false when any sample enqueue was
rejected; the two reserved delimiter slots ensure queue pressure does not omit
the footer.

## Timestamp and value conventions

All physical values use metres, metres per second, radians and radians per
second. Integer time values are decimal strings ending in _ns, measured from the
process's std::chrono::steady_clock epoch. They are suitable for within-process
latency/error analysis and retain integer precision. They cannot be compared
directly with a remote robot clock or another application's clock.

camera_time_ns is the original Frame.capturedAt measurement time. The export
labels this source explicitly; exposure time and upstream buffering are
unverified. frame_sequence is present only when the accepted metric metadata
matches the pose used by the control cycle. wire_sequence remains null because
BLE assigns that identifier inside the adapter. Domain command and Stop sequence
fields correlate application submissions only.

Pose and error fields use the active metric workspace frame. Error X/Y are target
minus measured coordinates, heading error is the shortest signed radian
difference, and distance is Euclidean position error. Missing values are null
when a source value is unavailable; optional pose/target/error components may be
omitted when the cycle did not have a finite snapshot.

## Configuration metadata

run_start stores the effective values passed to the goal-point controller and
control loop: target, update period, command validity, maximum observation age,
linear/angular bounds, position/bearing/final-heading gains, arrival tolerances,
turn-in-place threshold and final-heading policy. This captures software
settings, not physical actuator calibration or an approved operating envelope.

## Completeness and failure handling

The writer uses a bounded 256-item handoff and limits each record to 8,192 bytes.
Two queue slots are reserved for run start/end controls. When sample enqueue
fails, the producer drops that sample without waiting for disk I/O. run_end then
reports dropped_samples and sets queue_complete false. A true queue_complete
means no queue-pressure drops were observed; it is not a claim that hardware
accepted commands or that a file survived a process or disk failure. The worker
logs file-open/write errors to the ordinary application log and exposes
asynchronous open/write failure in the live-control status; a failed destination
is not shown as an available export path. Verify the file exists, contains
run_start and run_end, and has queue_complete true before using it as a complete
run.

The writer flushes each JSON line and drains accepted queued records at normal
application shutdown. An abrupt process termination can still leave a run
without run_end. A run may also be absent if the export directory is unwritable;
the live-control status reports both queue rejection and worker-side file
failure. Control safety gates remain independent of export success.

The JSONL file is analysis output, not physical acceptance evidence. Physical
approach, stopping, calibration and safety checks remain subject to #80 and the
existing #167/#154/#166/#228 hardware gates.
