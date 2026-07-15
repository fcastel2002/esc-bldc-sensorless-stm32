# Simulation Validation Runs

The GUI validates an already-executed discrete Simulink experiment against the
STM32 speed controller. It does not launch MATLAB or allow the MCU to control

## Import contract

Import a MAT v7 file created by `export_hil_validation_vector`. The GUI only
accepts the flat `esc_validation_v1` structure documented in
[`PIL_SIMULINK.md`](PIL_SIMULINK.md). `expected_pwm` and the MCU
`pwm_command` are logical PWM counts, so they are compared without percentage
conversion.

For `sim_motor`, use `export_sim_motor_validation`. Its `kp`, `ki`, and `kd`
options command both the MCU and the model RPM PI and default to
`0.28/1.00/0`. Both controllers operate on RPM error and canonical PWM counts
referenced to ARR=2000. Simulink divides canonical counts by 2000 before the
plant duty input. Pole pairs must match all electrical components in
`sim_motor` (currently two). Gain values use protocol resolution 0.01; KD must
remain zero for algorithm 2.

Before simulation, the exporter reads
`GET http://localhost:5187/api/bridge/validation-reference`. The GUI and an ESC
with compatible firmware must be connected. The response supplies PWM
frequency, actual PWM ARR, speed-timer frequency, period limits, controller
`dt`, minimum PWM, and algorithm version. Export fails instead of substituting
defaults when that live structural reference is unavailable.
The exporter also applies the live `minimumPwm/pwmArr` ratio to the plant duty
saturation, so the simulated enabled trajectory cannot use a duty below the
ESC closed-loop minimum.

New models should use `export_simulink_validation_run`, which runs the named
model for a requested stop time and extracts only explicitly declared `logsout`
signals. Its optional `experimentSignals` payload remains part of the stored
MAT artifact for later analysis.

Each import creates a persistent run under `%LOCALAPPDATA%\EscGui\validation-runs`:

- `runs.db` stores run metadata, configuration, metrics, and sample results.
- `artifacts/<run-id>/` stores the immutable original MAT file.

## Replay

The bridge captures the active controller configuration and verifies that the
ESC structural reference still matches the imported MAT. A mismatch aborts the
run without changing structural settings. It applies only KP, KI, KD, pole
pairs, and setpoint from the simulation in RAM and never sends `SAVE_CONFIG` or
changes PWM frequency. It then starts HIL with the run's configurable MCU input
timeout and replays samples within the manifest simulation horizon one at a
time.

A sample is accepted only when `HIL_GET_OUTPUTS` reports the same `run_id` and
`source_sequence`, with a newer `output_generation`. Arrival time is used only
to measure latency. The bridge records `Passed`, `OutOfTolerance`, `Timeout`,
`MismatchedOutput`, or `TransportError` for every sample and continues after
individual failures.

Expected PWM is recalculated during replay. The reference controller advances
according to `accepted_generation` and `output_generation`, rather than
assuming a fixed number of PI executions per imported sample.

Runs with warnings complete normally. A run becomes `Failed` only when its
count of non-warm-up timeouts exceeds the configured limit. Cancelling or an
unexpected bridge failure produces `Aborted` or `Failed`; cleanup always tries
to send `HIL_STOP` and restore the captured gains, pole pairs, setpoint, and
bridge mode. PWM frequency and structural timing are never modified by the
validation. A restoration failure is persisted with the run.

## GUI and API

- `/runs`: import a MAT, set tolerance/deadlines, list, and delete past runs.
- `/runs/{id}`: execute, inspect, or delete a persisted run.
- `GET /api/validation/runs`
- `GET /api/validation/runs/{id}`
- `POST /api/validation/import`
- `POST /api/validation/runs/{id}/execute`
- `DELETE /api/validation/runs/{id}`
- `GET /api/bridge/validation-reference`

Deleting a run removes its sample rows, metadata, and immutable MAT artifact.
Running validations cannot be deleted.

The bridge remains the only owner of HID. The legacy `/pil` page remains a
live transport diagnostic and is not the source of persistent validation
evidence.
