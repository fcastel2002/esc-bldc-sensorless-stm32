# Simulation Validation Runs

The GUI validates an already-executed discrete Simulink experiment against the
STM32 speed controller. It does not launch MATLAB or allow the MCU to control

## Import contract

Import a MAT v7 file created by `export_hil_validation_vector`. The GUI only
uses the flat `esc_validation_v2` structure documented in
[`PIL_SIMULINK.md`](PIL_SIMULINK.md). `expected_pwm` and the MCU
`pwm_command` are logical PWM counts, so they are compared without percentage
conversion.

For deterministic MAT v2, every sample must have `enable=true`, `target_rpm`
must be constant and equal to the manifest target, and `samplePeriodUs` must be
exactly divisible by the controller `dt` in whole microseconds. Timestamps mark
interval starts. Therefore rows run from `0` through
`stopTime - samplePeriod`, with no row at `stopTimeSeconds`; each
`expected_pwm` is the expected output after the exact
`N = samplePeriodUs / dtUs` controller ticks for that row.
Legacy `esc_validation_v1` artifacts remain importable for inspection, but
cannot be executed by the deterministic replay engine because their rows use
the previous endpoint timing and expected-output semantics.

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
timeout and stepped execution mode. The preflight requires deterministic HIL
capability bit 0, HIL step operation version 1, and a firmware maximum that can
accept `N = samplePeriodUs / controllerDtUs` ticks in one command.

The stepped session keeps TIM4 stopped. For each sample the bridge sends one
exact 18-byte `HIL_STEP` request, and the MCU applies the input, executes exactly
`N` PI ticks synchronously, and returns an exact 48-byte response. Acceptance
requires matching accepted/applied `run_id` and `source_sequence`, requested and
applied counts equal to `N`, and
`output_generation - accepted_generation == N` without overflow. No
`HIL_GET_OUTPUTS` polling is used to wait for completion.

Every MAT row marks the start of one complete sample interval. Timestamps start
at zero, advance without gaps by `samplePeriodUs`, and the final row ends exactly
at `stopTimeSeconds`; there is no row whose timestamp equals the stop time.

If the first response times out, the bridge retries once with the identical
payload. Firmware recognizes only that last identical command as an idempotent
replay, returns its result with the replay flag, and does not execute the ticks
again. Reusing its provenance with different content or submitting a stale
sequence is rejected. `Timeout`, `TransportError`, and `MismatchedOutput` stop
the deterministic replay; `OutOfTolerance` is recorded and replay continues.

For each matching MCU response, the GUI compares `pwm_command` directly with
the sample's imported `expected_pwm`. The persisted absolute error therefore
always corresponds to the expected and MCU values displayed in the run detail.

Runs with tolerance warnings complete normally. A deterministic protocol,
transport, or timeout failure makes the run `Failed`. Cancelling or an
unexpected bridge failure produces `Aborted` or `Failed`; cleanup always tries
to send `HIL_STOP` and restore the captured gains, pole pairs, setpoint, and
bridge mode. PWM frequency and structural timing are never modified by the
validation. A restoration failure is persisted with the run.

## GUI and API

- `/runs`: import a MAT, set tolerance/deadlines, list, edit metadata, and delete past runs.
- `/runs/{id}`: execute, inspect, or delete a persisted run.
- `GET /api/validation/runs`
- `GET /api/validation/runs/{id}`
- `POST /api/validation/import`
- `POST /api/validation/runs/{id}/execute`
- `PATCH /api/validation/runs/{id}/metadata`
- `DELETE /api/validation/runs/{id}`
- `GET /api/bridge/validation-reference`

Deleting a run removes its sample rows, metadata, and immutable MAT artifact.
Running validations cannot be deleted.

Editing a run changes only its local name and description in `runs.db`. The
original MAT artifact and its imported manifest remain unchanged as provenance.

The bridge remains the only owner of HID. The legacy `/pil` page remains a
live transport diagnostic and is not the source of persistent validation
evidence.
