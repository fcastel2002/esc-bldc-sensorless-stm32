# Simulation Validation Runs

The GUI validates an already-executed discrete Simulink experiment against the
STM32 speed controller. It does not launch MATLAB or allow the MCU to control

## Import contract

Import a MAT v7 file created by `export_hil_validation_vector`. The GUI only
accepts the flat `esc_validation_v1` structure documented in
[`PIL_SIMULINK.md`](PIL_SIMULINK.md). `expected_pwm` and the MCU
`pwm_command` are logical PWM counts, so they are compared without percentage
conversion.

New models should use `export_simulink_validation_run`, which runs the named
model for a requested stop time and extracts only explicitly declared `logsout`
signals. Its optional `experimentSignals` payload remains part of the stored
MAT artifact for later analysis.

Each import creates a persistent run under `%LOCALAPPDATA%\EscGui\validation-runs`:

- `runs.db` stores run metadata, configuration, metrics, and sample results.
- `artifacts/<run-id>/` stores the immutable original MAT file.

## Replay

The bridge applies the configuration from the imported manifest in RAM only;
it never sends `SAVE_CONFIG`. It then starts HIL with the run's configurable
MCU input timeout and replays samples one at a time.

A sample is accepted only when `HIL_GET_OUTPUTS` reports the same `run_id` and
`source_sequence`, with a newer `output_generation`. Arrival time is used only
to measure latency. The bridge records `Passed`, `OutOfTolerance`, `Timeout`,
`MismatchedOutput`, or `TransportError` for every sample and continues after
individual failures.

Runs with warnings complete normally. A run becomes `Failed` only when its
count of non-warm-up timeouts exceeds the configured limit. Cancelling or an
unexpected bridge failure produces `Aborted` or `Failed`; cleanup always tries
to send `HIL_STOP`.

## GUI and API

- `/runs`: import a MAT, set tolerance/deadlines, and list past runs.
- `/runs/{id}`: execute a persisted run and inspect every comparison sample.
- `GET /api/validation/runs`
- `GET /api/validation/runs/{id}`
- `POST /api/validation/import`
- `POST /api/validation/runs/{id}/execute`

The bridge remains the only owner of HID. The legacy `/pil` page remains a
live transport diagnostic and is not the source of persistent validation
evidence.
