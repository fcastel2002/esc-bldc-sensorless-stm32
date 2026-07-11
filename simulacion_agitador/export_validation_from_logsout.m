function [artifact, report, outputPath] = export_validation_from_logsout( ...
    logsout, modelName, stopTime, spec, config, outputPath)
%EXPORT_VALIDATION_FROM_LOGSOUT Export explicitly declared Simulink logs to MAT v7.
%   This function is the model-independent core of EXPORT_SIMULINK_VALIDATION_RUN.
%   LOGSOUT must be a Simulink signal logging Dataset. SPEC declares the four
%   validation signals and optional extra signals. CONFIG supplies controller
%   metadata and can optionally override the automatically generated runId.

arguments
    logsout (1, 1) Simulink.SimulationData.Dataset
    modelName (1, 1) string
    stopTime (1, 1) double {mustBeFinite, mustBePositive}
    spec (1, 1) struct
    config (1, 1) struct
    outputPath (1, 1) string = ""
end

validationSpec = localValidationSpec(spec);
referenceConfig = localReferenceConfig(config);
runId = localRunId(config);

[pwmTime, expectedPwm] = localReadScalarSignal(logsout, validationSpec.expectedPwm);
localValidateControllerTimeline(pwmTime, referenceConfig.dt);

[speedTime, speedValues] = localReadScalarSignal(logsout, validationSpec.speedRpm);
[enableTime, enableValues] = localReadScalarSignal(logsout, validationSpec.enable);
[targetTime, targetValues] = localReadScalarSignal(logsout, validationSpec.targetRpm);

speedRpm = localUInt16(localZohAlign(speedTime, speedValues, pwmTime, validationSpec.speedRpm), ...
    validationSpec.speedRpm);
enable = localEnable(localZohAlign(enableTime, enableValues, pwmTime, validationSpec.enable), ...
    validationSpec.enable);
targetRpm = localUInt16(localZohAlign(targetTime, targetValues, pwmTime, validationSpec.targetRpm), ...
    validationSpec.targetRpm);
expectedPwm = localUInt16(expectedPwm, validationSpec.expectedPwm);

if any(targetRpm ~= referenceConfig.targetRpm)
    error("export_validation_from_logsout:VariableTarget", ...
        "Signal '%s' must remain equal to config.targetRpm for the current MCU validation protocol.", ...
        validationSpec.targetRpm);
end
if any(expectedPwm > referenceConfig.pwmArr)
    error("export_validation_from_logsout:PwmOutOfRange", ...
        "Signal '%s' exceeds config.pwmArr.", validationSpec.expectedPwm);
end

sourceSequence = uint32((1:numel(pwmTime)).');
validationVector = table(pwmTime, repmat(runId, numel(pwmTime), 1), ...
    sourceSequence, speedRpm, enable, targetRpm, expectedPwm, ...
    VariableNames=["SimulationTime", "RunId", "SourceSequence", "SpeedRpm", ...
    "Enable", "TargetRpm", "ExpectedPwm"]);

experimentSignals = localExtraSignals(logsout, spec);
manifest = localManifest(modelName, stopTime, runId, referenceConfig, spec, config);
esc_validation_v1 = struct( ...
    "schema_version", uint32(1), ...
    "manifest_json", char(jsonencode(manifest)), ...
    "simulation_time_s", pwmTime, ...
    "run_id", repmat(runId, numel(pwmTime), 1), ...
    "source_sequence", sourceSequence, ...
    "speed_rpm", speedRpm, ...
    "enable", enable, ...
    "target_rpm", targetRpm, ...
    "expected_pwm", expectedPwm);

if strlength(outputPath) == 0
    outputPath = fullfile(pwd, modelName + "_validation.mat");
end
outputPath = string(outputPath);
outputDirectory = fileparts(outputPath);
if ~isfolder(outputDirectory)
    mkdir(outputDirectory);
end
save(outputPath, "validationVector", "manifest", "esc_validation_v1", ...
    "experimentSignals", "-v7");

artifact = struct( ...
    "validationVector", validationVector, ...
    "manifest", manifest, ...
    "escValidation", esc_validation_v1, ...
    "experimentSignals", experimentSignals);
report = struct( ...
    "modelName", modelName, ...
    "runId", runId, ...
    "sampleCount", numel(pwmTime), ...
    "controllerPeriodSeconds", referenceConfig.dt, ...
    "validationSignals", validationSpec, ...
    "extraSignalCount", numel(experimentSignals), ...
    "outputPath", outputPath);
end

function validationSpec = localValidationSpec(spec)
if ~isfield(spec, "validation") || ~isstruct(spec.validation) || ~isscalar(spec.validation)
    error("export_validation_from_logsout:MissingValidationSpec", ...
        "spec.validation must declare speedRpm, expectedPwm, enable, and targetRpm.");
end

validation = spec.validation;
validationSpec = struct( ...
    "speedRpm", localSignalName(validation, "speedRpm"), ...
    "expectedPwm", localSignalName(validation, "expectedPwm"), ...
    "enable", localSignalName(validation, "enable"), ...
    "targetRpm", localSignalName(validation, "targetRpm"));
end

function signalName = localSignalName(validation, fieldName)
if ~isfield(validation, fieldName) || strlength(string(validation.(fieldName))) == 0
    error("export_validation_from_logsout:MissingSignalName", ...
        "spec.validation.%s must name a logsout signal.", fieldName);
end
signalName = string(validation.(fieldName));
end

function referenceConfig = localReferenceConfig(config)
referenceConfig = struct( ...
    "kp", localFiniteScalar(config, "kp"), ...
    "ki", localFiniteScalar(config, "ki"), ...
    "kd", localFiniteScalar(config, "kd"), ...
    "pwmFrequency", localPositiveInteger(config, "pwmFrequency", 65535), ...
    "pwmArr", localPositiveInteger(config, "pwmArr", 65535), ...
    "polePairs", localPositiveInteger(config, "polePairs", 255), ...
    "dt", localPositiveScalar(config, "controllerPeriodSeconds"), ...
    "targetRpm", uint16(localPositiveInteger(config, "targetRpm", 65535)));
end

function value = localFiniteScalar(config, fieldName)
if ~isfield(config, fieldName) || ~isnumeric(config.(fieldName)) || ...
        ~isscalar(config.(fieldName)) || ~isfinite(config.(fieldName))
    error("export_validation_from_logsout:InvalidConfig", ...
        "config.%s must be a finite numeric scalar.", fieldName);
end
value = double(config.(fieldName));
end

function value = localPositiveScalar(config, fieldName)
value = localFiniteScalar(config, fieldName);
if value <= 0
    error("export_validation_from_logsout:InvalidConfig", ...
        "config.%s must be positive.", fieldName);
end
end

function value = localPositiveInteger(config, fieldName, maximum)
value = localPositiveScalar(config, fieldName);
if value > maximum || value ~= floor(value)
    error("export_validation_from_logsout:InvalidConfig", ...
        "config.%s must be an integer in [1, %.0f].", fieldName, maximum);
end
end

function runId = localRunId(config)
if isfield(config, "runId") && ~isempty(config.runId)
    value = config.runId;
    if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || ...
            value < 1 || value > double(intmax("uint32")) || value ~= floor(value)
        error("export_validation_from_logsout:InvalidRunId", ...
            "config.runId must be a nonzero uint32 value.");
    end
    runId = uint32(value);
    return;
end

epochMicroseconds = floor(posixtime(datetime("now", "TimeZone", "UTC")) * 1e6);
runId = uint32(mod(epochMicroseconds, double(intmax("uint32") - 1)) + 1);
end

function [time, values] = localReadScalarSignal(logsout, signalName)
signal = localGetSignal(logsout, signalName);
if ~isa(signal.Values, "timeseries")
    error("export_validation_from_logsout:UnsupportedSignal", ...
        "Signal '%s' must log as a timeseries.", signalName);
end

time = double(signal.Values.Time(:));
values = double(signal.Values.Data);
if ~isvector(values)
    error("export_validation_from_logsout:NonScalarSignal", ...
        "Validation signal '%s' must be scalar at every sample.", signalName);
end
values = values(:);
localValidateSignalTimeline(time, values, signalName);
end

function signal = localGetSignal(logsout, signalName)
try
    signal = logsout.get(signalName);
catch exception
    error("export_validation_from_logsout:MissingSignal", ...
        "logsout does not contain declared signal '%s': %s", signalName, exception.message);
end

if isempty(signal)
    error("export_validation_from_logsout:MissingSignal", ...
        "logsout does not contain declared signal '%s'.", signalName);
end
end

function localValidateSignalTimeline(time, values, signalName)
if isempty(time) || numel(time) ~= numel(values) || any(~isfinite(time)) || any(~isfinite(values))
    error("export_validation_from_logsout:InvalidSignal", ...
        "Signal '%s' must contain equally sized finite time and value vectors.", signalName);
end
if any(diff(time) <= 0)
    error("export_validation_from_logsout:InvalidTimeline", ...
        "Signal '%s' time values must be strictly increasing.", signalName);
end
end

function localValidateControllerTimeline(time, controllerPeriod)
if numel(time) < 2
    error("export_validation_from_logsout:TooFewSamples", ...
        "The expected PWM signal must contain at least two controller samples.");
end

tolerance = max(1e-12, controllerPeriod * 1e-9);
if any(abs(diff(time) - controllerPeriod) > tolerance)
    error("export_validation_from_logsout:ControllerPeriodMismatch", ...
        "Expected PWM timeline must use config.controllerPeriodSeconds exactly.");
end
end

function alignedValues = localZohAlign(sourceTime, sourceValues, targetTime, signalName)
tolerance = max(1e-12, eps(max(abs(targetTime))));
if sourceTime(1) > targetTime(1) + tolerance || sourceTime(end) < targetTime(end) - tolerance
    error("export_validation_from_logsout:InsufficientCoverage", ...
        "Signal '%s' does not cover the expected PWM timeline.", signalName);
end

alignedValues = interp1(sourceTime, sourceValues, targetTime, "previous");
if any(isnan(alignedValues))
    error("export_validation_from_logsout:AlignmentFailure", ...
        "Signal '%s' could not be aligned by zero-order hold.", signalName);
end
end

function values = localUInt16(values, signalName)
if any(values < 0 | values > 65535 | values ~= floor(values))
    error("export_validation_from_logsout:InvalidUInt16", ...
        "Signal '%s' must contain integer values in [0, 65535].", signalName);
end
values = uint16(values);
end

function values = localEnable(values, signalName)
if any(values ~= 0 & values ~= 1)
    error("export_validation_from_logsout:InvalidEnable", ...
        "Signal '%s' must contain only 0 or 1.", signalName);
end
values = uint8(values);
end

function experimentSignals = localExtraSignals(logsout, spec)
if ~isfield(spec, "extraSignals") || isempty(spec.extraSignals)
    experimentSignals = struct("name", {}, "unit", {}, "time_s", {}, "values", {});
    return;
end
if ~isstruct(spec.extraSignals)
    error("export_validation_from_logsout:InvalidExtraSignals", ...
        "spec.extraSignals must be a struct array with name and optional unit.");
end

template = struct("name", "", "unit", "", "time_s", zeros(0, 1), "values", zeros(0, 1));
experimentSignals = repmat(template, numel(spec.extraSignals), 1);
for index = 1:numel(spec.extraSignals)
    declaration = spec.extraSignals(index);
    signalName = localExtraSignalName(declaration);
    signal = localGetSignal(logsout, signalName);
    if ~isa(signal.Values, "timeseries")
        error("export_validation_from_logsout:UnsupportedSignal", ...
            "Extra signal '%s' must log as a timeseries.", signalName);
    end

    signalTime = double(signal.Values.Time(:));
    signalValues = double(signal.Values.Data);
    if size(signalValues, 1) ~= numel(signalTime) || any(~isfinite(signalTime)) || any(~isfinite(signalValues), "all")
        error("export_validation_from_logsout:InvalidExtraSignal", ...
            "Extra signal '%s' must have finite samples aligned to its time vector.", signalName);
    end
    if any(diff(signalTime) <= 0)
        error("export_validation_from_logsout:InvalidTimeline", ...
            "Extra signal '%s' time values must be strictly increasing.", signalName);
    end

    experimentSignals(index) = struct( ...
        "name", signalName, ...
        "unit", localExtraSignalUnit(declaration), ...
        "time_s", signalTime, ...
        "values", signalValues);
end
end

function signalName = localExtraSignalName(declaration)
if ~isfield(declaration, "name") || strlength(string(declaration.name)) == 0
    error("export_validation_from_logsout:InvalidExtraSignals", ...
        "Each extra signal declaration must provide a nonempty name.");
end
signalName = string(declaration.name);
end

function unit = localExtraSignalUnit(declaration)
if isfield(declaration, "unit")
    unit = string(declaration.unit);
else
    unit = "raw";
end
end

function manifest = localManifest(modelName, stopTime, runId, referenceConfig, spec, config)
experimentName = localOptionalText(config, "experimentName", "Unnamed validation");
description = localOptionalText(config, "description", "");
manifest = struct( ...
    "schemaVersion", uint32(1), ...
    "experimentName", experimentName, ...
    "description", description, ...
    "createdAtUtc", string(datetime("now", "TimeZone", "UTC", ...
        "Format", "yyyy-MM-dd'T'HH:mm:ss.SSSXXX")), ...
    "modelName", modelName, ...
    "stopTimeSeconds", stopTime, ...
    "samplePeriodUs", uint64(round(referenceConfig.dt * 1e6)), ...
    "targetRpm", referenceConfig.targetRpm, ...
    "runId", runId, ...
    "signalMap", spec, ...
    "referenceConfig", rmfield(referenceConfig, "targetRpm"));
end

function value = localOptionalText(config, fieldName, defaultValue)
if isfield(config, fieldName) && strlength(string(config.(fieldName))) > 0
    value = string(config.(fieldName));
else
    value = string(defaultValue);
end
end
