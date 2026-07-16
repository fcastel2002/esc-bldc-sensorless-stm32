function [validationVector, manifest, outputPath] = export_hil_validation_vector(motor_rpm, config, outputPath)
%EXPORT_HIL_VALIDATION_VECTOR Build a provenance-tagged HIL validation MAT file.
%   MOTOR_RPM is an Nx2 [simulation time, RPM] numeric log. If MOTOR_RPM is
%   omitted, the function reads motor_rpm from the base workspace. CONFIG
%   requires targetRpm and runId, accepts enable, and defaults sampleTime to
%   0.02 seconds. Controller fields are forwarded to the firmware PI reference.
%   The output preserves the legacy validationVector/manifest variables and
%   includes esc_validation_v2, a flat typed contract for GUI import.

if nargin < 1 || isempty(motor_rpm)
    motor_rpm = evalin("base", "motor_rpm");
elseif isstruct(motor_rpm)
    if nargin < 2 || isempty(config)
        config = motor_rpm;
        motor_rpm = evalin("base", "motor_rpm");
    else
        error("export_hil_validation_vector:InvalidInput", ...
            "motor_rpm must be an Nx2 numeric log when config is supplied.");
    end
end
if nargin < 2 || isempty(config)
    config = struct();
end
if nargin < 3 || isempty(outputPath)
    outputPath = fullfile(fileparts(mfilename("fullpath")), "hil_validation_vector.mat");
end
outputPath = string(outputPath);

if ~isstruct(config) || ~isscalar(config)
    error("export_hil_validation_vector:InvalidConfig", "config must be a scalar struct.");
end
if ~isnumeric(motor_rpm) || size(motor_rpm, 2) ~= 2 || isempty(motor_rpm)
    error("export_hil_validation_vector:InvalidLog", "motor_rpm must be a nonempty Nx2 numeric array.");
end
if ~isfield(config, "targetRpm") || ~isfield(config, "runId")
    error("export_hil_validation_vector:MissingConfig", "config must contain targetRpm and runId.");
end

targetRpm = localUint16(config.targetRpm, "targetRpm");
runId = localUint32Nonzero(config.runId, "runId");
sampleTime = localPositiveScalar(localField(config, "sampleTime", 0.02), "sampleTime");
enable = uint8(localLogicalValue(localField(config, "enable", true), "enable"));
if enable ~= 1
    error("export_hil_validation_vector:UnsupportedEnable", ...
        "Deterministic validation schema v2 requires enable=true.");
end
referenceConfig = localReferenceConfig(config);

time = double(motor_rpm(:, 1));
rpm = double(motor_rpm(:, 2));
if any(~isfinite(time)) || any(~isfinite(rpm))
    error("export_hil_validation_vector:InvalidLog", "motor_rpm values must be finite.");
end
[time, order] = sort(time);
rpm = rpm(order);
[time, lastIndex] = unique(time, "last");
rpm = rpm(lastIndex);
if time(1) > 0 || time(end) <= 0
    error("export_hil_validation_vector:InvalidLog", ...
        "motor_rpm must span time zero and include a positive simulation interval.");
end

stopTimeSeconds = localPositiveScalar( ...
    localField(config, "stopTimeSeconds", time(end)), "stopTimeSeconds");
availableStopTime = min(stopTimeSeconds, time(end));
sampleCount = floor(availableStopTime / sampleTime + 1e-12);
if sampleCount < 1
    error("export_hil_validation_vector:InvalidHorizon", ...
        "The validation horizon must contain at least one complete sample interval.");
end
% Each timestamp marks the interval start; expected PWM is captured after all
% controller steps in that complete interval.
simulationTime = (0:(sampleCount - 1)).' * sampleTime;
if isscalar(time)
    measuredRpm = repmat(rpm, size(simulationTime));
else
    measuredRpm = interp1(time, rpm, simulationTime, "linear", NaN);
    measuredRpm(simulationTime < time(1)) = rpm(1);
    measuredRpm(simulationTime > time(end)) = rpm(end);
end
speedRpm = zeros(numel(simulationTime), 1, "uint16");
for index = 1:numel(speedRpm)
    speedRpm(index) = localUint16(measuredRpm(index), "motor_rpm RPM");
end

stepsPerInput = sampleTime / double(referenceConfig.dt);
roundedSteps = round(stepsPerInput);
if roundedSteps < 1 || abs(stepsPerInput - roundedSteps) > 1e-9
    error("export_hil_validation_vector:IncompatibleTiming", ...
        "sampleTime must be an integer multiple of reference config dt.");
end
if numel(speedRpm) > double(intmax("uint32"))
    error("export_hil_validation_vector:SequenceOverflow", "The vector exceeds uint32 source sequence capacity.");
end

expectedPwm = zeros(numel(speedRpm), 1, "uint16");
state = struct("integralQ16", int32(0), "previousError", int32(0));
for index = 1:numel(speedRpm)
    for step = 1:roundedSteps
        [state, expectedPwm(index)] = firmware_pi_reference_step( ...
            state, double(speedRpm(index)), double(targetRpm), referenceConfig);
    end
end

sourceSequence = uint32((1:numel(speedRpm)).');
validationVector = table(simulationTime, repmat(runId, numel(speedRpm), 1), ...
    sourceSequence, speedRpm, repmat(enable, numel(speedRpm), 1), ...
    repmat(targetRpm, numel(speedRpm), 1), expectedPwm, ...
    VariableNames=["SimulationTime", "RunId", "SourceSequence", "SpeedRpm", ...
    "Enable", "TargetRpm", "ExpectedPwm"]);
manifest = struct( ...
    "referenceConfig", referenceConfig, ...
    "targetRpm", targetRpm, ...
    "stopTimeSeconds", sampleCount * sampleTime, ...
    "sampleTime", sampleTime, ...
    "runId", runId, ...
    "enable", enable, ...
    "createdAt", datetime("now", "TimeZone", "local"));

flatManifest = struct( ...
    "schemaVersion", uint32(2), ...
    "experimentName", string(localField(config, "experimentName", "Unnamed validation")), ...
    "description", string(localField(config, "description", "")), ...
    "createdAtUtc", string(datetime("now", "TimeZone", "UTC", "Format", "yyyy-MM-dd'T'HH:mm:ss.SSSXXX")), ...
    "stopTimeSeconds", sampleCount * sampleTime, ...
    "samplePeriodUs", uint64(round(sampleTime * 1e6)), ...
    "targetRpm", targetRpm, ...
    "referenceConfig", referenceConfig);
esc_validation_v2 = struct( ...
    "schema_version", uint32(2), ...
    "manifest_json", char(jsonencode(flatManifest)), ...
    "simulation_time_s", simulationTime, ...
    "run_id", repmat(runId, numel(speedRpm), 1), ...
    "source_sequence", sourceSequence, ...
    "speed_rpm", speedRpm, ...
    "enable", repmat(enable, numel(speedRpm), 1), ...
    "target_rpm", repmat(targetRpm, numel(speedRpm), 1), ...
    "expected_pwm", expectedPwm);

outputDirectory = fileparts(outputPath);
if ~isfolder(outputDirectory)
    error("export_hil_validation_vector:MissingDirectory", ...
        "Output directory does not exist: %s", outputDirectory);
end
save(outputPath, "validationVector", "manifest", "esc_validation_v2", "-v7");
end

function config = localReferenceConfig(source)
config = struct( ...
    "kp", localRequiredField(source, "kp"), ...
    "ki", localRequiredField(source, "ki"), ...
    "kd", localRequiredField(source, "kd"), ...
    "pwmFrequency", localRequiredField(source, "pwmFrequency"), ...
    "pwmArr", localRequiredField(source, "pwmArr"), ...
    "polePairs", localRequiredField(source, "polePairs"), ...
    "timerHz", localRequiredField(source, "timerHz"), ...
    "speedMinPeriod", localRequiredField(source, "speedMinPeriod"), ...
    "speedMaxPeriod", localRequiredField(source, "speedMaxPeriod"), ...
    "dt", localRequiredField(source, "dt"), ...
    "minimumPwm", localRequiredField(source, "minimumPwm"), ...
    "algorithmVersion", localRequiredField(source, "algorithmVersion"));
end

function value = localRequiredField(source, name)
if ~isfield(source, name) || isempty(source.(name))
    error("export_hil_validation_vector:MissingReference", ...
        "config.%s must come from the simulation or live ESC validation reference.", name);
end
value = source.(name);
end

function value = localField(source, name, defaultValue)
if isfield(source, name) && ~isempty(source.(name))
    value = source.(name);
else
    value = defaultValue;
end
end

function value = localPositiveScalar(value, name)
if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || value <= 0
    error("export_hil_validation_vector:InvalidConfig", "%s must be a finite positive scalar.", name);
end
value = double(value);
end

function value = localUint16(value, name)
if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value)
    error("export_hil_validation_vector:InvalidValue", "%s must be a finite numeric scalar.", name);
end
value = uint16(min(65535, max(0, floor(double(value)))));
end

function value = localUint32Nonzero(value, name)
if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || ...
        value < 1 || value > double(intmax("uint32")) || value ~= floor(value)
    error("export_hil_validation_vector:InvalidConfig", ...
        "%s must be a nonzero uint32 value.", name);
end
value = uint32(value);
end

function value = localLogicalValue(value, name)
if (islogical(value) || isnumeric(value)) && isscalar(value) && isfinite(double(value))
    value = double(value) ~= 0;
else
    error("export_hil_validation_vector:InvalidConfig", "%s must be a scalar logical value.", name);
end
end
