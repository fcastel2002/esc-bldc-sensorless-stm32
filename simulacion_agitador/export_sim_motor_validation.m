function [validationVector, manifest, outputPath] = export_sim_motor_validation(options)
%EXPORT_SIM_MOTOR_VALIDATION Simulate sim_motor and export a GUI validation MAT.
%   KP and KI command both the MCU and the canonical-count RPM PI in sim_motor.

arguments
    options.stopTime (1, 1) double {mustBeFinite, mustBePositive} = 3
    options.targetRpm (1, 1) double {mustBeFinite, mustBeNonnegative} = 1000
    options.kp (1, 1) double {mustBeFinite, mustBeNonnegative} = 0.28
    options.ki (1, 1) double {mustBeFinite, mustBeNonnegative} = 1.00
    options.kd (1, 1) double {mustBeFinite, mustBeNonnegative} = 0
    options.polePairs (1, 1) double {mustBeFinite, mustBePositive} = 2
    options.runId = []
    options.sampleTime (1, 1) double {mustBeFinite, mustBePositive} = 0.02
    options.outputPath (1, 1) string = ""
    options.experimentName (1, 1) string = "Arranque sim_motor"
    options.description (1, 1) string = "Rampa de velocidad con PID local"
    options.bridgeUrl (1, 1) string = "http://localhost:5187"
end

modelDirectory = fileparts(mfilename("fullpath"));
modelPath = fullfile(modelDirectory, "sim_motor.mdl");
modelName = "sim_motor";

if isempty(options.runId)
    epochMicroseconds = floor(posixtime(datetime("now", "TimeZone", "UTC")) * 1e6);
    runId = uint32(mod(epochMicroseconds, double(intmax("uint32") - 1)) + 1);
else
    runId = localRunId(options.runId);
end

if strlength(options.outputPath) == 0
    outputPath = fullfile(modelDirectory, "sim_motor_validation.mat");
else
    outputPath = options.outputPath;
end

structuralReference = localReadStructuralReference(options.bridgeUrl);

wasLoaded = bdIsLoaded(modelName);
if ~wasLoaded
    load_system(modelPath);
    cleanup = onCleanup(@()close_system(modelName, 0));
end

simulationInput = Simulink.SimulationInput(modelName);
simulationInput = simulationInput.setModelParameter( ...
    "StopTime", num2str(options.stopTime, 17));
simulationInput = simulationInput.setBlockParameter( ...
    modelName + "/Final Speed RPM", "Value", num2str(options.targetRpm, 17));

pidPath = modelName + "/PID Controller";
kp = localValidateGain(options.kp, "kp");
ki = localValidateGain(options.ki, "ki");
kd = localValidateGain(options.kd, "kd");
if kd ~= 0
    error("export_sim_motor_validation:KdNotImplemented", ...
        "KD must be zero for RPM PI algorithm version 2.");
end
simulationInput = simulationInput.setBlockParameter(pidPath, "P", num2str(kp, 17));
simulationInput = simulationInput.setBlockParameter(pidPath, "I", num2str(ki, 17));
simulationInput = simulationInput.setBlockParameter(pidPath, "D", "0");

modelPolePairs = str2double(get_param(modelName + "/BLDC", "nPolePairs"));
if ~isfinite(modelPolePairs) || options.polePairs ~= modelPolePairs || options.polePairs ~= 2
    error("export_sim_motor_validation:PolePairMismatch", ...
        "Requested MCU polePairs=%g does not match every sim_motor electrical model " + ...
        "component (BLDC=%g, commutator=2).", ...
        options.polePairs, modelPolePairs);
end

minimumDuty = structuralReference.minimumPwmCounts / structuralReference.pwmArrCounts;
simulationInput = simulationInput.setBlockParameter( ...
    modelName + "/Duty Saturation", "LowerLimit", num2str(minimumDuty, 17));
simulationOutput = sim(simulationInput);

if ~ismember("logsout", string(simulationOutput.who()))
    error("export_sim_motor_validation:MissingLogsout", ...
        "sim_motor did not produce the logsout dataset.");
end

rpmElement = simulationOutput.logsout.get("rpm");
if isempty(rpmElement) || ~isa(rpmElement.Values, "timeseries")
    error("export_sim_motor_validation:MissingRpm", ...
        "sim_motor must log the 'rpm' signal as a timeseries.");
end

motorRpm = [double(rpmElement.Values.Time(:)), double(rpmElement.Values.Data(:))];
config = struct( ...
    "targetRpm", options.targetRpm, ...
    "runId", runId, ...
    "sampleTime", options.sampleTime, ...
    "stopTimeSeconds", options.stopTime, ...
    "kp", kp, ...
    "ki", ki, ...
    "kd", kd, ...
    "polePairs", options.polePairs, ...
    "pwmFrequency", structuralReference.pwmFrequencyHz, ...
    "pwmArr", structuralReference.pwmArrCounts, ...
    "timerHz", structuralReference.speedTimerHz, ...
    "speedMinPeriod", structuralReference.speedMinPeriodTicks, ...
    "speedMaxPeriod", structuralReference.speedMaxPeriodTicks, ...
    "dt", structuralReference.controllerDtSeconds, ...
    "minimumPwm", structuralReference.minimumPwmCounts, ...
    "algorithmVersion", structuralReference.algorithmVersion, ...
    "experimentName", options.experimentName, ...
    "description", options.description);

[validationVector, manifest, outputPath] = export_hil_validation_vector( ...
    motorRpm, config, outputPath);

if ~wasLoaded
    clear cleanup
end

fprintf("Validation MAT generated: %s\n", outputPath);
end

function reference = localReadStructuralReference(bridgeUrl)
endpoint = strip(bridgeUrl, "right", "/") + "/api/bridge/validation-reference";
try
    reference = webread(endpoint, weboptions("Timeout", 3, "ContentType", "json"));
catch exception
    error("export_sim_motor_validation:BridgeUnavailable", ...
        "Could not read the live ESC validation reference from %s: %s", endpoint, exception.message);
end

required = ["schemaVersion", "algorithmVersion", "pwmFrequencyHz", ...
    "pwmArrCounts", "speedTimerHz", "speedMinPeriodTicks", ...
    "speedMaxPeriodTicks", "controllerDtSeconds", "minimumPwmCounts"];
for name = required
    if ~isfield(reference, name) || ~isnumeric(reference.(name)) || ...
            ~isscalar(reference.(name)) || ~isfinite(reference.(name))
        error("export_sim_motor_validation:InvalidBridgeReference", ...
            "Bridge response is missing a finite scalar '%s'.", name);
    end
end
if reference.schemaVersion ~= 1 || reference.algorithmVersion ~= 2
    error("export_sim_motor_validation:UnsupportedReference", ...
        "Unsupported validation reference schema %g or algorithm %g.", ...
        reference.schemaVersion, reference.algorithmVersion);
end
end

function runId = localRunId(value)
if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || ...
        value < 1 || value > double(intmax("uint32")) || value ~= floor(value)
    error("export_sim_motor_validation:InvalidRunId", ...
        "runId must be a nonzero uint32-compatible integer.");
end
runId = uint32(value);
end

function value = localValidateGain(value, name)
if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || value < 0 || value > 10
    error("export_sim_motor_validation:InvalidPidGain", ...
        "PID gain %s must be a finite numeric scalar in [0, 10].", name);
end

value = double(value);
encodedValue = round(value * 100) / 100;
if abs(value - encodedValue) > 1e-12
    error("export_sim_motor_validation:UnrepresentablePidGain", ...
        "PID gain %s=%.17g is not representable by the protocol's 0.01 resolution.", ...
        name, value);
end
value = encodedValue;
end
