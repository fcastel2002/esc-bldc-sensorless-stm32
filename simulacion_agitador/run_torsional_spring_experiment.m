function [results, experiment, outputPath] = run_torsional_spring_experiment(options)
%RUN_TORSIONAL_SPRING_EXPERIMENT Sweep the virtual coupling stiffness.
%   The experiment changes only SimulationInput values. It does not save
%   modifications to sim_motor.mdl or identify the physical coupling. The
%   result MAT contains plain numeric traces and can be inspected without
%   rerunning Simulink.

arguments
    options.springRates (1, :) double {mustBeFinite, mustBePositive} = ...
        [1e-3, 3e-3, 1e-2, 3e-2, 1e-1, 3e-1, 1, 3, 10]
    options.targetRpm (1, 1) double {mustBeFinite, mustBePositive} = 1000
    options.stopTime (1, 1) double {mustBeFinite, mustBePositive} = 10
    options.settleTime (1, 1) double {mustBeFinite, mustBePositive} = 2
    options.maxLinearAngleDeg (1, 1) double {mustBeFinite, mustBePositive} = 15
    options.makePlots (1, 1) logical = true
    options.outputPath (1, 1) string = ""
end

if options.settleTime >= options.stopTime
    error("run_torsional_spring_experiment:InvalidSettleTime", ...
        "settleTime must be shorter than stopTime.");
end

modelDirectory = fileparts(mfilename("fullpath"));
modelPath = fullfile(modelDirectory, "sim_motor.mdl");
modelName = "sim_motor";
springPath = modelName + "/Rotational Spring";
targetPath = modelName + "/Final Speed RPM1";
outputPath = localOutputPath(options.outputPath, modelDirectory);

wasLoaded = bdIsLoaded(modelName);
if ~wasLoaded
    load_system(modelPath);
    cleanup = onCleanup(@() close_system(modelName, 0));
end

motorInertia = localBlockScalar(modelName + "/BLDC1", "J");
driveMagnetInertia = localBlockScalar(modelName + "/Imán impulso", "inertia");
agitatorMagnetInertia = localBlockScalar(modelName + "/Imán Agitador", "inertia");
driveInertia = motorInertia + driveMagnetInertia;
equivalentInertia = driveInertia * agitatorMagnetInertia / ...
    (driveInertia + agitatorMagnetInertia);

springRates = options.springRates(:);
runCount = numel(springRates);
naturalFrequencyHz = sqrt(springRates / equivalentInertia) / (2 * pi);
motorRpm = nan(runCount, 1);
driveRpm = nan(runCount, 1);
agitatorRpm = nan(runCount, 1);
speedDifferenceRpm = nan(runCount, 1);
targetErrorRpm = nan(runCount, 1);
steadyAngleDeg = nan(runCount, 1);
steadyAnglePeakDeg = nan(runCount, 1);
runAnglePeakDeg = nan(runCount, 1);
steadyTorqueMilliNm = nan(runCount, 1);
steadyTorqueRmsMilliNm = nan(runCount, 1);
runTorquePeakMilliNm = nan(runCount, 1);
synchronized = false(runCount, 1);
tracksTarget = false(runCount, 1);
withinLinearRegion = false(runCount, 1);
validOperatingPoint = false(runCount, 1);
status = strings(runCount, 1);
emptySignal = struct('time', zeros(0, 1), 'values', zeros(0, 1), 'unit', "");
emptyRun = struct('K_NmPerRad', nan, 'motorRpm', emptySignal, ...
    'driveMagnetRpm', emptySignal, 'agitatorMagnetRpm', emptySignal, ...
    'relativeAngleDeg', emptySignal, 'springTorqueNm', emptySignal, ...
    'status', "not-run");
traces = repmat(emptyRun, runCount, 1);

steadyStart = options.stopTime - options.settleTime;
speedToleranceRpm = max(5, 0.01 * options.targetRpm);
targetToleranceRpm = max(10, 0.02 * options.targetRpm);

for index = 1:runCount
    simulationInput = Simulink.SimulationInput(modelName);
    simulationInput = simulationInput.setModelParameter( ...
        "StopTime", num2str(options.stopTime, 17));
    simulationInput = simulationInput.setBlockParameter( ...
        springPath, "spr_rate", num2str(springRates(index), 17));
    simulationInput = simulationInput.setBlockParameter( ...
        targetPath, "Value", num2str(options.targetRpm, 17));

    try
        simulationOutput = sim(simulationInput);
        if ~ismember("simlog", string(simulationOutput.who()))
            error("run_torsional_spring_experiment:MissingSimlog", ...
                "sim_motor did not produce the Simscape log 'simlog'.");
        end

        simulationLog = simulationOutput.simlog;
        traces(index).K_NmPerRad = springRates(index);
        traces(index).motorRpm = localSeriesTrace( ...
            simulationLog.BLDC1.angular_velocity, "rpm");
        traces(index).driveMagnetRpm = localSeriesTrace( ...
            simulationLog.Imn_impulso.w, "rpm");
        traces(index).agitatorMagnetRpm = localSeriesTrace( ...
            simulationLog.Imn_Agitador.w, "rpm");
        traces(index).relativeAngleDeg = localSeriesTrace( ...
            simulationLog.Rotational_Spring.phi, "deg");
        traces(index).springTorqueNm = localSeriesTrace( ...
            simulationLog.Rotational_Spring.t, "N*m");

        motorStats = localTraceStats(traces(index).motorRpm, steadyStart);
        driveStats = localTraceStats(traces(index).driveMagnetRpm, steadyStart);
        agitatorStats = localTraceStats(traces(index).agitatorMagnetRpm, steadyStart);
        steadyAngleStats = localTraceStats( ...
            traces(index).relativeAngleDeg, steadyStart);
        runAngleStats = localTraceStats(traces(index).relativeAngleDeg, 0);
        steadyTorqueStats = localTraceStats( ...
            traces(index).springTorqueNm, steadyStart);
        runTorqueStats = localTraceStats(traces(index).springTorqueNm, 0);

        motorRpm(index) = motorStats.mean;
        driveRpm(index) = driveStats.mean;
        agitatorRpm(index) = agitatorStats.mean;
        speedDifferenceRpm(index) = abs(driveRpm(index) - agitatorRpm(index));
        targetErrorRpm(index) = abs(agitatorRpm(index) - options.targetRpm);
        steadyAngleDeg(index) = steadyAngleStats.mean;
        steadyAnglePeakDeg(index) = steadyAngleStats.peakAbsolute;
        runAnglePeakDeg(index) = runAngleStats.peakAbsolute;
        steadyTorqueMilliNm(index) = 1e3 * steadyTorqueStats.mean;
        steadyTorqueRmsMilliNm(index) = 1e3 * steadyTorqueStats.rms;
        runTorquePeakMilliNm(index) = 1e3 * runTorqueStats.peakAbsolute;
        synchronized(index) = speedDifferenceRpm(index) <= speedToleranceRpm;
        tracksTarget(index) = targetErrorRpm(index) <= targetToleranceRpm;
        withinLinearRegion(index) = runAnglePeakDeg(index) <= ...
            options.maxLinearAngleDeg;
        validOperatingPoint(index) = synchronized(index) && tracksTarget(index) && ...
            withinLinearRegion(index);
        status(index) = "ok";
        traces(index).status = "ok";
    catch exception
        status(index) = "error: " + string(exception.identifier);
        traces(index).K_NmPerRad = springRates(index);
        traces(index).status = status(index);
    end
end

results = table(springRates, naturalFrequencyHz, motorRpm, driveRpm, ...
    agitatorRpm, speedDifferenceRpm, targetErrorRpm, steadyAngleDeg, steadyAnglePeakDeg, ...
    runAnglePeakDeg, steadyTorqueMilliNm, steadyTorqueRmsMilliNm, ...
    runTorquePeakMilliNm, synchronized, tracksTarget, withinLinearRegion, ...
    validOperatingPoint, status, ...
    'VariableNames', ["K_NmPerRad", "NaturalFrequencyHz", "MotorRpm", ...
    "DriveMagnetRpm", "AgitatorMagnetRpm", "SpeedDifferenceRpm", ...
    "TargetErrorRpm", "SteadyAngleDeg", "SteadyAnglePeakDeg", "RunAnglePeakDeg", ...
    "SteadyTorqueMilliNm", "SteadyTorqueRmsMilliNm", ...
    "RunTorquePeakMilliNm", "Synchronized", "TracksTarget", ...
    "WithinLinearRegion", "ValidOperatingPoint", "Status"]);

disp(results);

experiment = struct;
experiment.schemaVersion = 1;
experiment.createdAtUtc = datetime("now", "TimeZone", "UTC");
experiment.modelPath = string(modelPath);
experiment.configuration = struct( ...
    'springRates', options.springRates, ...
    'targetRpm', options.targetRpm, ...
    'stopTime', options.stopTime, ...
    'settleTime', options.settleTime, ...
    'maxLinearAngleDeg', options.maxLinearAngleDeg);
experiment.inertias = struct( ...
    'motorKgM2', motorInertia, ...
    'driveMagnetKgM2', driveMagnetInertia, ...
    'agitatorMagnetKgM2', agitatorMagnetInertia, ...
    'driveTotalKgM2', driveInertia, ...
    'relativeEquivalentKgM2', equivalentInertia);
experiment.results = results;
experiment.traces = traces;

save(char(outputPath), 'experiment', 'results', '-v7.3');
fprintf("Experiment MAT generated: %s\n", outputPath);

if options.makePlots
    localPlotResults(results, options.targetRpm, options.maxLinearAngleDeg);
end

if ~wasLoaded
    clear cleanup
end
end

function value = localBlockScalar(blockPath, parameterName)
value = str2double(get_param(blockPath, parameterName));
if ~isfinite(value) || value <= 0
    error("run_torsional_spring_experiment:InvalidModelParameter", ...
        "%s/%s must be a positive numeric scalar.", blockPath, parameterName);
end
end

function trace = localSeriesTrace(node, unit)
series = node.series;
time = series.time;
values = series.values(unit);
trace.time = double(time(:));
trace.values = double(values);
trace.values = trace.values(:);
trace.unit = string(unit);
if numel(trace.time) ~= numel(trace.values)
    error("run_torsional_spring_experiment:InvalidLoggedDimensions", ...
        "Logged series '%s' has incompatible time and value dimensions.", node.id);
end
end

function stats = localTraceStats(trace, startTime)
values = trace.values(trace.time >= startTime);
if isempty(values) || any(~isfinite(values))
    error("run_torsional_spring_experiment:InvalidLoggedSeries", ...
        "A logged series has no finite values in the requested window.");
end

stats.mean = mean(values);
stats.rms = sqrt(mean(values .^ 2));
stats.peakAbsolute = max(abs(values));
end

function outputPath = localOutputPath(requestedPath, modelDirectory)
if strlength(requestedPath) == 0
    outputPath = string(fullfile(modelDirectory, "torsional_spring_experiment.mat"));
else
    outputPath = requestedPath;
    [folder, name, extension] = fileparts(outputPath);
    if strlength(extension) == 0
        extension = ".mat";
    elseif ~strcmpi(extension, ".mat")
        error("run_torsional_spring_experiment:InvalidOutputExtension", ...
            "outputPath must use the .mat extension.");
    end
    if strlength(folder) == 0
        folder = string(modelDirectory);
    end
    outputPath = string(fullfile(folder, name + extension));
end

outputFolder = string(fileparts(outputPath));
if ~isfolder(outputFolder)
    error("run_torsional_spring_experiment:MissingOutputFolder", ...
        "Output folder does not exist: %s", outputFolder);
end
end

function localPlotResults(results, targetRpm, maxLinearAngleDeg)
valid = results.Status == "ok";
if ~any(valid)
    warning("run_torsional_spring_experiment:NoValidRuns", ...
        "No successful simulations are available to plot.");
    return;
end

figure('Name', 'Torsional spring virtual experiment');
layout = tiledlayout(2, 2, 'TileSpacing', 'compact');
title(layout, 'Virtual torsional coupling sweep');

nexttile;
semilogx(results.K_NmPerRad(valid), results.DriveMagnetRpm(valid), '-o', ...
    results.K_NmPerRad(valid), results.AgitatorMagnetRpm(valid), '-s');
yline(targetRpm, '--', 'Target');
grid on;
xlabel('K [N m/rad]');
ylabel('Steady speed [RPM]');
legend('Drive magnet', 'Agitator magnet', 'Location', 'best');

nexttile;
semilogx(results.K_NmPerRad(valid), results.RunAnglePeakDeg(valid), '-o', ...
    results.K_NmPerRad(valid), results.SteadyAnglePeakDeg(valid), '-s');
yline(maxLinearAngleDeg, '--', 'Linear-region limit');
grid on;
xlabel('K [N m/rad]');
ylabel('|Relative angle| [deg]');
legend('Whole-run peak', 'Steady peak', 'Location', 'best');

nexttile;
loglog(results.K_NmPerRad(valid), results.RunTorquePeakMilliNm(valid), '-o', ...
    results.K_NmPerRad(valid), results.SteadyTorqueRmsMilliNm(valid), '-s');
grid on;
xlabel('K [N m/rad]');
ylabel('Spring torque [mN m]');
legend('Whole-run peak', 'Steady RMS', 'Location', 'best');

nexttile;
loglog(results.K_NmPerRad(valid), results.NaturalFrequencyHz(valid), '-o');
grid on;
xlabel('K [N m/rad]');
ylabel('Undamped relative-mode frequency [Hz]');
end
