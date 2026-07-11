function [artifact, report, outputPath] = export_simulink_validation_run( ...
    modelName, stopTime, spec, config, outputPath)
%EXPORT_SIMULINK_VALIDATION_RUN Simulate a model and export declared logsout signals.
%   The model must enable signal logging and expose the names declared in SPEC.
%   The model is never saved or otherwise modified by this function.

arguments
    modelName (1, 1) string
    stopTime (1, 1) double {mustBeFinite, mustBePositive}
    spec (1, 1) struct
    config (1, 1) struct
    outputPath (1, 1) string = ""
end

wasLoaded = bdIsLoaded(modelName);
if ~wasLoaded
    load_system(modelName);
    cleanup = onCleanup(@()close_system(modelName, 0));
end

simulationInput = Simulink.SimulationInput(modelName);
simulationInput = simulationInput.setModelParameter( ...
    "StopTime", num2str(stopTime, 17));
simulationOutput = sim(simulationInput);

if ~ismember("logsout", string(simulationOutput.who()))
    error("export_simulink_validation_run:MissingLogsout", ...
        "Model '%s' must enable signal logging and provide logsout.", modelName);
end

logsout = simulationOutput.get("logsout");
[artifact, report, outputPath] = export_validation_from_logsout( ...
    logsout, modelName, stopTime, spec, config, outputPath);

if ~wasLoaded
    clear cleanup
end
end
