function [responseLog, responsePath] = replay_hil_validation(vectorPath, options)
%REPLAY_HIL_VALIDATION Replay a validation vector through the local UDP bridge.
%   REPLAY_HIL_VALIDATION(VECTORPATH) sends SETPOINT and HIL_START, then sends
%   each PILV row to 127.0.0.1:5055 from an ephemeral local UDP port. Each
%   response is recorded before the next input is sent. HIL_STOP is attempted
%   by cleanup on both successful and failed runs.

arguments
    vectorPath (1, 1) string
    options.minimumPeriodMs (1, 1) double {mustBeNonnegative} = 20
    options.responseTimeoutSeconds (1, 1) double {mustBePositive} = 1
    options.host (1, 1) string = "127.0.0.1"
    options.port (1, 1) double {mustBeInteger, mustBePositive} = 5055
    options.configureBridge (1, 1) logical = true
    options.apiBaseUrl (1, 1) string = "http://localhost:5187/api/bridge"
end

loaded = load(vectorPath, "validationVector", "manifest");
if ~isfield(loaded, "validationVector") || ~isfield(loaded, "manifest")
    error("replay_hil_validation:InvalidVector", ...
        "The vector MAT file must contain validationVector and manifest.");
end
validationVector = loaded.validationVector;
manifest = loaded.manifest;
localValidateVector(validationVector);

if isfield(manifest, "targetRpm")
    targetRpm = uint16(manifest.targetRpm);
else
    targetRpm = validationVector.TargetRpm(1);
end
[vectorDirectory, vectorName] = fileparts(vectorPath);
responsePath = fullfile(vectorDirectory, vectorName + "_responses.mat");

count = height(validationVector);
hostTimestamp = NaT(count, 1, "TimeZone", "local");
responseLog = table(validationVector.SimulationTime, validationVector.RunId, ...
    validationVector.SourceSequence, strings(count, 1), hostTimestamp, ...
    VariableNames=["SimulationTime", "RunId", "SourceSequence", ...
    "RawResponse", "HostTimestamp"]);

connection = udpport("datagram", "IPV4", "LocalHost", "127.0.0.1", "LocalPort", 0);
cleanup = onCleanup(@()localStop(connection, options.host, options.port, options.responseTimeoutSeconds));

try
    if options.configureBridge
        localConfigureBridge(manifest, options.apiBaseUrl, options.responseTimeoutSeconds);
    end
    localRequireOk(localRequest(connection, sprintf("SETPOINT,0,%d", double(targetRpm)), ...
        options.host, options.port, options.responseTimeoutSeconds), "SETPOINT");
    localRequireOk(localRequest(connection, "HIL_START", options.host, options.port, ...
        options.responseTimeoutSeconds), "HIL_START");

    lastSend = tic;
    for index = 1:count
        remainingSeconds = options.minimumPeriodMs / 1000 - toc(lastSend);
        if remainingSeconds > 0
            pause(remainingSeconds);
        end
        request = sprintf("PILV,%u,%u,%u,%u", double(validationVector.RunId(index)), ...
            double(validationVector.SourceSequence(index)), double(validationVector.SpeedRpm(index)), ...
            double(validationVector.Enable(index)));
        responseLog.HostTimestamp(index) = datetime("now", "TimeZone", "local");
        try
            response = localRequest(connection, request, options.host, options.port, ...
                options.responseTimeoutSeconds);
        catch exception
            responseLog.RawResponse(index) = "<timeout>";
            save(responsePath, "responseLog", "vectorPath", "manifest");
            rethrow(exception);
        end
        responseLog.RawResponse(index) = response;
        localRequireOk(response, sprintf("PILV source sequence %u", ...
            double(validationVector.SourceSequence(index))));
        lastSend = tic;
    end
    save(responsePath, "responseLog", "vectorPath", "manifest");
catch exception
    save(responsePath, "responseLog", "vectorPath", "manifest");
    rethrow(exception);
end

clear cleanup
end

function localConfigureBridge(manifest, apiBaseUrl, timeoutSeconds)
if ~isfield(manifest, "referenceConfig")
    error("replay_hil_validation:InvalidManifest", "manifest.referenceConfig is required.");
end

config = manifest.referenceConfig;
webOptions = weboptions("MediaType", "application/json", "Timeout", timeoutSeconds);
webwrite(apiBaseUrl + "/mode", struct("Mode", 0), webOptions);
localPostConfig(apiBaseUrl, "KP", config.kp, webOptions);
localPostConfig(apiBaseUrl, "KI", config.ki, webOptions);
localPostConfig(apiBaseUrl, "KD", config.kd, webOptions);
localPostConfig(apiBaseUrl, "POLE_PAIRS", config.polePairs, webOptions);
localPostConfig(apiBaseUrl, "PWM_FREQ", config.pwmFrequency, webOptions);
webwrite(apiBaseUrl + "/mode", struct("Mode", 1), webOptions);
end

function localPostConfig(apiBaseUrl, parameter, value, webOptions)
result = webwrite(apiBaseUrl + "/config/" + parameter, struct("Value", double(value)), webOptions);
if isstruct(result) && isfield(result, "success") && ~result.success
    error("replay_hil_validation:ConfigRejected", ...
        "Bridge rejected %s: %s", parameter, string(result.message));
end
end

function localValidateVector(vector)
requiredNames = ["SimulationTime", "RunId", "SourceSequence", "SpeedRpm", "Enable", "TargetRpm", "ExpectedPwm"];
if ~istable(vector) || isempty(vector) || ~all(ismember(requiredNames, string(vector.Properties.VariableNames)))
    error("replay_hil_validation:InvalidVector", "validationVector is missing required columns.");
end
if any(vector.RunId == 0) || any(diff(double(vector.SourceSequence)) <= 0)
    error("replay_hil_validation:InvalidVector", ...
        "RunId must be nonzero and SourceSequence must strictly increase.");
end
end

function response = localRequest(connection, request, host, port, timeoutSeconds)
write(connection, uint8(char(request)), "uint8", char(host), port);
started = tic;
while toc(started) < timeoutSeconds
    if connection.NumDatagramsAvailable > 0
        bytes = read(connection, 1, "uint8");
        response = string(strtrim(char(bytes(:).')));
        return;
    end
    pause(0.001);
end
error("replay_hil_validation:ResponseTimeout", ...
    "Timed out after %.3f seconds waiting for %s.", timeoutSeconds, request);
end

function localRequireOk(response, command)
if startsWith(lower(response), "err,")
    error("replay_hil_validation:BridgeError", "%s failed: %s", command, response);
end
if ~startsWith(lower(response), "ok,")
    error("replay_hil_validation:InvalidResponse", "%s returned: %s", command, response);
end
end

function localStop(connection, host, port, timeoutSeconds)
try
    localRequest(connection, "HIL_STOP", host, port, timeoutSeconds);
catch
    % Cleanup must not hide the original replay failure.
end
end
