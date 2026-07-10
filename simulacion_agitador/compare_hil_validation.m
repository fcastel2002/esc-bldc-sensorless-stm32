function report = compare_hil_validation(vectorPath, responsePath, options)
%COMPARE_HIL_VALIDATION Compare reference PWM values with bridge HIL responses.
%   REPORT = COMPARE_HIL_VALIDATION(VECTORPATH, RESPONSEPATH) parses current
%   ok CSV responses: 15 legacy numeric values followed by validation fields.
%   It joins vector rows to MCU PWM by applied run ID and applied source
%   sequence, removing repeated cached outputs by output generation.

arguments
    vectorPath (1, 1) string
    responsePath (1, 1) string
    options.plot (1, 1) logical = false
end

vectorData = load(vectorPath, "validationVector", "manifest");
responseData = load(responsePath, "responseLog");
if ~isfield(vectorData, "validationVector") || ~isfield(vectorData, "manifest") || ...
        ~isfield(responseData, "responseLog")
    error("compare_hil_validation:InvalidFile", ...
        "Expected validationVector, manifest, and responseLog MAT variables.");
end
validationVector = vectorData.validationVector;
responseLog = responseData.responseLog;
localValidateInputs(validationVector, responseLog);

expectedRunId = uint32(validationVector.RunId(1));
if any(validationVector.RunId ~= expectedRunId)
    error("compare_hil_validation:InvalidVector", "All vector rows must use one run ID.");
end

parsed = localParseResponses(responseLog, expectedRunId);
candidateRows = find(parsed.appliedRunId == expectedRunId & parsed.outputGeneration ~= 0);
if ~isempty(candidateRows)
    [~, keep] = unique(parsed.outputGeneration(candidateRows), "last");
    candidateRows = candidateRows(sort(keep));
end

[expectedByGeneration, sourceByGeneration] = localReplayFirmware( ...
    validationVector, parsed, vectorData.manifest, expectedRunId);
count = numel(candidateRows);
sourceSequence = zeros(count, 1, "uint32");
expectedPwm = nan(count, 1);
rawMcuPwm = nan(count, 1);
outputGeneration = zeros(count, 1, "uint32");
status = repmat("missing", count, 1);
for index = 1:count
    row = candidateRows(index);
    generation = double(parsed.outputGeneration(row));
    sourceSequence(index) = parsed.appliedSourceSequence(row);
    rawMcuPwm(index) = parsed.pwmCommand(row);
    outputGeneration(index) = parsed.outputGeneration(row);
    if generation > numel(expectedByGeneration) || sourceSequence(index) == 0
        status(index) = "unreplayable";
        continue;
    end
    if sourceByGeneration(generation) ~= sourceSequence(index)
        status(index) = "provenance_mismatch";
        continue;
    end
    expectedPwm(index) = double(expectedByGeneration(generation));
    status(index) = "matched";
end

pwmError = rawMcuPwm - expectedPwm;
matched = status == "matched";
comparison = table(sourceSequence, expectedPwm, rawMcuPwm, pwmError, outputGeneration, status, ...
    VariableNames=["SourceSequence", "ExpectedPwm", "RawMcuPwm", "PwmError", "OutputGeneration", "Status"]);
absoluteError = abs(pwmError(matched));
matchedSequences = unique(sourceSequence(matched));
missingSequences = validationVector.SourceSequence(~ismember(validationVector.SourceSequence, matchedSequences));
coverage = struct("total", height(validationVector), "matched", numel(matchedSequences), ...
    "fraction", numel(matchedSequences) / height(validationVector));
report = struct( ...
    "coverage", coverage, ...
    "missingSequences", missingSequences, ...
    "maxAbsolutePwmError", localMetric(absoluteError, @max), ...
    "meanAbsolutePwmError", localMetric(absoluteError, @mean), ...
    "p95AbsolutePwmError", localPercentile95(absoluteError), ...
    "comparison", comparison);

if options.plot
    localPlot(comparison, matched);
end
end

function localValidateInputs(vector, responseLog)
requiredVector = ["RunId", "SourceSequence", "ExpectedPwm"];
requiredResponse = "RawResponse";
if ~istable(vector) || isempty(vector) || ...
        ~all(ismember(requiredVector, string(vector.Properties.VariableNames)))
    error("compare_hil_validation:InvalidVector", "validationVector is missing required columns.");
end
if ~istable(responseLog) || ~all(ismember(requiredResponse, string(responseLog.Properties.VariableNames)))
    error("compare_hil_validation:InvalidResponseLog", "responseLog must contain RawResponse.");
end
end

function parsed = localParseResponses(responseLog, expectedRunId)
count = height(responseLog);
parsed = struct( ...
    "pwmCommand", nan(count, 1), ...
    "inputRunId", zeros(count, 1, "uint32"), ...
    "acceptedRunId", zeros(count, 1, "uint32"), ...
    "acceptedSourceSequence", zeros(count, 1, "uint32"), ...
    "acceptedGeneration", zeros(count, 1, "uint32"), ...
    "appliedRunId", zeros(count, 1, "uint32"), ...
    "appliedSourceSequence", zeros(count, 1, "uint32"), ...
    "outputGeneration", zeros(count, 1, "uint32"));

for index = 1:count
    response = string(responseLog.RawResponse(index));
    if startsWith(lower(response), "err,") || response == "<timeout>"
        error("compare_hil_validation:ReplayFailure", ...
            "Response row %d reports a replay failure: %s", index, response);
    end
    values = localOkValues(response, index);
    parsed.pwmCommand(index) = localUInt(values(7), "PWM command", index, 65535);
    parsed.inputRunId(index) = uint32(localUInt(values(16), "input run ID", index, double(intmax("uint32"))));
    parsed.acceptedRunId(index) = uint32(localUInt(values(17), "accepted run ID", index, double(intmax("uint32"))));
    parsed.acceptedSourceSequence(index) = uint32(localUInt(values(18), "accepted source sequence", index, double(intmax("uint32"))));
    parsed.acceptedGeneration(index) = uint32(localUInt(values(19), "accepted generation", index, double(intmax("uint32"))));
    parsed.appliedRunId(index) = uint32(localUInt(values(20), "applied run ID", index, double(intmax("uint32"))));
    parsed.appliedSourceSequence(index) = uint32(localUInt(values(21), ...
        "applied source sequence", index, double(intmax("uint32"))));
    parsed.outputGeneration(index) = uint32(localUInt(values(22), ...
        "output generation", index, double(intmax("uint32"))));

    if parsed.inputRunId(index) ~= expectedRunId
        error("compare_hil_validation:WrongRunId", ...
            "Response row %d has input run ID %u, expected %u.", index, ...
            double(parsed.inputRunId(index)), double(expectedRunId));
    end
    if parsed.appliedRunId(index) ~= 0 && parsed.appliedRunId(index) ~= expectedRunId
        error("compare_hil_validation:WrongRunId", ...
            "Response row %d has applied run ID %u, expected %u.", index, ...
            double(parsed.appliedRunId(index)), double(expectedRunId));
    end
end
end

function [expectedPwm, sourceSequence] = localReplayFirmware(vector, parsed, manifest, runId)
if ~isfield(manifest, "referenceConfig") || ~isfield(manifest, "targetRpm")
    error("compare_hil_validation:InvalidManifest", ...
        "The vector manifest must contain referenceConfig and targetRpm.");
end

acceptedRows = find(parsed.acceptedRunId == runId & parsed.acceptedSourceSequence ~= 0);
if isempty(acceptedRows)
    error("compare_hil_validation:MissingAcceptedInputs", ...
        "No response contains accepted validation input provenance.");
end
[~, order] = sortrows([double(parsed.acceptedGeneration(acceptedRows)), ...
    double(parsed.acceptedSourceSequence(acceptedRows))], [1, 2]);
acceptedRows = acceptedRows(order);
maxGeneration = double(max(parsed.outputGeneration));
expectedPwm = zeros(maxGeneration, 1, "uint16");
sourceSequence = zeros(maxGeneration, 1, "uint32");
state = struct("integral", single(0), "previousError", int32(0), "diffSpeed", int32(0));
activeSpeed = uint16(0);
activeSequence = uint32(0);
eventIndex = 1;

for generation = 1:maxGeneration
    while eventIndex <= numel(acceptedRows) && ...
            parsed.acceptedGeneration(acceptedRows(eventIndex)) < uint32(generation)
        sequence = parsed.acceptedSourceSequence(acceptedRows(eventIndex));
        vectorIndex = find(vector.SourceSequence == sequence, 1);
        if isempty(vectorIndex)
            error("compare_hil_validation:UnknownSequence", ...
                "MCU accepted source sequence %u absent from validationVector.", double(sequence));
        end
        activeSpeed = vector.SpeedRpm(vectorIndex);
        activeSequence = sequence;
        eventIndex = eventIndex + 1;
    end
    [state, expectedPwm(generation)] = firmware_pi_reference_step( ...
        state, double(activeSpeed), double(manifest.targetRpm), manifest.referenceConfig);
    sourceSequence(generation) = activeSequence;
end
end

function values = localOkValues(response, index)
parts = split(strtrim(response), ",");
if numel(parts) < 26 || lower(parts(1)) ~= "ok"
    error("compare_hil_validation:MissingValidationExtension", ...
        "Response row %d is not an ok response with the validation extension.", index);
end
values = str2double(parts(2:end));
if numel(values) < 25 || any(~isfinite(values))
    error("compare_hil_validation:InvalidResponse", ...
        "Response row %d contains nonnumeric CSV fields.", index);
end
end

function value = localUInt(value, name, row, maximum)
if value < 0 || value > maximum || value ~= floor(value)
    error("compare_hil_validation:InvalidResponse", ...
        "Response row %d has invalid %s.", row, name);
end
end

function value = localMetric(values, operation)
if isempty(values)
    value = NaN;
else
    value = operation(values);
end
end

function value = localPercentile95(values)
if isempty(values)
    value = NaN;
    return;
end
values = sort(values(:));
position = 1 + 0.95 * (numel(values) - 1);
lowerIndex = floor(position);
upperIndex = ceil(position);
value = values(lowerIndex) + (position - lowerIndex) * ...
    (values(upperIndex) - values(lowerIndex));
end

function localPlot(comparison, matched)
figure("Name", "HIL validation PWM error", "NumberTitle", "off", ...
    "Position", [100, 100, 650, 280]);
if any(matched)
    plot(double(comparison.SourceSequence(matched)), comparison.PwmError(matched), "o-");
else
    plot(0, 0, "x");
end
grid on
xlabel("Source sequence")
ylabel("MCU PWM - expected PWM")
title("HIL validation PWM error")
end
