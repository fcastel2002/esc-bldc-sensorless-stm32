function tests = test_export_validation_from_logsout
%TEST_EXPORT_VALIDATION_FROM_LOGSOUT Verify the model-independent MAT exporter.

tests = functiontests(localfunctions);
end

function testExportsDeclaredSignalsAndGeneratedRunId(testCase)
logsout = localLogsout();
spec = localSpec();
config = localConfig();
outputPath = string(tempname) + ".mat";
cleanup = onCleanup(@()localDelete(outputPath));

[artifact, report] = export_validation_from_logsout( ...
    logsout, "fixture_model", 0.004, spec, config, outputPath);
loaded = load(outputPath, "esc_validation_v1", "experimentSignals");

verifyNotEqual(testCase, artifact.manifest.runId, uint32(0));
verifyEqual(testCase, report.sampleCount, 3);
verifyEqual(testCase, loaded.esc_validation_v1.speed_rpm, uint16([100; 100; 200]));
verifyEqual(testCase, loaded.esc_validation_v1.expected_pwm, uint16([300; 310; 320]));
verifyEqual(testCase, numel(loaded.experimentSignals), 1);
verifyEqual(testCase, loaded.experimentSignals.name, "motor_rpm");
verifyEqual(testCase, loaded.experimentSignals.unit, "rpm");
end

function testRejectsUndeclaredRequiredSignal(testCase)
logsout = localLogsout();
spec = localSpec();
spec.validation.enable = "missing_enable";
config = localConfig();
outputPath = string(tempname) + ".mat";
cleanup = onCleanup(@()localDelete(outputPath));

verifyError(testCase, @()export_validation_from_logsout( ...
    logsout, "fixture_model", 0.004, spec, config, outputPath), ...
    "export_validation_from_logsout:MissingSignal");
end

function logsout = localLogsout()
logsout = Simulink.SimulationData.Dataset;
logsout = logsout.addElement(localSignal("validation_pwm_counts", [0; 0.002; 0.004], [300; 310; 320]));
logsout = logsout.addElement(localSignal("validation_speed_rpm", [0; 0.004], [100; 200]));
logsout = logsout.addElement(localSignal("validation_enable", [0; 0.004], [1; 1]));
logsout = logsout.addElement(localSignal("validation_target_rpm", [0; 0.004], [1000; 1000]));
logsout = logsout.addElement(localSignal("motor_rpm", [0; 0.002; 0.004], [0; 20; 40]));
logsout = logsout.addElement(localSignal("ignored_signal", [0; 0.002; 0.004], [1; 2; 3]));
end

function signal = localSignal(name, time, values)
signal = Simulink.SimulationData.Signal;
signal.Name = name;
signal.Values = timeseries(values, time);
end

function spec = localSpec()
spec = struct( ...
    "validation", struct( ...
        "speedRpm", "validation_speed_rpm", ...
        "expectedPwm", "validation_pwm_counts", ...
        "enable", "validation_enable", ...
        "targetRpm", "validation_target_rpm"), ...
    "extraSignals", struct("name", "motor_rpm", "unit", "rpm"));
end

function config = localConfig()
config = struct( ...
    "experimentName", "Fixture export", ...
    "description", "logsout exporter test", ...
    "controllerPeriodSeconds", 0.002, ...
    "targetRpm", 1000, ...
    "kp", 0.75, ...
    "ki", 1.35, ...
    "kd", 0, ...
    "pwmFrequency", 18000, ...
    "pwmArr", 2000, ...
    "polePairs", 2);
end

function localDelete(path)
if isfile(path)
    delete(path);
end
end
