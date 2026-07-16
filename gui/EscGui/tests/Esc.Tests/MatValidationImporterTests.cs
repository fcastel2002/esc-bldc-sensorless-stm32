using System.Text.Json;
using Esc.Bridge;
using MatFileHandler;

namespace Esc.Tests;

public sealed class MatValidationImporterTests
{
    [Fact]
    public void ImportsFlatValidationContract()
    {
        string path = Path.Combine(Path.GetTempPath(), $"esc-validation-{Guid.NewGuid():N}.mat");
        try
        {
            WriteVector(path);

            ImportedValidationVector vector = new MatValidationImporter().Import(path);

            Assert.Equal(77u, vector.SourceRunId);
            Assert.Equal("MAT import", vector.Manifest.ExperimentName);
            Assert.Equal(0.04, vector.Manifest.StopTimeSeconds);
            Assert.Equal((ulong)20_000, vector.Manifest.SamplePeriodUs);
            Assert.Equal(0.75, vector.Manifest.ReferenceConfig.Kp);
            Assert.Equal(2, vector.Samples.Count);
            Assert.Equal(2u, vector.Samples[1].SourceSequence);
            Assert.Equal((ushort)321, vector.Samples[1].ExpectedPwm);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void DerivesDurationForLegacyManifest()
    {
        string path = Path.Combine(Path.GetTempPath(), $"esc-validation-{Guid.NewGuid():N}.mat");
        try
        {
            WriteVector(path, includeStopTime: false);

            ImportedValidationVector vector = new MatValidationImporter().Import(path);

            Assert.Equal(0.04, vector.Manifest.StopTimeSeconds);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void RejectsGainOutsideProtocolResolution()
    {
        string path = Path.Combine(Path.GetTempPath(), $"esc-validation-{Guid.NewGuid():N}.mat");
        try
        {
            WriteVector(path, kp: 0.755);

            InvalidDataException exception = Assert.Throws<InvalidDataException>(() =>
                new MatValidationImporter().Import(path));
            Assert.Contains("0.01 resolution", exception.Message);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void ImportsLegacyV1ContractWithoutApplyingV2IntervalRules()
    {
        string path = Path.Combine(Path.GetTempPath(), $"esc-validation-{Guid.NewGuid():N}.mat");
        try
        {
            WriteVector(path, schemaVersion: 1, stopTimeSeconds: 0.02);

            ImportedValidationVector vector = new MatValidationImporter().Import(path);

            Assert.Equal(1u, vector.Manifest.SchemaVersion);
            Assert.Equal(0.02, vector.Manifest.StopTimeSeconds);
            Assert.Equal(2, vector.Samples.Count);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void RejectsZeroSamplePeriod()
    {
        AssertInvalidVector(
            path => WriteVector(path, samplePeriodUs: 0),
            "samplePeriodUs must be positive");
    }

    [Fact]
    public void RejectsControllerDtThatIsNotRepresentableInMicroseconds()
    {
        AssertInvalidVector(
            path => WriteVector(path, controllerDt: 0.0020005),
            "whole number of microseconds");
    }

    [Fact]
    public void RejectsSamplePeriodNotDivisibleByControllerDt()
    {
        AssertInvalidVector(
            path => WriteVector(path, samplePeriodUs: 20_001),
            "exactly divisible");
    }

    [Fact]
    public void RejectsTimesThatAreNotStrictlyIncreasing()
    {
        AssertInvalidVector(
            path => WriteVector(path, simulationTimes: [0.0, 0.0]),
            "strictly increasing");
    }

    [Fact]
    public void RejectsTimesOutsideTheSampleGrid()
    {
        AssertInvalidVector(
            path => WriteVector(path, simulationTimes: [0.0, 0.021]),
            "samplePeriodUs grid");
    }

    [Fact]
    public void RejectsMissingSampleInterval()
    {
        AssertInvalidVector(
            path => WriteVector(path, simulationTimes: [0.0, 0.04]),
            "advance by exactly one samplePeriodUs");
    }

    [Fact]
    public void RejectsStopTimeThatDoesNotEndAfterTheFinalInterval()
    {
        AssertInvalidVector(
            path => WriteVector(path, stopTimeSeconds: 0.02),
            "end of the final complete sample interval");
    }

    [Fact]
    public void RejectsTargetThatDiffersFromManifest()
    {
        AssertInvalidVector(
            path => WriteVector(path, targets: [(ushort)1000, (ushort)1001]),
            "constant and equal");
    }

    [Fact]
    public void RejectsDisabledDeterministicSample()
    {
        AssertInvalidVector(
            path => WriteVector(path, enabled: [(byte)1, (byte)0]),
            "true for every deterministic v2 sample");
    }

    [Fact]
    public void RejectsExpectedPwmAboveManifestArr()
    {
        AssertInvalidVector(
            path => WriteVector(path, expected: [(ushort)300, (ushort)2001]),
            "cannot exceed");
    }

    private static void AssertInvalidVector(Action<string> write, string expectedMessage)
    {
        string path = Path.Combine(Path.GetTempPath(), $"esc-validation-{Guid.NewGuid():N}.mat");
        try
        {
            write(path);

            InvalidDataException exception = Assert.Throws<InvalidDataException>(() =>
                new MatValidationImporter().Import(path));
            Assert.Contains(expectedMessage, exception.Message);
        }
        finally
        {
            File.Delete(path);
        }
    }

    private static void WriteVector(
        string path,
        bool includeStopTime = true,
        double kp = 0.75,
        ulong samplePeriodUs = 20_000,
        double controllerDt = 0.002,
        double stopTimeSeconds = 0.04,
        double[]? simulationTimes = null,
        byte[]? enabled = null,
        ushort[]? targets = null,
        ushort[]? expected = null,
        uint schemaVersion = 2)
    {
        simulationTimes ??= [0.0, 0.02];
        enabled ??= [(byte)1, (byte)1];
        targets ??= [(ushort)1000, (ushort)1000];
        expected ??= [(ushort)300, (ushort)321];
        var builder = new DataBuilder();
        IStructureArray payload = builder.NewStructureArray(
            ["schema_version", "manifest_json", "simulation_time_s", "run_id", "source_sequence", "speed_rpm", "enable", "target_rpm", "expected_pwm"],
            [1, 1]);
        int[] index = [0, 0];
        var manifestValues = new Dictionary<string, object?>
        {
            ["schemaVersion"] = schemaVersion,
            ["experimentName"] = "MAT import",
            ["description"] = "test",
            ["createdAtUtc"] = "2026-01-01T00:00:00.000+00:00",
            ["samplePeriodUs"] = samplePeriodUs,
            ["targetRpm"] = 1000,
            ["referenceConfig"] = new { kp, ki = 1.35, kd = 0.0, pwmFrequency = 18000, polePairs = 2, pwmArr = 2000, dt = controllerDt }
        };
        if (includeStopTime)
        {
            manifestValues["stopTimeSeconds"] = stopTimeSeconds;
        }
        string manifest = JsonSerializer.Serialize(manifestValues);
        payload["schema_version", index] = builder.NewArray([schemaVersion], [1, 1]);
        payload["manifest_json", index] = builder.NewCharArray(manifest);
        payload["simulation_time_s", index] = builder.NewArray(simulationTimes, [2, 1]);
        payload["run_id", index] = builder.NewArray([77u, 77u], [2, 1]);
        payload["source_sequence", index] = builder.NewArray([1u, 2u], [2, 1]);
        payload["speed_rpm", index] = builder.NewArray([(ushort)1000, (ushort)1100], [2, 1]);
        payload["enable", index] = builder.NewArray(enabled, [2, 1]);
        payload["target_rpm", index] = builder.NewArray(targets, [2, 1]);
        payload["expected_pwm", index] = builder.NewArray(expected, [2, 1]);

        IMatFile file = builder.NewFile([
            builder.NewVariable($"esc_validation_v{schemaVersion}", payload, false)
        ]);
        using FileStream stream = File.Create(path);
        new MatFileWriter(stream).Write(file);
    }
}
