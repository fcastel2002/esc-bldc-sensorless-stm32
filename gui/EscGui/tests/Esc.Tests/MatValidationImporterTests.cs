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
            Assert.Equal(0.02, vector.Manifest.StopTimeSeconds);
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

            Assert.Equal(0.02, vector.Manifest.StopTimeSeconds);
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

    private static void WriteVector(string path, bool includeStopTime = true, double kp = 0.75)
    {
        var builder = new DataBuilder();
        IStructureArray payload = builder.NewStructureArray(
            ["schema_version", "manifest_json", "simulation_time_s", "run_id", "source_sequence", "speed_rpm", "enable", "target_rpm", "expected_pwm"],
            [1, 1]);
        int[] index = [0, 0];
        var manifestValues = new Dictionary<string, object?>
        {
            ["schemaVersion"] = 1,
            ["experimentName"] = "MAT import",
            ["description"] = "test",
            ["createdAtUtc"] = "2026-01-01T00:00:00.000+00:00",
            ["samplePeriodUs"] = 20_000,
            ["targetRpm"] = 1000,
            ["referenceConfig"] = new { kp, ki = 1.35, kd = 0.0, pwmFrequency = 18000, polePairs = 2, pwmArr = 2000, dt = 0.002 }
        };
        if (includeStopTime)
        {
            manifestValues["stopTimeSeconds"] = 0.02;
        }
        string manifest = JsonSerializer.Serialize(manifestValues);
        payload["schema_version", index] = builder.NewArray([1u], [1, 1]);
        payload["manifest_json", index] = builder.NewCharArray(manifest);
        payload["simulation_time_s", index] = builder.NewArray([0.0, 0.02], [2, 1]);
        payload["run_id", index] = builder.NewArray([77u, 77u], [2, 1]);
        payload["source_sequence", index] = builder.NewArray([1u, 2u], [2, 1]);
        payload["speed_rpm", index] = builder.NewArray([(ushort)1000, (ushort)1100], [2, 1]);
        payload["enable", index] = builder.NewArray([(byte)1, (byte)1], [2, 1]);
        payload["target_rpm", index] = builder.NewArray([(ushort)1000, (ushort)1000], [2, 1]);
        payload["expected_pwm", index] = builder.NewArray([(ushort)300, (ushort)321], [2, 1]);

        IMatFile file = builder.NewFile([builder.NewVariable("esc_validation_v1", payload, false)]);
        using FileStream stream = File.Create(path);
        new MatFileWriter(stream).Write(file);
    }
}
