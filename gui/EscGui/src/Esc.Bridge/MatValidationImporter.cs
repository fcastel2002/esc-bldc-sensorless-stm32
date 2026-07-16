using System.Text.Json;
using MatFileHandler;

namespace Esc.Bridge;

public sealed class MatValidationImporter
{
    public ImportedValidationVector Import(string sourcePath)
    {
        if (!File.Exists(sourcePath))
        {
            throw new FileNotFoundException("Validation MAT file was not found.", sourcePath);
        }

        using FileStream stream = File.OpenRead(sourcePath);
        IMatFile matFile = new MatFileReader(stream).Read();
        IStructureArray payload;
        uint expectedSchemaVersion;
        if (ReadOptionalStructure(matFile, "esc_validation_v2") is IStructureArray v2Payload)
        {
            payload = v2Payload;
            expectedSchemaVersion = 2;
        }
        else if (ReadOptionalStructure(matFile, "esc_validation_v1") is IStructureArray v1Payload)
        {
            payload = v1Payload;
            expectedSchemaVersion = 1;
        }
        else
        {
            throw new InvalidDataException("MAT file must contain an esc_validation_v1 or esc_validation_v2 structure.");
        }

        int[] index = new int[payload.Dimensions.Length];
        uint schemaVersion = ReadSingleUInt32(payload, "schema_version", index);
        if (schemaVersion != expectedSchemaVersion)
        {
            throw new InvalidDataException(
                $"Validation structure v{expectedSchemaVersion} declares schema version {schemaVersion}.");
        }

        string manifestJson = ReadString(payload, "manifest_json", index);
        ValidationManifest manifest = ParseManifest(manifestJson, schemaVersion);
        if (manifest.SamplePeriodUs == 0)
        {
            throw new InvalidDataException("manifest samplePeriodUs must be positive.");
        }

        if (schemaVersion == 2)
        {
            double controllerDtUsValue = manifest.ReferenceConfig.Dt * 1_000_000d;
            double roundedControllerDtUs = Math.Round(controllerDtUsValue);
            if (!double.IsFinite(controllerDtUsValue) || roundedControllerDtUs < 1 ||
                roundedControllerDtUs > uint.MaxValue || Math.Abs(controllerDtUsValue - roundedControllerDtUs) > 1e-6)
            {
                throw new InvalidDataException("manifest referenceConfig.dt must be a positive whole number of microseconds.");
            }

            ulong controllerDtUs = (ulong)roundedControllerDtUs;
            if (manifest.SamplePeriodUs % controllerDtUs != 0)
            {
                throw new InvalidDataException("manifest samplePeriodUs must be exactly divisible by referenceConfig.dt in microseconds.");
            }
        }

        double[] simulationTimes = ReadDoubleVector(payload, "simulation_time_s", index);
        uint[] runIds = ReadUInt32Vector(payload, "run_id", index);
        uint[] sequences = ReadUInt32Vector(payload, "source_sequence", index);
        ushort[] speeds = ReadUInt16Vector(payload, "speed_rpm", index);
        byte[] enabled = ReadByteVector(payload, "enable", index);
        ushort[] targets = ReadUInt16Vector(payload, "target_rpm", index);
        ushort[] expected = ReadUInt16Vector(payload, "expected_pwm", index);

        int count = simulationTimes.Length;
        if (count == 0 || runIds.Length != count || sequences.Length != count || speeds.Length != count ||
            enabled.Length != count || targets.Length != count || expected.Length != count)
        {
            throw new InvalidDataException("Validation vectors must be nonempty and have equal lengths.");
        }

        uint sourceRunId = runIds[0];
        if (sourceRunId == 0 || runIds.Any(id => id != sourceRunId))
        {
            throw new InvalidDataException("run_id must be nonzero and constant for the complete validation vector.");
        }

        var samples = new List<ValidationInputSample>(count);
        double previousTime = double.NegativeInfinity;
        ulong previousTimeUs = 0;
        uint previousSequence = 0;
        for (int i = 0; i < count; i++)
        {
            bool invalidTime = schemaVersion == 2
                ? !double.IsFinite(simulationTimes[i]) || simulationTimes[i] < 0 || simulationTimes[i] <= previousTime
                : !double.IsFinite(simulationTimes[i]) || simulationTimes[i] < previousTime;
            if (invalidTime)
            {
                throw new InvalidDataException(schemaVersion == 2
                    ? "simulation_time_s must be finite, nonnegative, and strictly increasing."
                    : "simulation_time_s must be finite and monotonic.");
            }

            ulong timeUs = 0;
            if (schemaVersion == 2)
            {
                double timeUsValue = simulationTimes[i] * 1_000_000d;
                double roundedTimeUs = Math.Round(timeUsValue);
                if (roundedTimeUs < 0 || roundedTimeUs > ulong.MaxValue ||
                    Math.Abs(timeUsValue - roundedTimeUs) > 1e-6 ||
                    (ulong)roundedTimeUs % manifest.SamplePeriodUs != 0)
                {
                    throw new InvalidDataException("simulation_time_s values must align exactly to the manifest samplePeriodUs grid.");
                }
                timeUs = (ulong)roundedTimeUs;
                if ((i == 0 && timeUs != 0) ||
                    (i > 0 && timeUs != checked(previousTimeUs + manifest.SamplePeriodUs)))
                {
                    throw new InvalidDataException(
                        "simulation_time_s must start at zero and advance by exactly one samplePeriodUs per row.");
                }
            }

            if (sequences[i] == 0 || (i > 0 && sequences[i] <= previousSequence))
            {
                throw new InvalidDataException("source_sequence must be strictly increasing and nonzero.");
            }

            if (schemaVersion == 2 && enabled[i] != 1)
            {
                throw new InvalidDataException("enable must be true for every deterministic v2 sample.");
            }
            if (schemaVersion == 1 && enabled[i] > 1)
            {
                throw new InvalidDataException("enable must contain only 0 or 1.");
            }
            if (schemaVersion == 2 && targets[i] != manifest.TargetRpm)
            {
                throw new InvalidDataException("target_rpm must be constant and equal to manifest targetRpm.");
            }
            if (schemaVersion == 2 && expected[i] > manifest.ReferenceConfig.PwmArr)
            {
                throw new InvalidDataException("expected_pwm cannot exceed manifest referenceConfig.pwmArr.");
            }

            samples.Add(new ValidationInputSample(sequences[i], simulationTimes[i], speeds[i], enabled[i] != 0, targets[i], expected[i]));
            previousTime = simulationTimes[i];
            previousTimeUs = timeUs;
            previousSequence = sequences[i];
        }

        double requiredStopTimeSeconds = schemaVersion == 2
            ? (ulong)count * manifest.SamplePeriodUs / 1_000_000d
            : Math.Max(simulationTimes[^1] - simulationTimes[0], manifest.SamplePeriodUs / 1_000_000d);
        if (double.IsNaN(manifest.StopTimeSeconds))
        {
            manifest = manifest with
            {
                StopTimeSeconds = requiredStopTimeSeconds
            };
        }
        else if (!double.IsFinite(manifest.StopTimeSeconds) || manifest.StopTimeSeconds <= 0)
        {
            throw new InvalidDataException("manifest stopTimeSeconds must be finite and positive.");
        }
        else if (schemaVersion == 2 && Math.Abs(manifest.StopTimeSeconds - requiredStopTimeSeconds) > 1e-9)
        {
            throw new InvalidDataException("manifest stopTimeSeconds must equal the end of the final complete sample interval.");
        }

        return new ImportedValidationVector(sourceRunId, manifest, samples, sourcePath);
    }

    private static ValidationManifest ParseManifest(string json, uint schemaVersion)
    {
        using JsonDocument document = JsonDocument.Parse(json);
        JsonElement root = document.RootElement;
        JsonElement config = root.GetProperty("referenceConfig");
        var referenceConfig = new ValidationReferenceConfig(
            config.GetProperty("kp").GetDouble(),
            config.GetProperty("ki").GetDouble(),
            config.GetProperty("kd").GetDouble(),
            config.GetProperty("pwmFrequency").GetInt32(),
            config.GetProperty("polePairs").GetInt32(),
            config.GetProperty("pwmArr").GetInt32(),
            config.GetProperty("dt").GetDouble(),
            GetOptionalUInt32(config, "timerHz", 180000),
            GetOptionalInt32(config, "speedMinPeriod", 14000),
            GetOptionalInt32(config, "speedMaxPeriod", 200),
            GetOptionalInt32(config, "minimumPwm", (int)Math.Floor(config.GetProperty("pwmArr").GetInt32() * 0.05)),
            GetOptionalInt32(config, "algorithmVersion", 1));
        ValidateReferenceConfig(referenceConfig);

        return new ValidationManifest(
            schemaVersion,
            root.GetProperty("experimentName").GetString() ?? "Unnamed validation",
            root.GetProperty("description").GetString() ?? string.Empty,
            root.GetProperty("createdAtUtc").GetString() ?? string.Empty,
            root.TryGetProperty("stopTimeSeconds", out JsonElement stopTime)
                ? stopTime.GetDouble()
                : double.NaN,
            root.GetProperty("samplePeriodUs").GetUInt64(),
            root.GetProperty("targetRpm").GetUInt16(),
            referenceConfig);
    }

    private static IStructureArray? ReadOptionalStructure(IMatFile matFile, string name)
    {
        try
        {
            return matFile[name]?.Value as IStructureArray;
        }
        catch (KeyNotFoundException)
        {
            return null;
        }
    }

    private static int GetOptionalInt32(JsonElement element, string name, int defaultValue) =>
        element.TryGetProperty(name, out JsonElement value) ? value.GetInt32() : defaultValue;

    private static uint GetOptionalUInt32(JsonElement element, string name, uint defaultValue) =>
        element.TryGetProperty(name, out JsonElement value) ? value.GetUInt32() : defaultValue;

    private static void ValidateReferenceConfig(ValidationReferenceConfig config)
    {
        ValidateGain(config.Kp, "kp");
        ValidateGain(config.Ki, "ki");
        ValidateGain(config.Kd, "kd");
        if (config.AlgorithmVersion is not (1 or 2))
        {
            throw new InvalidDataException($"Unsupported controller algorithm version {config.AlgorithmVersion}.");
        }
        if (config.AlgorithmVersion == 2 && config.Kd != 0)
        {
            throw new InvalidDataException("manifest referenceConfig.kd must be zero for RPM PI algorithm version 2.");
        }
    }

    private static void ValidateGain(double value, string name)
    {
        if (!double.IsFinite(value) || value is < 0 or > 10 || Math.Abs(value * 100 - Math.Round(value * 100)) > 1e-9)
        {
            throw new InvalidDataException($"manifest referenceConfig.{name} must be in [0, 10] with 0.01 resolution.");
        }
    }

    private static IArray Field(IStructureArray payload, string name, int[] index)
    {
        try
        {
            return payload[name, index];
        }
        catch (Exception exception) when (exception is KeyNotFoundException or IndexOutOfRangeException)
        {
            throw new InvalidDataException($"Validation structure is missing field '{name}'.", exception);
        }
    }

    private static string ReadString(IStructureArray payload, string name, int[] index)
    {
        return (Field(payload, name, index) as ICharArray)?.String
            ?? throw new InvalidDataException($"Field '{name}' must be a MATLAB char array.");
    }

    private static double[] ReadDoubleVector(IStructureArray payload, string name, int[] index) =>
        ReadData<double>(Field(payload, name, index), name);

    private static byte[] ReadByteVector(IStructureArray payload, string name, int[] index) =>
        ReadData<byte>(Field(payload, name, index), name);

    private static ushort[] ReadUInt16Vector(IStructureArray payload, string name, int[] index) =>
        ReadData<ushort>(Field(payload, name, index), name);

    private static uint[] ReadUInt32Vector(IStructureArray payload, string name, int[] index) =>
        ReadData<uint>(Field(payload, name, index), name);

    private static uint ReadSingleUInt32(IStructureArray payload, string name, int[] index)
    {
        uint[] values = ReadUInt32Vector(payload, name, index);
        return values.Length == 1
            ? values[0]
            : throw new InvalidDataException($"Field '{name}' must contain exactly one value.");
    }

    private static T[] ReadData<T>(IArray array, string name)
    {
        return (array as IArrayOf<T>)?.Data
            ?? throw new InvalidDataException($"Field '{name}' has an unexpected MATLAB numeric type.");
    }
}
