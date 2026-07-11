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
        if (matFile["esc_validation_v1"]?.Value is not IStructureArray payload)
        {
            throw new InvalidDataException("MAT file must contain the esc_validation_v1 structure.");
        }

        int[] index = new int[payload.Dimensions.Length];
        uint schemaVersion = ReadSingleUInt32(payload, "schema_version", index);
        if (schemaVersion != 1)
        {
            throw new InvalidDataException($"Unsupported validation schema version {schemaVersion}.");
        }

        string manifestJson = ReadString(payload, "manifest_json", index);
        ValidationManifest manifest = ParseManifest(manifestJson, schemaVersion);
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
        uint previousSequence = 0;
        for (int i = 0; i < count; i++)
        {
            if (!double.IsFinite(simulationTimes[i]) || simulationTimes[i] < previousTime)
            {
                throw new InvalidDataException("simulation_time_s must be finite and monotonic.");
            }

            if (sequences[i] == 0 || (i > 0 && sequences[i] <= previousSequence))
            {
                throw new InvalidDataException("source_sequence must be strictly increasing and nonzero.");
            }

            if (enabled[i] > 1)
            {
                throw new InvalidDataException("enable must contain only 0 or 1.");
            }

            samples.Add(new ValidationInputSample(sequences[i], simulationTimes[i], speeds[i], enabled[i] != 0, targets[i], expected[i]));
            previousTime = simulationTimes[i];
            previousSequence = sequences[i];
        }

        return new ImportedValidationVector(sourceRunId, manifest, samples, sourcePath);
    }

    private static ValidationManifest ParseManifest(string json, uint schemaVersion)
    {
        using JsonDocument document = JsonDocument.Parse(json);
        JsonElement root = document.RootElement;
        JsonElement config = root.GetProperty("referenceConfig");
        return new ValidationManifest(
            schemaVersion,
            root.GetProperty("experimentName").GetString() ?? "Unnamed validation",
            root.GetProperty("description").GetString() ?? string.Empty,
            root.GetProperty("createdAtUtc").GetString() ?? string.Empty,
            root.GetProperty("samplePeriodUs").GetUInt64(),
            root.GetProperty("targetRpm").GetUInt16(),
            new ValidationReferenceConfig(
                config.GetProperty("kp").GetDouble(),
                config.GetProperty("ki").GetDouble(),
                config.GetProperty("kd").GetDouble(),
                config.GetProperty("pwmFrequency").GetInt32(),
                config.GetProperty("polePairs").GetInt32(),
                config.GetProperty("pwmArr").GetInt32(),
                config.GetProperty("dt").GetDouble()));
    }

    private static IArray Field(IStructureArray payload, string name, int[] index)
    {
        try
        {
            return payload[name, index];
        }
        catch (Exception exception) when (exception is KeyNotFoundException or IndexOutOfRangeException)
        {
            throw new InvalidDataException($"esc_validation_v1 is missing field '{name}'.", exception);
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
