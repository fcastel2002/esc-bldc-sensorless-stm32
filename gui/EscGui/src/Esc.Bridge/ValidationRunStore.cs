using System.Globalization;
using System.Text.Json;
using Microsoft.Data.Sqlite;

namespace Esc.Bridge;

public sealed class ValidationRunStore
{
    private static readonly JsonSerializerOptions JsonOptions = new(JsonSerializerDefaults.Web);
    private readonly string _rootDirectory;
    private readonly string _databasePath;
    private readonly SemaphoreSlim _initializationLock = new(1, 1);
    private bool _initialized;

    public ValidationRunStore(string? rootDirectory = null)
    {
        _rootDirectory = rootDirectory ?? Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "EscGui", "validation-runs");
        _databasePath = Path.Combine(_rootDirectory, "runs.db");
    }

    public async Task<ValidationRunSummary> CreateAsync(ImportedValidationVector vector, ValidationRunOptions options, CancellationToken cancellationToken = default)
    {
        await EnsureInitializedAsync(cancellationToken).ConfigureAwait(false);
        Guid id = Guid.NewGuid();
        DateTimeOffset createdAt = DateTimeOffset.UtcNow;
        string artifactDirectory = Path.Combine(_rootDirectory, "artifacts", id.ToString("N"));
        Directory.CreateDirectory(artifactDirectory);
        string artifactPath = Path.Combine(artifactDirectory, Path.GetFileName(vector.SourcePath));
        File.Copy(vector.SourcePath, artifactPath, overwrite: false);

        await using SqliteConnection connection = OpenConnection();
        await connection.OpenAsync(cancellationToken).ConfigureAwait(false);
        await using SqliteTransaction transaction = (SqliteTransaction)await connection.BeginTransactionAsync(cancellationToken).ConfigureAwait(false);
        await ExecuteAsync(connection, transaction, """
            INSERT INTO validation_runs (
                id, source_run_id, experiment_name, description, status, created_at, manifest_json,
                artifact_path, sample_count, tolerance_counts, response_deadline_ms, warmup_samples, maximum_timeouts, hil_input_timeout_ms)
            VALUES ($id, $sourceRunId, $experimentName, $description, $status, $createdAt, $manifestJson,
                $artifactPath, $sampleCount, $toleranceCounts, $responseDeadlineMs, $warmupSamples, $maximumTimeouts, $hilInputTimeoutMs)
            """,
            cancellationToken,
            ("$id", id.ToString("N")),
            ("$sourceRunId", (long)vector.SourceRunId),
            ("$experimentName", vector.Manifest.ExperimentName),
            ("$description", vector.Manifest.Description),
            ("$status", (int)ValidationRunStatus.Ready),
            ("$createdAt", createdAt.ToString("O", CultureInfo.InvariantCulture)),
            ("$manifestJson", JsonSerializer.Serialize(vector.Manifest, JsonOptions)),
            ("$artifactPath", artifactPath),
            ("$sampleCount", vector.Samples.Count),
            ("$toleranceCounts", options.AbsoluteToleranceCounts),
            ("$responseDeadlineMs", options.ResponseDeadlineMs),
            ("$warmupSamples", options.WarmupSamples),
            ("$maximumTimeouts", options.MaximumTimeouts),
            ("$hilInputTimeoutMs", options.HilInputTimeoutMs)).ConfigureAwait(false);

        foreach (ValidationInputSample sample in vector.Samples)
        {
            await ExecuteAsync(connection, transaction, """
                INSERT INTO validation_samples (
                    run_id, source_sequence, simulation_time_s, speed_rpm, enable, target_rpm, expected_pwm, status)
                VALUES ($runId, $sequence, $simulationTime, $speedRpm, $enable, $targetRpm, $expectedPwm, $status)
                """,
                cancellationToken,
                ("$runId", id.ToString("N")),
                ("$sequence", (long)sample.SourceSequence),
                ("$simulationTime", sample.SimulationTimeSeconds),
                ("$speedRpm", sample.SpeedRpm),
                ("$enable", sample.Enable ? 1 : 0),
                ("$targetRpm", sample.TargetRpm),
                ("$expectedPwm", sample.ExpectedPwm),
                ("$status", (int)ValidationSampleStatus.Pending)).ConfigureAwait(false);
        }

        await transaction.CommitAsync(cancellationToken).ConfigureAwait(false);
        return new ValidationRunSummary(id, vector.SourceRunId, vector.Manifest.ExperimentName, vector.Manifest.Description, ValidationRunStatus.Ready,
            createdAt, null, null, vector.Samples.Count, 0, 0, 0, 0, options.AbsoluteToleranceCounts, options.ResponseDeadlineMs,
            options.WarmupSamples, options.MaximumTimeouts, options.HilInputTimeoutMs, null);
    }

    public async Task<IReadOnlyList<ValidationRunSummary>> ListAsync(CancellationToken cancellationToken = default)
    {
        await EnsureInitializedAsync(cancellationToken).ConfigureAwait(false);
        await using SqliteConnection connection = OpenConnection();
        await connection.OpenAsync(cancellationToken).ConfigureAwait(false);
        await using SqliteCommand command = connection.CreateCommand();
        command.CommandText = RunSelectSql + " ORDER BY r.created_at DESC";
        await using SqliteDataReader reader = await command.ExecuteReaderAsync(cancellationToken).ConfigureAwait(false);
        var runs = new List<ValidationRunSummary>();
        while (await reader.ReadAsync(cancellationToken).ConfigureAwait(false))
        {
            runs.Add(ReadSummary(reader));
        }

        return runs;
    }

    public async Task<ValidationRunDetail?> GetAsync(Guid id, CancellationToken cancellationToken = default)
    {
        await EnsureInitializedAsync(cancellationToken).ConfigureAwait(false);
        await using SqliteConnection connection = OpenConnection();
        await connection.OpenAsync(cancellationToken).ConfigureAwait(false);
        await using SqliteCommand runCommand = connection.CreateCommand();
        runCommand.CommandText = RunSelectSql + " WHERE r.id = $id";
        runCommand.Parameters.AddWithValue("$id", id.ToString("N"));
        await using SqliteDataReader runReader = await runCommand.ExecuteReaderAsync(cancellationToken).ConfigureAwait(false);
        if (!await runReader.ReadAsync(cancellationToken).ConfigureAwait(false))
        {
            return null;
        }

        ValidationRunSummary summary = ReadSummary(runReader);
        ValidationManifest manifest = JsonSerializer.Deserialize<ValidationManifest>(runReader.GetString(runReader.GetOrdinal("manifest_json")), JsonOptions)
            ?? throw new InvalidDataException("Stored validation manifest is invalid.");
        await runReader.DisposeAsync().ConfigureAwait(false);

        await using SqliteCommand samplesCommand = connection.CreateCommand();
        samplesCommand.CommandText = "SELECT * FROM validation_samples WHERE run_id = $id ORDER BY source_sequence";
        samplesCommand.Parameters.AddWithValue("$id", id.ToString("N"));
        await using SqliteDataReader sampleReader = await samplesCommand.ExecuteReaderAsync(cancellationToken).ConfigureAwait(false);
        var samples = new List<ValidationSampleResult>();
        while (await sampleReader.ReadAsync(cancellationToken).ConfigureAwait(false))
        {
            samples.Add(new ValidationSampleResult(
                (uint)sampleReader.GetInt64(sampleReader.GetOrdinal("source_sequence")),
                sampleReader.GetDouble(sampleReader.GetOrdinal("simulation_time_s")),
                (ushort)sampleReader.GetInt64(sampleReader.GetOrdinal("speed_rpm")),
                sampleReader.GetInt64(sampleReader.GetOrdinal("enable")) != 0,
                (ushort)sampleReader.GetInt64(sampleReader.GetOrdinal("target_rpm")),
                (ushort)sampleReader.GetInt64(sampleReader.GetOrdinal("expected_pwm")),
                ReadNullableUInt16(sampleReader, "actual_pwm"),
                (ValidationSampleStatus)sampleReader.GetInt32(sampleReader.GetOrdinal("status")),
                ReadNullableDouble(sampleReader, "round_trip_ms"),
                ReadNullableUInt32(sampleReader, "mcu_tick_ms"),
                ReadNullableUInt32(sampleReader, "output_generation"),
                ReadNullableUInt32(sampleReader, "applied_run_id"),
                ReadNullableUInt32(sampleReader, "applied_source_sequence"),
                ReadNullableInt32(sampleReader, "absolute_error")));
        }

        return new ValidationRunDetail(summary, manifest, samples);
    }

    public async Task MarkRunningAsync(Guid id, CancellationToken cancellationToken = default) =>
        await UpdateRunAsync(id, ValidationRunStatus.Running, null, cancellationToken).ConfigureAwait(false);

    public async Task CompleteAsync(Guid id, ValidationRunStatus status, string? failureReason, CancellationToken cancellationToken = default) =>
        await UpdateRunAsync(id, status, failureReason, cancellationToken).ConfigureAwait(false);

    public async Task UpdateSampleAsync(Guid runId, ValidationSampleResult result, CancellationToken cancellationToken = default)
    {
        await EnsureInitializedAsync(cancellationToken).ConfigureAwait(false);
        await using SqliteConnection connection = OpenConnection();
        await connection.OpenAsync(cancellationToken).ConfigureAwait(false);
        await ExecuteAsync(connection, null, """
            UPDATE validation_samples SET
                actual_pwm = $actualPwm, status = $status, round_trip_ms = $roundTripMs,
                mcu_tick_ms = $mcuTickMs, output_generation = $outputGeneration,
                applied_run_id = $appliedRunId, applied_source_sequence = $appliedSourceSequence,
                absolute_error = $absoluteError
            WHERE run_id = $runId AND source_sequence = $sequence
            """,
            cancellationToken,
            ("$actualPwm", result.ActualPwm),
            ("$status", (int)result.Status),
            ("$roundTripMs", result.RoundTripMs),
            ("$mcuTickMs", result.McuTickMs),
            ("$outputGeneration", result.OutputGeneration),
            ("$appliedRunId", result.AppliedRunId),
            ("$appliedSourceSequence", result.AppliedSourceSequence),
            ("$absoluteError", result.AbsoluteError),
            ("$runId", runId.ToString("N")),
            ("$sequence", (long)result.SourceSequence)).ConfigureAwait(false);
    }

    private async Task UpdateRunAsync(Guid id, ValidationRunStatus status, string? failureReason, CancellationToken cancellationToken)
    {
        await EnsureInitializedAsync(cancellationToken).ConfigureAwait(false);
        await using SqliteConnection connection = OpenConnection();
        await connection.OpenAsync(cancellationToken).ConfigureAwait(false);
        string timeColumn = status == ValidationRunStatus.Running ? "started_at" : "completed_at";
        await ExecuteAsync(connection, null, $"UPDATE validation_runs SET status = $status, {timeColumn} = $timestamp, failure_reason = $failureReason WHERE id = $id",
            cancellationToken,
            ("$status", (int)status),
            ("$timestamp", DateTimeOffset.UtcNow.ToString("O", CultureInfo.InvariantCulture)),
            ("$failureReason", failureReason),
            ("$id", id.ToString("N"))).ConfigureAwait(false);
    }

    private async Task EnsureInitializedAsync(CancellationToken cancellationToken)
    {
        if (_initialized)
        {
            return;
        }

        await _initializationLock.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            if (_initialized)
            {
                return;
            }

            Directory.CreateDirectory(_rootDirectory);
            await using SqliteConnection connection = OpenConnection();
            await connection.OpenAsync(cancellationToken).ConfigureAwait(false);
            await ExecuteAsync(connection, null, "PRAGMA journal_mode = WAL;", cancellationToken).ConfigureAwait(false);
            await ExecuteAsync(connection, null, """
                CREATE TABLE IF NOT EXISTS validation_runs (
                    id TEXT PRIMARY KEY,
                    source_run_id INTEGER NOT NULL,
                    experiment_name TEXT NOT NULL,
                    description TEXT NOT NULL,
                    status INTEGER NOT NULL,
                    created_at TEXT NOT NULL,
                    started_at TEXT NULL,
                    completed_at TEXT NULL,
                    manifest_json TEXT NOT NULL,
                    artifact_path TEXT NOT NULL,
                    sample_count INTEGER NOT NULL,
                    tolerance_counts INTEGER NOT NULL,
                    response_deadline_ms INTEGER NOT NULL,
                    warmup_samples INTEGER NOT NULL,
                    maximum_timeouts INTEGER NOT NULL,
                    hil_input_timeout_ms INTEGER NOT NULL,
                    failure_reason TEXT NULL
                );
                CREATE TABLE IF NOT EXISTS validation_samples (
                    run_id TEXT NOT NULL,
                    source_sequence INTEGER NOT NULL,
                    simulation_time_s REAL NOT NULL,
                    speed_rpm INTEGER NOT NULL,
                    enable INTEGER NOT NULL,
                    target_rpm INTEGER NOT NULL,
                    expected_pwm INTEGER NOT NULL,
                    actual_pwm INTEGER NULL,
                    status INTEGER NOT NULL,
                    round_trip_ms REAL NULL,
                    mcu_tick_ms INTEGER NULL,
                    output_generation INTEGER NULL,
                    applied_run_id INTEGER NULL,
                    applied_source_sequence INTEGER NULL,
                    absolute_error INTEGER NULL,
                    PRIMARY KEY (run_id, source_sequence),
                    FOREIGN KEY (run_id) REFERENCES validation_runs(id)
                );
                """, cancellationToken).ConfigureAwait(false);
            _initialized = true;
        }
        finally
        {
            _initializationLock.Release();
        }
    }

    private SqliteConnection OpenConnection() => new($"Data Source={_databasePath}");

    private static async Task ExecuteAsync(SqliteConnection connection, SqliteTransaction? transaction, string sql, CancellationToken cancellationToken, params (string Name, object? Value)[] parameters)
    {
        await using SqliteCommand command = connection.CreateCommand();
        command.Transaction = transaction;
        command.CommandText = sql;
        foreach ((string name, object? value) in parameters)
        {
            command.Parameters.AddWithValue(name, value ?? DBNull.Value);
        }

        await command.ExecuteNonQueryAsync(cancellationToken).ConfigureAwait(false);
    }

    private const string RunSelectSql = """
        SELECT r.*,
            (SELECT COUNT(*) FROM validation_samples s WHERE s.run_id = r.id AND s.status = 1) AS passed_count,
            (SELECT COUNT(*) FROM validation_samples s WHERE s.run_id = r.id AND s.status = 2) AS out_of_tolerance_count,
            (SELECT COUNT(*) FROM validation_samples s WHERE s.run_id = r.id AND s.status = 3) AS timeout_count,
            (SELECT COUNT(*) FROM validation_samples s WHERE s.run_id = r.id AND s.status = 4) AS mismatched_output_count
        FROM validation_runs r
        """;

    private static ValidationRunSummary ReadSummary(SqliteDataReader reader) => new(
        Guid.ParseExact(reader.GetString(reader.GetOrdinal("id")), "N"),
        (uint)reader.GetInt64(reader.GetOrdinal("source_run_id")),
        reader.GetString(reader.GetOrdinal("experiment_name")),
        reader.GetString(reader.GetOrdinal("description")),
        (ValidationRunStatus)reader.GetInt32(reader.GetOrdinal("status")),
        DateTimeOffset.Parse(reader.GetString(reader.GetOrdinal("created_at")), CultureInfo.InvariantCulture),
        ReadNullableDateTimeOffset(reader, "started_at"),
        ReadNullableDateTimeOffset(reader, "completed_at"),
        reader.GetInt32(reader.GetOrdinal("sample_count")),
        reader.GetInt32(reader.GetOrdinal("passed_count")),
        reader.GetInt32(reader.GetOrdinal("out_of_tolerance_count")),
        reader.GetInt32(reader.GetOrdinal("timeout_count")),
        reader.GetInt32(reader.GetOrdinal("mismatched_output_count")),
        (ushort)reader.GetInt64(reader.GetOrdinal("tolerance_counts")),
        reader.GetInt32(reader.GetOrdinal("response_deadline_ms")),
        reader.GetInt32(reader.GetOrdinal("warmup_samples")),
        reader.GetInt32(reader.GetOrdinal("maximum_timeouts")),
        (ushort)reader.GetInt64(reader.GetOrdinal("hil_input_timeout_ms")),
        ReadNullableString(reader, "failure_reason"));

    private static DateTimeOffset? ReadNullableDateTimeOffset(SqliteDataReader reader, string column) => reader.IsDBNull(reader.GetOrdinal(column)) ? null : DateTimeOffset.Parse(reader.GetString(reader.GetOrdinal(column)), CultureInfo.InvariantCulture);
    private static string? ReadNullableString(SqliteDataReader reader, string column) => reader.IsDBNull(reader.GetOrdinal(column)) ? null : reader.GetString(reader.GetOrdinal(column));
    private static ushort? ReadNullableUInt16(SqliteDataReader reader, string column) => reader.IsDBNull(reader.GetOrdinal(column)) ? null : (ushort)reader.GetInt64(reader.GetOrdinal(column));
    private static uint? ReadNullableUInt32(SqliteDataReader reader, string column) => reader.IsDBNull(reader.GetOrdinal(column)) ? null : (uint)reader.GetInt64(reader.GetOrdinal(column));
    private static int? ReadNullableInt32(SqliteDataReader reader, string column) => reader.IsDBNull(reader.GetOrdinal(column)) ? null : reader.GetInt32(reader.GetOrdinal(column));
    private static double? ReadNullableDouble(SqliteDataReader reader, string column) => reader.IsDBNull(reader.GetOrdinal(column)) ? null : reader.GetDouble(reader.GetOrdinal(column));
}
