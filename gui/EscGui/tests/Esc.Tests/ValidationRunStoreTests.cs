using Esc.Bridge;
using Microsoft.Data.Sqlite;

namespace Esc.Tests;

public sealed class ValidationRunStoreTests
{
    [Fact]
    public async Task PersistsImportedRunAndPerSampleResult()
    {
        string root = Path.Combine(Path.GetTempPath(), $"esc-runs-{Guid.NewGuid():N}");
        Directory.CreateDirectory(root);
        string artifact = Path.Combine(root, "input.mat");
        await File.WriteAllTextAsync(artifact, "validation artifact");
        try
        {
            var manifest = new ValidationManifest(1, "Store test", "persistent", "2026-01-01T00:00:00Z", 20_000, 1000,
                new ValidationReferenceConfig(0.75, 1.35, 0, 18000, 2, 2000, 0.002));
            var vector = new ImportedValidationVector(44, manifest,
                [new ValidationInputSample(1, 0, 900, true, 1000, 300)], artifact);
            var store = new ValidationRunStore(root);

            ValidationRunSummary created = await store.CreateAsync(vector, new ValidationRunOptions(2, 100, 0, 3, 500));
            await store.MarkRunningAsync(created.Id);
            await store.UpdateSampleAsync(created.Id, new ValidationSampleResult(1, 0, 900, true, 1000, 300, 301,
                ValidationSampleStatus.Passed, 4.5, 42, 7, 44, 1, 1));
            await store.CompleteAsync(created.Id, ValidationRunStatus.Completed, null);

            ValidationRunDetail detail = (await store.GetAsync(created.Id))!;

            Assert.Equal(ValidationRunStatus.Completed, detail.Summary.Status);
            Assert.Equal(1, detail.Summary.PassedCount);
            Assert.Equal((ushort)500, detail.Summary.HilInputTimeoutMs);
            Assert.Equal<ushort?>((ushort)301, detail.Samples[0].ActualPwm);
            Assert.Equal(ValidationSampleStatus.Passed, detail.Samples[0].Status);
        }
        finally
        {
            SqliteConnection.ClearAllPools();
            for (int attempt = 0; attempt < 5 && Directory.Exists(root); attempt++)
            {
                try
                {
                    Directory.Delete(root, recursive: true);
                }
                catch (IOException) when (attempt < 4)
                {
                    await Task.Delay(100);
                }
            }
        }
    }
}
