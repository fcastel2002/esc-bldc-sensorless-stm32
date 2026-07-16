using Esc.Bridge;
using Microsoft.Data.Sqlite;

namespace Esc.Tests;

public sealed class ValidationRunStoreTests
{
    [Fact]
    public async Task PersistsRunAndDeletesDatabaseRowsAndArtifact()
    {
        string root = Path.Combine(Path.GetTempPath(), $"esc-runs-{Guid.NewGuid():N}");
        Directory.CreateDirectory(root);
        string artifact = Path.Combine(root, "input.mat");
        await File.WriteAllTextAsync(artifact, "validation artifact");
        try
        {
            var manifest = new ValidationManifest(1, "Store test", "persistent", "2026-01-01T00:00:00Z", 0.02, 20_000, 1000,
                new ValidationReferenceConfig(0.75, 1.35, 0, 18000, 2, 2000, 0.002));
            var vector = new ImportedValidationVector(44, manifest,
                [new ValidationInputSample(1, 0, 900, true, 1000, 300)], artifact);
            var store = new ValidationRunStore(root);

            ValidationRunSummary created = await store.CreateAsync(vector, new ValidationRunOptions(2, 100, 0, 3, 500));
            Assert.True(await store.UpdateMetadataAsync(created.Id, "Renamed run", "Edited locally"));
            Assert.False(await store.UpdateMetadataAsync(Guid.NewGuid(), "Missing", ""));
            await store.MarkRunningAsync(created.Id);
            await store.UpdateSampleAsync(created.Id, new ValidationSampleResult(1, 0, 900, true, 1000, 300, 301,
                ValidationSampleStatus.Passed, 4.5, 42, 7, 44, 1, 1));
            await store.CompleteAsync(created.Id, ValidationRunStatus.Completed, null);

            ValidationRunDetail detail = (await store.GetAsync(created.Id))!;

            Assert.Equal(ValidationRunStatus.Completed, detail.Summary.Status);
            Assert.Equal("Renamed run", detail.Summary.ExperimentName);
            Assert.Equal("Edited locally", detail.Summary.Description);
            Assert.Equal("Store test", detail.Manifest.ExperimentName);
            Assert.Equal("persistent", detail.Manifest.Description);
            Assert.Equal(0.02, detail.Manifest.StopTimeSeconds);
            Assert.Equal(1, detail.Summary.PassedCount);
            Assert.Equal((ushort)500, detail.Summary.HilInputTimeoutMs);
            Assert.Equal<ushort?>((ushort)301, detail.Samples[0].ActualPwm);
            Assert.Equal((ushort)300, detail.Samples[0].ExpectedPwm);
            Assert.Equal(1, detail.Samples[0].AbsoluteError);
            Assert.Equal(ValidationSampleStatus.Passed, detail.Samples[0].Status);

            string artifactDirectory = Path.Combine(root, "artifacts", created.Id.ToString("N"));
            Assert.True(Directory.Exists(artifactDirectory));
            Assert.True(await store.DeleteAsync(created.Id));
            Assert.Null(await store.GetAsync(created.Id));
            Assert.False(Directory.Exists(artifactDirectory));
            Assert.False(await store.DeleteAsync(created.Id));
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
