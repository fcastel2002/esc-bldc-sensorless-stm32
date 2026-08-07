namespace Esc.Tests;

public sealed class StartupControlTests
{
    [Fact]
    public void StartupPageExposesFivePhysicalRuntimeAndFlashControls()
    {
        string markup = File.ReadAllText(
            Path.Combine(AppContext.BaseDirectory, "TestAssets", "Startup.razor"));

        Assert.Contains("ConfigParam.StartupInitialAmplitude", markup, StringComparison.Ordinal);
        Assert.Contains("ConfigParam.StartupFinalAmplitude", markup, StringComparison.Ordinal);
        Assert.Contains("ConfigParam.StartupInitialFrequency", markup, StringComparison.Ordinal);
        Assert.Contains("ConfigParam.StartupFinalFrequency", markup, StringComparison.Ordinal);
        Assert.Contains("ConfigParam.StartupDuration", markup, StringComparison.Ordinal);
        Assert.Contains("Aplicar en RAM", markup, StringComparison.Ordinal);
        Assert.Contains("Guardar en flash", markup, StringComparison.Ordinal);
        Assert.Contains("CultureInfo.InvariantCulture", markup, StringComparison.Ordinal);
        Assert.Contains("startup-estop", markup, StringComparison.Ordinal);
    }
}
