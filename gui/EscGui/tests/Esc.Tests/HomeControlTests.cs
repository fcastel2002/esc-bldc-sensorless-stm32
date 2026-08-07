namespace Esc.Tests;

public sealed class HomeControlTests
{
    [Fact]
    public void NumericControlValuesUseInvariantHtmlFormatting()
    {
        string homeMarkup = File.ReadAllText(
            Path.Combine(AppContext.BaseDirectory, "TestAssets", "Home.razor"));

        Assert.DoesNotContain("value=\"@ControlValue(control)\"", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("value=\"@FormatControlValue(control)\"", homeMarkup, StringComparison.Ordinal);
        Assert.Contains(
            "ControlValue(control).ToString(System.Globalization.CultureInfo.InvariantCulture)",
            homeMarkup,
            StringComparison.Ordinal);
    }

    [Fact]
    public void DashboardPanelsAreAccessiblePersistentAndKeepChartMounted()
    {
        string homeMarkup = File.ReadAllText(
            Path.Combine(AppContext.BaseDirectory, "TestAssets", "Home.razor"));

        Assert.Equal(5, Count(homeMarkup, "class=\"panel-toggle\""));
        Assert.Contains("aria-expanded=", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("aria-controls=", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("hidden=\"@IsPanelCollapsed(PanelTelemetry)\"", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("<SpeedChart", homeMarkup, StringComparison.Ordinal);
        Assert.DoesNotContain("@if (!IsPanelCollapsed(PanelTelemetry))", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("localStorage.setItem", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("esc.dashboard.panels.v1", homeMarkup, StringComparison.Ordinal);
    }

    [Fact]
    public void DashboardKeepsEmergencyStopOutsideCollapsiblePowerPanel()
    {
        string homeMarkup = File.ReadAllText(
            Path.Combine(AppContext.BaseDirectory, "TestAssets", "Home.razor"));
        int topbarEstop = homeMarkup.IndexOf("class=\"topbar-estop\"", StringComparison.Ordinal);
        int powerPanel = homeMarkup.IndexOf("power-command-body", StringComparison.Ordinal);

        Assert.True(topbarEstop >= 0 && topbarEstop < powerPanel);
        Assert.Contains("disabled=\"@(_estopBusy || !CanEmergencyStop)\"", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("aria-live=\"polite\"", homeMarkup, StringComparison.Ordinal);
    }

    [Fact]
    public void SineDraftIsNotOverwrittenByPeriodicSnapshots()
    {
        string homeMarkup = File.ReadAllText(
            Path.Combine(AppContext.BaseDirectory, "TestAssets", "Home.razor"));

        Assert.DoesNotContain(
            "Snapshot = Bridge.Snapshot;\n            LoadSineValuesFromSnapshot();",
            homeMarkup,
            StringComparison.Ordinal);
        Assert.Contains("requested == _lastSineRequestedSeen || _sineDraftDirty", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("SynchronizeSineDraftFromSnapshot(force: true)", homeMarkup, StringComparison.Ordinal);
    }

    [Fact]
    public void SineInteractionModesUseExpectedAutomaticEventsAndPersistence()
    {
        string homeMarkup = File.ReadAllText(
            Path.Combine(AppContext.BaseDirectory, "TestAssets", "Home.razor"));

        Assert.Contains(">MANUAL</button>", homeMarkup, StringComparison.Ordinal);
        Assert.Contains(">DINAMICO</button>", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("esc.dashboard.sine-mode.v1", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("@oninput=\"UpdateSineFrequency\"", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("@oninput=\"UpdateSineAmplitudeDraft\"", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("@onchange=\"CommitSineAmplitude\"", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("Bridge.UpdateSineDriveAsync", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("_pendingDynamicSine", homeMarkup, StringComparison.Ordinal);
    }

    private static int Count(string source, string value)
    {
        int count = 0;
        int start = 0;
        while ((start = source.IndexOf(value, start, StringComparison.Ordinal)) >= 0)
        {
            count++;
            start += value.Length;
        }
        return count;
    }
}
