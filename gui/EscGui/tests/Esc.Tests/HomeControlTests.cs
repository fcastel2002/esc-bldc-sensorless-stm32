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
    public void BemfBlankingControlRequiresConnectedIdleStateAndUsesGenericConfig()
    {
        string homeMarkup = File.ReadAllText(
            Path.Combine(AppContext.BaseDirectory, "TestAssets", "Home.razor"));
        string controlsJson = File.ReadAllText(
            Path.Combine(AppContext.BaseDirectory, "TestAssets", "controls.json"));
        using System.Text.Json.JsonDocument document = System.Text.Json.JsonDocument.Parse(controlsJson);
        System.Text.Json.JsonElement control = document.RootElement.EnumerateArray().Single(
            item => item.TryGetProperty("param", out System.Text.Json.JsonElement parameter) &&
                    parameter.GetString() == "BemfBlankingUs");

        Assert.Equal("Blanking BEMF", control.GetProperty("label").GetString());
        Assert.Equal("number", control.GetProperty("kind").GetString());
        Assert.Equal("set_config", control.GetProperty("command").GetString());
        Assert.Equal(0, control.GetProperty("min").GetDouble());
        Assert.Equal(200, control.GetProperty("max").GetDouble());
        Assert.Equal(5, control.GetProperty("step").GetDouble());
        Assert.Equal("us", control.GetProperty("unit").GetString());
        Assert.Equal(0, control.GetProperty("defaultValue").GetDouble());
        Assert.True(control.GetProperty("requiresIdle").GetBoolean());
        Assert.Contains(
            "if (control.RequiresIdle && Snapshot.Status?.AppState != 0)",
            homeMarkup,
            StringComparison.Ordinal);
        Assert.Contains(
            "control.Command is not (\"log_speed_start\" or \"log_speed_stop\") && Snapshot.Mode != ControlMode.GuiControl",
            homeMarkup,
            StringComparison.Ordinal);
        Assert.Contains("Bridge.SetConfigAsync(parameter, value)", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("disabled=\"@SaveControlDisabled(control)\"", homeMarkup, StringComparison.Ordinal);
        Assert.Contains(
            "ControlDisabled(control) || Snapshot.Status?.AppState != 0",
            homeMarkup,
            StringComparison.Ordinal);
    }

    [Fact]
    public void DashboardPanelsAreAccessiblePersistentAndKeepChartMounted()
    {
        string homeMarkup = File.ReadAllText(
            Path.Combine(AppContext.BaseDirectory, "TestAssets", "Home.razor"));

        Assert.Equal(6, Count(homeMarkup, "class=\"panel-toggle\""));
        Assert.Contains("aria-expanded=", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("aria-controls=", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("hidden=\"@IsPanelCollapsed(PanelTelemetry)\"", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("<SpeedChart", homeMarkup, StringComparison.Ordinal);
        Assert.DoesNotContain("@if (!IsPanelCollapsed(PanelTelemetry))", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("hidden=\"@IsPanelCollapsed(PanelCurrents)\"", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("<CurrentChart", homeMarkup, StringComparison.Ordinal);
        Assert.DoesNotContain("@if (!IsPanelCollapsed(PanelCurrents))", homeMarkup, StringComparison.Ordinal);
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

    [Fact]
    public void DashboardExposesSynchronizedCurrentAndBemfChannels()
    {
        string homeMarkup = File.ReadAllText(
            Path.Combine(AppContext.BaseDirectory, "TestAssets", "Home.razor"));

        Assert.Contains("PA3 · SENSE_A", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("PA4 · SENSE_B", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("LogParam.CurrentU", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("LogParam.CurrentV", homeMarkup, StringComparison.Ordinal);
        Assert.Contains("LogParam.BemfPeriod", homeMarkup, StringComparison.Ordinal);
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
