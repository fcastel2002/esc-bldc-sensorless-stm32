namespace Esc.Web.Services;

public sealed record DashboardControl
{
    public string Id { get; init; } = string.Empty;
    public string Label { get; init; } = string.Empty;
    public string Kind { get; init; } = "number";
    public string Command { get; init; } = string.Empty;
    public string? Param { get; init; }
    public double Min { get; init; }
    public double Max { get; init; }
    public double Step { get; init; } = 1;
    public string Unit { get; init; } = string.Empty;
    public double DefaultValue { get; init; }
    public bool RequiresIdle { get; init; }
}
