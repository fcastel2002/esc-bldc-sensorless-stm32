namespace Esc.Protocol;

public sealed record TelemetrySample(
    string Variable,
    LogParam Parameter,
    int RawValue,
    string Unit,
    uint TargetTickMs,
    DateTimeOffset HostTimestamp,
    ushort? AdcRawValue = null,
    TelemetryQuality Quality = TelemetryQuality.None,
    ushort ValidSampleCount = 0,
    sbyte? CommutationStep = null)
{
    public double DisplayValue => Parameter == LogParam.Temperature ? RawValue / 100.0 : RawValue;
    public bool IsValid => (Quality & TelemetryQuality.Valid) != 0;
}
