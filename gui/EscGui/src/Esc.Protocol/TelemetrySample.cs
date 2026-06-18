namespace Esc.Protocol;

public sealed record TelemetrySample(
    string Variable,
    LogParam Parameter,
    int RawValue,
    string Unit,
    uint TargetTickMs,
    DateTimeOffset HostTimestamp)
{
    public double DisplayValue => Parameter == LogParam.Temperature ? RawValue / 100.0 : RawValue;
}
