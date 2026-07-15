namespace Esc.Protocol;

public sealed record ValidationReference(
    byte SchemaVersion,
    byte AlgorithmVersion,
    ushort PwmFrequencyHz,
    ushort PwmArrCounts,
    uint SpeedTimerHz,
    ushort SpeedMinPeriodTicks,
    ushort SpeedMaxPeriodTicks,
    uint ControllerDtUs,
    ushort MinimumPwmCounts)
{
    public const byte SupportedAlgorithmVersion = 2;
    public double ControllerDtSeconds => ControllerDtUs / 1_000_000d;
}
