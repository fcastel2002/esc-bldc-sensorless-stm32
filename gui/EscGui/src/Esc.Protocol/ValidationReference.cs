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
    ushort MinimumPwmCounts,
    byte CapabilityFlags = 0,
    byte HilStepOperationVersion = 0,
    ushort MaximumHilSteps = 0)
{
    public const byte SupportedAlgorithmVersion = 2;
    public const byte DeterministicHilCapability = 0x01;
    public double ControllerDtSeconds => ControllerDtUs / 1_000_000d;
    public bool SupportsDeterministicHil => (CapabilityFlags & DeterministicHilCapability) != 0;
}
