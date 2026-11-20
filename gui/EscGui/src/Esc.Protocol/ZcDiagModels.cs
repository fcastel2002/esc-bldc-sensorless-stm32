namespace Esc.Protocol;

/// <summary>
/// Mapeo por software de fases logicas (U, V, W) a canales fisicos de TIM2:
/// 0 = PA0 (TIM2_CH1), 1 = PA1 (TIM2_CH2), 2 = PB10 (TIM2_CH3).
/// </summary>
public sealed record ZcChannelMap(byte ChannelU, byte ChannelV, byte ChannelW)
{
    public static readonly ZcChannelMap Default = new(2, 1, 0);

    public bool IsValid =>
        ChannelU < 3 && ChannelV < 3 && ChannelW < 3 &&
        ChannelU != ChannelV && ChannelU != ChannelW && ChannelV != ChannelW;

    public string PhaseForHardwareChannel(byte hwChannel)
    {
        if (hwChannel == ChannelU) return "U";
        if (hwChannel == ChannelV) return "V";
        if (hwChannel == ChannelW) return "W";
        return "?";
    }

    public static string HardwareChannelLabel(byte hwChannel) => hwChannel switch
    {
        0 => "PA0 (CH1)",
        1 => "PA1 (CH2)",
        2 => "PB10 (CH3)",
        _ => $"CH{hwChannel + 1}"
    };

    public static string HardwarePinShort(byte hwChannel) => hwChannel switch
    {
        0 => "PA0",
        1 => "PA1",
        2 => "PB10",
        _ => $"CH{hwChannel + 1}"
    };

    public string FormatSummary() =>
        $"U={HardwarePinShort(ChannelU)}, V={HardwarePinShort(ChannelV)}, W={HardwarePinShort(ChannelW)}";
}

public sealed record ZcRecentEvent(byte HardwareChannel, bool IsRising);

public sealed record ZcDiagReport(
    bool Active,
    byte LivePinsMask,
    byte SeenStatesMask,
    ZcHealthFlags HealthFlags,
    ushort EdgesCh0,
    ushort EdgesCh1,
    ushort EdgesCh2,
    ushort ForwardSteps,
    ushort ReverseSteps,
    ushort InvalidSteps,
    ushort GlitchCount,
    ushort EstimatedRpm,
    ushort AvgStepTicks,
    byte StepAsymmetryPct,
    ZcChannelMap ActiveMap,
    IReadOnlyList<ZcRecentEvent> RecentEvents)
{
    public bool IsPinHigh(byte hwChannel) => (LivePinsMask & (1 << hwChannel)) != 0;

    public ushort EdgesForChannel(byte hwChannel) => hwChannel switch
    {
        0 => EdgesCh0,
        1 => EdgesCh1,
        2 => EdgesCh2,
        _ => 0
    };

    public bool IsChannelDead(byte hwChannel) => hwChannel switch
    {
        0 => HealthFlags.HasFlag(ZcHealthFlags.Channel0Dead),
        1 => HealthFlags.HasFlag(ZcHealthFlags.Channel1Dead),
        2 => HealthFlags.HasFlag(ZcHealthFlags.Channel2Dead),
        _ => false
    };

    public bool HasDeadChannel =>
        HealthFlags.HasFlag(ZcHealthFlags.Channel0Dead) ||
        HealthFlags.HasFlag(ZcHealthFlags.Channel1Dead) ||
        HealthFlags.HasFlag(ZcHealthFlags.Channel2Dead);
}

public sealed record ZcAutoDetectResult(
    ZcAutoDetectStatus Status,
    ZcChannelMap? DetectedMap,
    ZcChannelMap ActiveMap,
    IReadOnlyList<byte> HighPercentMatrix)
{
    public bool HasDetectedMap => DetectedMap is { IsValid: true };

    public bool MatchesActiveMap => DetectedMap is not null && DetectedMap == ActiveMap;

    public byte HighPercent(int probePhaseIndex, int hwChannel) =>
        HighPercentMatrix[probePhaseIndex * 3 + hwChannel];
}
