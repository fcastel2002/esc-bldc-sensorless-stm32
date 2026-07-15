namespace Esc.Bridge;

public sealed class RpmPiReferenceModel
{
    public const int CanonicalArr = 2000;
    public const int CanonicalMinimum = 100;

    private int _integralQ16;
    private int _previousError;

    public void Reset()
    {
        _integralQ16 = 0;
        _previousError = 0;
    }

    public ushort Step(ushort speedRpm, ushort targetRpm, ValidationReferenceConfig config)
    {
        if (config.AlgorithmVersion != 2 || config.Kd != 0)
        {
            throw new InvalidOperationException("RPM PI reference requires algorithm version 2 with KD=0.");
        }

        int error = targetRpm - speedRpm;
        int kpQ16 = checked((int)(config.Kp * 65536.0 + 0.5));
        int kiDtHalfQ30 = checked((int)(config.Ki * config.Dt * 0.5 * 1073741824.0 + 0.5));
        long proportionalQ16 = (long)kpQ16 * error;
        long integralDeltaQ16 = ((long)kiDtHalfQ30 * (error + _previousError)) >> 14;
        long minimumQ16 = (long)CanonicalMinimum << 16;
        long maximumQ16 = (long)CanonicalArr << 16;
        long integralQ16 = Math.Clamp(
            (long)_integralQ16 + integralDeltaQ16,
            minimumQ16 - proportionalQ16,
            maximumQ16 - proportionalQ16);
        _integralQ16 = (int)Math.Clamp(integralQ16, int.MinValue, int.MaxValue);
        _previousError = error;
        long outputQ16 = Math.Clamp(proportionalQ16 + _integralQ16, minimumQ16, maximumQ16);
        int canonicalPwm = (int)(outputQ16 >> 16);
        return checked((ushort)((canonicalPwm * config.PwmArr + CanonicalArr / 2) / CanonicalArr));
    }
}
