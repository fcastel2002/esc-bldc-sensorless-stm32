using System.Net;

namespace Esc.Bridge;

internal sealed class HilUdpSenderSession
{
    private readonly double _leaseMs;
    private IPEndPoint? _activeSender;
    private uint? _activeSequence;
    private double _lastAcceptedAtMs = double.NegativeInfinity;

    public HilUdpSenderSession(double leaseMs)
    {
        _leaseMs = leaseMs;
    }

    public bool CanAccept(
        IPEndPoint sender,
        uint? sequence,
        bool isStartCommand,
        bool isEmergencyStop,
        double nowMs)
    {
        if (isEmergencyStop)
        {
            return true;
        }

        ExpireIfIdle(nowMs);
        if (_activeSender is null || Matches(sender))
        {
            return true;
        }

        if (isStartCommand)
        {
            return true;
        }

        return sequence.HasValue &&
            _activeSequence.HasValue &&
            sequence.Value < _activeSequence.Value;
    }

    public bool WouldTakeOver(
        IPEndPoint sender,
        uint? sequence,
        bool isStartCommand,
        double nowMs)
    {
        ExpireIfIdle(nowMs);
        if (_activeSender is null || Matches(sender))
        {
            return false;
        }

        if (isStartCommand)
        {
            return true;
        }

        return sequence.HasValue &&
            _activeSequence.HasValue &&
            sequence.Value < _activeSequence.Value;
    }

    public void Accept(IPEndPoint sender, uint? sequence, double nowMs)
    {
        _activeSender = new IPEndPoint(sender.Address, sender.Port);
        _activeSequence = sequence;
        _lastAcceptedAtMs = nowMs;
    }

    public void Release()
    {
        _activeSender = null;
        _activeSequence = null;
        _lastAcceptedAtMs = double.NegativeInfinity;
    }

    public void Release(IPEndPoint sender)
    {
        if (Matches(sender))
        {
            Release();
        }
    }

    public string DescribeActiveSender(double nowMs)
    {
        ExpireIfIdle(nowMs);
        return _activeSender is null
            ? "none"
            : $"{_activeSender.Address}:{_activeSender.Port}";
    }

    private void ExpireIfIdle(double nowMs)
    {
        if (_activeSender is not null && nowMs - _lastAcceptedAtMs > _leaseMs)
        {
            Release();
        }
    }

    private bool Matches(IPEndPoint sender)
    {
        return _activeSender is not null &&
            _activeSender.Address.Equals(sender.Address) &&
            _activeSender.Port == sender.Port;
    }
}
