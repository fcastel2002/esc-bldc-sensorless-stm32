namespace Esc.Bridge;

public enum DeviceConnectionState
{
    NotDetected,
    Detected,
    Connecting,
    Connected,
    Error,
    Disconnected,
}

public enum ControlMode
{
    GuiControl,
    SimulinkControl,
    MonitorOnly,
}
