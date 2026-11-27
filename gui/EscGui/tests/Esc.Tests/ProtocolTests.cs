using System.Buffers.Binary;
using Esc.Protocol;

namespace Esc.Tests;

public sealed class ProtocolTests
{
    [Fact]
    public void BuildPingFrameMatchesExistingPythonHelperCrc()
    {
        byte[] frame = EscProtocol.BuildRequest(1, CommOpcode.Ping, 0, "ping"u8.ToArray());

        Assert.Equal(CommConstants.FrameSize, frame.Length);
        Assert.Equal(0xEB, frame[62]);
        Assert.Equal(0x15, frame[63]);
    }

    [Fact]
    public void ParseRoundTripDecodesRequest()
    {
        byte[] frame = EscProtocol.BuildRequest(7, CommOpcode.SetSpeedRpm, payload: EscProtocol.UInt16Payload(1500));

        EscFrame parsed = EscProtocol.Parse(frame);

        Assert.Equal(CommFrameType.Request, parsed.Type);
        Assert.Equal((byte)7, parsed.Sequence);
        Assert.Equal(CommOpcode.SetSpeedRpm, parsed.Opcode);
        Assert.Equal(new byte[] { 0xDC, 0x05 }, parsed.Payload);
    }

    [Fact]
    public void ParseRejectsBadCrc()
    {
        byte[] frame = EscProtocol.BuildRequest(1, CommOpcode.GetStatus);
        frame[12] ^= 0x55;

        Assert.Throws<EscProtocolException>(() => EscProtocol.Parse(frame));
    }

    [Fact]
    public void DecodeTelemetryReadsSpeedEvent()
    {
        byte[] payload = new byte[9];
        payload[0] = (byte)LogParam.Speed;
        BitConverter.GetBytes(1234).CopyTo(payload, 1);
        BitConverter.GetBytes(5678u).CopyTo(payload, 5);

        byte[] raw = EscProtocol.BuildFrame(2, CommOpcode.TelemetryEvent, (byte)LogParam.Speed, payload, CommFrameType.Event, CommStatus.Ok);
        TelemetrySample sample = EscProtocol.DecodeTelemetry(EscProtocol.Parse(raw), DateTimeOffset.UnixEpoch);

        Assert.Equal("speed", sample.Variable);
        Assert.Equal(1234, sample.RawValue);
        Assert.Equal(5678u, sample.TargetTickMs);
        Assert.Equal("rpm", sample.Unit);
    }

    [Fact]
    public void DecodeTelemetryReadsExtendedDutyMetadata()
    {
        byte[] payload = new byte[15];
        payload[0] = (byte)LogParam.Duty;
        BitConverter.GetBytes(1200).CopyTo(payload, 1);
        BitConverter.GetBytes(5678u).CopyTo(payload, 5);
        BitConverter.GetBytes((ushort)4321).CopyTo(payload, 9);
        payload[11] = (byte)TelemetryQuality.Valid;
        BitConverter.GetBytes((ushort)25).CopyTo(payload, 12);
        payload[14] = 3;

        byte[] raw = EscProtocol.BuildFrame(
            2, CommOpcode.TelemetryEvent, (byte)LogParam.Duty,
            payload, CommFrameType.Event, CommStatus.Ok);
        TelemetrySample sample = EscProtocol.DecodeTelemetry(
            EscProtocol.Parse(raw), DateTimeOffset.UnixEpoch);

        Assert.Equal("duty", sample.Variable);
        Assert.Equal("%", sample.Unit);
        Assert.Equal(1200, sample.RawValue);
        Assert.Equal((ushort)4321, sample.AdcRawValue);
        Assert.True(sample.IsValid);
        Assert.Equal((ushort)25, sample.ValidSampleCount);
        Assert.Equal((sbyte)3, sample.CommutationStep);
    }

    [Fact]
    public void DecodeTelemetryReportsDutyInPercent()
    {
        byte[] payload = new byte[15];
        payload[0] = (byte)LogParam.Duty;
        BitConverter.GetBytes(4567).CopyTo(payload, 1);
        byte[] raw = EscProtocol.BuildFrame(
            2, CommOpcode.TelemetryEvent, (byte)LogParam.Duty, payload, CommFrameType.Event, CommStatus.Ok);

        TelemetrySample sample = EscProtocol.DecodeTelemetry(EscProtocol.Parse(raw), DateTimeOffset.UnixEpoch);

        Assert.Equal("duty", sample.Variable);
        Assert.Equal("%", sample.Unit);
        Assert.Equal(45.67, sample.DisplayValue, 6);
    }

    [Fact]
    public void DecodeTelemetryRejectsPartialExtendedPayload()
    {
        byte[] payload = new byte[12];
        payload[0] = (byte)LogParam.Duty;
        byte[] raw = EscProtocol.BuildFrame(
            2, CommOpcode.TelemetryEvent, (byte)LogParam.Duty,
            payload, CommFrameType.Event, CommStatus.Ok);

        Assert.Throws<EscProtocolException>(() =>
            EscProtocol.DecodeTelemetry(EscProtocol.Parse(raw), DateTimeOffset.UnixEpoch));
    }

    [Fact]
    public void DecodeStatusParsesIna219PowerExtension()
    {
        byte[] payload = new byte[26];
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(24, 2), 513);
        payload[20] = 0x01;
        payload[21] = (byte)PowerMonitorStage.ConfigVerify;
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(22, 2), 0x399F);
        payload[1] = 1;
        payload[18] = (byte)PowerMonitorError.Nack;
        payload[19] = 7;
        payload[10] = (byte)(PowerMonitorFlags.Present | PowerMonitorFlags.Valid);
        payload[11] = 0x40;
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(12, 2), 12_040);
        BinaryPrimitives.WriteInt32LittleEndian(payload.AsSpan(14, 4), -250_300);
        byte[] raw = EscProtocol.BuildFrame(3, CommOpcode.GetStatus, 0, payload, CommFrameType.Response, CommStatus.Ok);

        EscStatus status = EscProtocol.DecodeStatus(EscProtocol.Parse(raw));

        Assert.NotNull(status.Power);
        Assert.True(status.Power!.Present);
        Assert.True(status.Power.Valid);
        Assert.False(status.Power.Disabled);
        Assert.Equal(0x40, status.Power.I2cAddress);
        Assert.Equal(12.04, status.Power.BusVolts, 3);
        Assert.Equal(-0.2503, status.Power.CurrentAmps, 6);
        Assert.Equal(12.04 * -0.2503, status.Power.PowerWatts, 6);
        Assert.Equal(PowerMonitorError.Nack, status.Power.LastError);
        Assert.Equal(7, status.Power.ErrorCount);
        Assert.True(status.Power.SclIdleHigh);
        Assert.False(status.Power.SdaIdleHigh);
        Assert.Equal(PowerMonitorStage.ConfigVerify, status.Power.FailStage);
        Assert.Equal(0x399F, status.Power.ConfigReadback);
        Assert.Equal(513, status.Power.BusyRecoveries);
    }

    [Fact]
    public void DecodeStatusLegacyPayloadHasNoPowerReading()
    {
        byte[] raw = EscProtocol.BuildFrame(3, CommOpcode.GetStatus, 0, new byte[10], CommFrameType.Response, CommStatus.Ok);

        EscStatus status = EscProtocol.DecodeStatus(EscProtocol.Parse(raw));
        Assert.Null(status.Power);
        Assert.Null(status.ReferenceRpm);
        Assert.Null(status.DutyPercent);
        Assert.False(status.LoadSweepActive);
    }

    [Fact]
    public void DecodeStatusReadsReferenceDutyAndSweepExtension()
    {
        byte[] payload = new byte[31];
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(4, 2), 3000);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(26, 2), 2750);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(28, 2), 4321);
        payload[30] = 1;
        byte[] raw = EscProtocol.BuildFrame(3, CommOpcode.GetStatus, 0, payload, CommFrameType.Response, CommStatus.Ok);

        EscStatus status = EscProtocol.DecodeStatus(EscProtocol.Parse(raw));

        Assert.Equal((ushort)3000, status.SpeedSetpointRpm);
        Assert.Equal((ushort)2750, status.ReferenceRpm);
        Assert.Equal(43.21, status.DutyPercent!.Value, 6);
        Assert.True(status.LoadSweepActive);
        Assert.NotNull(status.Power);
    }

    [Theory]
    [InlineData(LogParam.BusVoltage, "bus_voltage", "mV", 12040)]
    [InlineData(LogParam.BusCurrent, "bus_current", "mA", -250)]
    [InlineData(LogParam.Duty, "duty", "%", 5000)]
    public void DecodeTelemetryNamesIna219Channels(LogParam parameter, string name, string unit, int value)
    {
        byte[] payload = new byte[15];
        payload[0] = (byte)parameter;
        BinaryPrimitives.WriteInt32LittleEndian(payload.AsSpan(1, 4), value);
        byte[] raw = EscProtocol.BuildFrame(0, CommOpcode.TelemetryEvent, (byte)parameter, payload, CommFrameType.Event, CommStatus.Ok);

        TelemetrySample sample = EscProtocol.DecodeTelemetry(EscProtocol.Parse(raw), DateTimeOffset.UtcNow);

        Assert.Equal(name, sample.Variable);
        Assert.Equal(unit, sample.Unit);
        Assert.Equal(value, sample.RawValue);
    }

    [Fact]
    public void DecodeStatusUsesFirmwareStateOrder()
    {
        byte[] payload = new byte[10];
        payload[0] = 2;
        payload[1] = 1;

        byte[] raw = EscProtocol.BuildFrame(3, CommOpcode.GetStatus, 0, payload, CommFrameType.Response, CommStatus.Ok);
        EscStatus status = EscProtocol.DecodeStatus(EscProtocol.Parse(raw));

        Assert.Equal("FOC_STARTUP", status.AppStateName);
    }

    [Fact]
    public void DecodeStatusRecognizesSineDriveStateWithoutRenumberingExistingStates()
    {
        byte[] payload = new byte[10];
        payload[0] = 10;
        byte[] raw = EscProtocol.BuildFrame(
            3, CommOpcode.GetStatus, 0, payload, CommFrameType.Response, CommStatus.Ok);

        EscStatus status = EscProtocol.DecodeStatus(EscProtocol.Parse(raw));

        Assert.Equal("SINE_DRIVE", status.AppStateName);
        Assert.Equal(0x50, (byte)CommOpcode.SineDrive);
        Assert.Equal(0, (byte)SineDriveCommand.Apply);
        Assert.Equal(1, (byte)SineDriveCommand.KeepAlive);
    }

    [Fact]
    public void SineDrivePayloadUsesExactFixedPointWireShape()
    {
        byte[] payload = EscProtocol.SineDrivePayload(7.25, 42.5);

        Assert.Equal([0x52, 0x1C, 0x00, 0x00, 0xA9, 0x01], payload);
    }

    [Theory]
    [InlineData(1.99, 20)]
    [InlineData(10.01, 20)]
    [InlineData(5, -0.1)]
    [InlineData(5, 100.1)]
    [InlineData(double.NaN, 20)]
    public void SineDrivePayloadRejectsOutOfRangeValues(double frequencyHz, double amplitudePercent)
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => EscProtocol.SineDrivePayload(frequencyHz, amplitudePercent));
    }

    [Fact]
    public void StepTestPayloadUsesFirmwareStepIndexAndPermilleDuty()
    {
        byte[] payload = EscProtocol.StepTestPayload(new StepTestSettings(CommutationStep.WV, 30));

        Assert.Equal(0x51, (byte)CommOpcode.StepTest);
        Assert.Equal(0, (byte)CommutationStep.UV);
        Assert.Equal([5, 0x2C, 0x01], payload);
    }

    [Theory]
    [InlineData(-0.1)]
    [InlineData(30.1)]
    [InlineData(double.NaN)]
    public void StepTestPayloadRejectsOutOfRangeDuty(double dutyPercent)
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => EscProtocol.StepTestPayload(new StepTestSettings(CommutationStep.UV, dutyPercent)));
    }

    [Fact]
    public void StepTestPayloadRejectsUnknownStep()
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => EscProtocol.StepTestPayload(new StepTestSettings((CommutationStep)6, 5)));
    }

    [Fact]
    public void DecodeStepTestAndStatusRecognizeStepTestState()
    {
        byte[] raw = EscProtocol.BuildFrame(
            4, CommOpcode.StepTest, 0, [3, 0x64, 0x00], CommFrameType.Response, CommStatus.Ok);
        byte[] statusPayload = new byte[10];
        statusPayload[0] = 11;
        byte[] rawStatus = EscProtocol.BuildFrame(
            5, CommOpcode.GetStatus, 0, statusPayload, CommFrameType.Response, CommStatus.Ok);

        StepTestSettings settings = EscProtocol.DecodeStepTest(EscProtocol.Parse(raw));

        Assert.Equal(new StepTestSettings(CommutationStep.VU, 10), settings);
        Assert.Equal("STEP_TEST", EscProtocol.DecodeStatus(EscProtocol.Parse(rawStatus)).AppStateName);
    }

    [Fact]
    public void ZcDiagReportAndAutoDetectAndZcMapDecodeCorrectly()
    {
        var map = new ZcChannelMap(1, 2, 0);
        byte[] mapPayload = EscProtocol.ZcMapPayload(map);
        Assert.Equal([1, 2, 0], mapPayload);
        Assert.Equal(map, Assert.IsType<ZcChannelMap>(EscProtocol.DecodeConfigValue(ConfigParam.ZcMap, mapPayload)));
        Assert.Throws<ArgumentOutOfRangeException>(() => EscProtocol.ZcMapPayload(new ZcChannelMap(1, 1, 0)));

        byte[] reportPayload = new byte[32];
        reportPayload[0] = 1; // active
        reportPayload[1] = 0b101; // PA0 & PB10 high
        reportPayload[2] = 0x7E; // all 6 valid states seen
        reportPayload[3] = (byte)(ZcHealthFlags.HasMotion | ZcHealthFlags.SequenceValid | ZcHealthFlags.DirectionForward);
        BitConverter.GetBytes((ushort)12).CopyTo(reportPayload, 4);
        BitConverter.GetBytes((ushort)12).CopyTo(reportPayload, 6);
        BitConverter.GetBytes((ushort)12).CopyTo(reportPayload, 8);
        BitConverter.GetBytes((ushort)35).CopyTo(reportPayload, 10);
        BitConverter.GetBytes((ushort)0).CopyTo(reportPayload, 12);
        BitConverter.GetBytes((ushort)0).CopyTo(reportPayload, 14);
        BitConverter.GetBytes((ushort)1).CopyTo(reportPayload, 16);
        BitConverter.GetBytes((ushort)420).CopyTo(reportPayload, 18);
        BitConverter.GetBytes((ushort)2143).CopyTo(reportPayload, 20);
        reportPayload[22] = 4;
        reportPayload[23] = 2;
        reportPayload[24] = 1;
        reportPayload[25] = 0;
        reportPayload[26] = 0x80 | 0x04 | 2; // PB10 rising
        reportPayload[27] = 0x80 | 0x00 | 0; // PA0 falling

        byte[] rawReport = EscProtocol.BuildFrame(
            7, CommOpcode.ZcDiag, (byte)ZcDiagCommand.GetReport, reportPayload, CommFrameType.Response, CommStatus.Ok);
        ZcDiagReport report = EscProtocol.DecodeZcDiagReport(EscProtocol.Parse(rawReport));

        Assert.True(report.Active);
        Assert.True(report.IsPinHigh(0));
        Assert.False(report.IsPinHigh(1));
        Assert.True(report.IsPinHigh(2));
        Assert.Equal((ushort)420, report.EstimatedRpm);
        Assert.Equal((byte)4, report.StepAsymmetryPct);
        Assert.Equal(ZcChannelMap.Default, report.ActiveMap);
        Assert.Equal(2, report.RecentEvents.Count);
        Assert.Equal(new ZcRecentEvent(2, true), report.RecentEvents[0]);
        Assert.Equal(new ZcRecentEvent(0, false), report.RecentEvents[1]);

        byte[] detectPayload = [0, 1, 2, 0, 2, 1, 0, 10, 95, 5, 5, 8, 92, 96, 4, 6];
        byte[] rawDetect = EscProtocol.BuildFrame(
            8, CommOpcode.ZcDiag, (byte)ZcDiagCommand.AutoDetect, detectPayload, CommFrameType.Response, CommStatus.Ok);
        ZcAutoDetectResult detected = EscProtocol.DecodeZcAutoDetect(EscProtocol.Parse(rawDetect));

        Assert.Equal(ZcAutoDetectStatus.Ok, detected.Status);
        Assert.Equal(new ZcChannelMap(1, 2, 0), detected.DetectedMap);
        Assert.Equal(ZcChannelMap.Default, detected.ActiveMap);
        Assert.False(detected.MatchesActiveMap);
        Assert.Equal((byte)95, detected.HighPercent(0, 1));
        Assert.Equal((byte)92, detected.HighPercent(1, 2));
        Assert.Equal((byte)96, detected.HighPercent(2, 0));

        byte[] statusPayload = new byte[10];
        statusPayload[0] = 12;
        byte[] rawStatus12 = EscProtocol.BuildFrame(
            9, CommOpcode.GetStatus, 0, statusPayload, CommFrameType.Response, CommStatus.Ok);
        Assert.Equal("FREEWHEEL", EscProtocol.DecodeStatus(EscProtocol.Parse(rawStatus12)).AppStateName);
    }

    [Fact]
    public void DecodeSineDriveReadsAppliedQuantizedSettings()
    {
        byte[] raw = EscProtocol.BuildFrame(
            2,
            CommOpcode.SineDrive,
            0,
            [0x51, 0x1C, 0x00, 0x00, 0xA9, 0x01],
            CommFrameType.Response,
            CommStatus.Ok);

        SineDriveSettings settings = EscProtocol.DecodeSineDrive(EscProtocol.Parse(raw));

        Assert.Equal(7.249, settings.ElectricalFrequencyHz);
        Assert.Equal(42.5, settings.AmplitudePercent);
    }

    [Theory]
    [InlineData(ConfigParam.StartupInitialAmplitude, 20.0, 2)]
    [InlineData(ConfigParam.StartupFinalAmplitude, 100.0, 2)]
    [InlineData(ConfigParam.StartupInitialFrequency, 2.09, 4)]
    [InlineData(ConfigParam.StartupFinalFrequency, 9.28, 4)]
    [InlineData(ConfigParam.StartupDuration, 3.0, 4)]
    public void StartupConfigPayloadRoundTripsPhysicalUnits(ConfigParam parameter, double value, int length)
    {
        byte[] payload = EscProtocol.ConfigPayload(parameter, value);

        Assert.Equal(length, payload.Length);
        Assert.Equal(value, Assert.IsType<double>(EscProtocol.DecodeConfigValue(parameter, payload)), 3);
    }

    [Fact]
    public void StartupConfigPayloadsUseDocumentedWireUnits()
    {
        Assert.Equal([0xC8, 0x00], EscProtocol.ConfigPayload(ConfigParam.StartupInitialAmplitude, 20));
        Assert.Equal([0x2A, 0x08, 0x00, 0x00], EscProtocol.ConfigPayload(ConfigParam.StartupInitialFrequency, 2.09));
        Assert.Equal([0xB8, 0x0B, 0x00, 0x00], EscProtocol.ConfigPayload(ConfigParam.StartupDuration, 3));
    }

    [Fact]
    public void BemfBlankingUsesAssignedParameterAndUInt16Microseconds()
    {
        byte[] payload = EscProtocol.ConfigPayload(ConfigParam.BemfBlankingUs, 0x1234);

        Assert.Equal(0x12, (byte)ConfigParam.BemfBlankingUs);
        Assert.Equal([0x34, 0x12], payload);
        Assert.Equal((ushort)0x1234, Assert.IsType<ushort>(
            EscProtocol.DecodeConfigValue(ConfigParam.BemfBlankingUs, payload)));
    }

    [Fact]
    public void RpmPiGainUsesNewParameterAndHundredthsPayload()
    {
        byte[] payload = EscProtocol.ConfigPayload(ConfigParam.KpRpm, 0.28);

        Assert.Equal((byte)0x0A, (byte)ConfigParam.KpRpm);
        Assert.Equal([0x1C, 0x00], payload);
    }

    [Fact]
    public void SpeedRampUsesAssignedParameterAndUInt16RpmPerSecond()
    {
        byte[] payload = EscProtocol.ConfigPayload(ConfigParam.SpeedRamp, 750);

        Assert.Equal((byte)0x14, (byte)ConfigParam.SpeedRamp);
        Assert.Equal([0xEE, 0x02], payload);
        Assert.Equal((ushort)750, Assert.IsType<ushort>(EscProtocol.DecodeConfigValue(ConfigParam.SpeedRamp, payload)));
    }

    [Theory]
    [InlineData(0x03)]
    [InlineData(0x04)]
    [InlineData(0x05)]
    [InlineData(0x08)]
    [InlineData(0x09)]
    public void RemovedConfigParametersAreNotDefined(byte value)
    {
        Assert.False(Enum.IsDefined(typeof(ConfigParam), value));
    }

    [Theory]
    [InlineData(0x14)]
    [InlineData(0x24)]
    [InlineData(0x40)]
    [InlineData(0x41)]
    [InlineData(0x42)]
    [InlineData(0x43)]
    [InlineData(0x44)]
    public void RemovedOpcodesAreNotDefined(byte value)
    {
        Assert.False(Enum.IsDefined(typeof(CommOpcode), value));
    }

    [Theory]
    [InlineData(0x02)]
    [InlineData(0x03)]
    [InlineData(0x04)]
    [InlineData(0x05)]
    [InlineData(0x09)]
    public void RemovedLogParametersAreNotDefined(byte value)
    {
        Assert.False(Enum.IsDefined(typeof(LogParam), value));
    }

    [Fact]
    public void LoadSweepUsesAssignedOpcodeAndCommands()
    {
        Assert.Equal((byte)0x60, (byte)CommOpcode.LoadSweep);
        Assert.Equal((byte)0, (byte)LoadSweepCommand.Start);
        Assert.Equal((byte)1, (byte)LoadSweepCommand.Abort);
        Assert.Equal((byte)2, (byte)LoadSweepCommand.GetStatus);
        Assert.Equal((byte)3, (byte)LoadSweepCommand.GetPoints);
    }

    [Fact]
    public void LoadSweepStartPayloadUsesExactLittleEndianWireShape()
    {
        byte[] payload = EscProtocol.LoadSweepStartPayload(new LoadSweepRequest(
            LoadSweepMode.Ramp, 1000, 3000, 0x0102, 1500, 0x0304, LoadSweepOptions.StopAtEnd));

        Assert.Equal(
            [0x01, 0xE8, 0x03, 0xB8, 0x0B, 0x02, 0x01, 0xDC, 0x05, 0x04, 0x03, 0x01],
            payload);
    }

    [Fact]
    public void LoadSweepStartPayloadRejectsUnknownMode()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => EscProtocol.LoadSweepStartPayload(
            new LoadSweepRequest((LoadSweepMode)7, 1000, 2000, 100, 0, 500)));
    }

    [Fact]
    public void DecodeLoadSweepStatusReadsExactPayload()
    {
        byte[] payload = new byte[29];
        payload[0] = (byte)LoadSweepState.Measuring;
        payload[1] = (byte)LoadSweepAbortReason.None;
        payload[2] = (byte)LoadSweepMode.Staircase;
        payload[3] = (byte)LoadSweepOptions.StopAtEnd;
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(4, 2), 5);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(6, 2), 128);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(8, 2), 1000);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(10, 2), 3000);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(12, 2), 250);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(14, 2), 1500);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(16, 2), 1000);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(18, 2), 2250);
        BinaryPrimitives.WriteInt32LittleEndian(payload.AsSpan(20, 4), 14_500);
        BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(24, 4), 23_456);
        payload[28] = 1;
        byte[] raw = EscProtocol.BuildFrame(
            5, CommOpcode.LoadSweep, (byte)LoadSweepCommand.GetStatus, payload, CommFrameType.Response, CommStatus.Ok);

        LoadSweepStatus status = EscProtocol.DecodeLoadSweepStatus(EscProtocol.Parse(raw));

        Assert.Equal(LoadSweepState.Measuring, status.State);
        Assert.True(status.IsActive);
        Assert.Equal(LoadSweepOptions.StopAtEnd, status.Options);
        Assert.Equal((ushort)5, status.PointCount);
        Assert.Equal((ushort)128, status.Capacity);
        Assert.Equal((ushort)1000, status.StartRpm);
        Assert.Equal((ushort)3000, status.EndRpm);
        Assert.Equal((ushort)250, status.StepOrRate);
        Assert.Equal((ushort)1500, status.SettleMs);
        Assert.Equal((ushort)1000, status.WindowMs);
        Assert.Equal((ushort)2250, status.TargetRpm);
        Assert.Equal(14_500, status.IdleCurrentMicroamps);
        Assert.Equal(23_456u, status.ElapsedMs);
        Assert.True(status.IdleCurrentValid);
    }

    [Theory]
    [InlineData(LoadSweepState.Idle, false)]
    [InlineData(LoadSweepState.MeasuringIdleCurrent, true)]
    [InlineData(LoadSweepState.Measuring, true)]
    [InlineData(LoadSweepState.Done, false)]
    [InlineData(LoadSweepState.Aborted, false)]
    public void LoadSweepStatusActiveCoversOnlyRunningStates(LoadSweepState state, bool active)
    {
        var status = new LoadSweepStatus(state, LoadSweepAbortReason.None, LoadSweepMode.Ramp, LoadSweepOptions.None,
            0, 128, 1000, 2000, 50, 0, 500, 1000, 0, 0, false);

        Assert.Equal(active, status.IsActive);
    }

    [Fact]
    public void DecodeLoadSweepStatusRejectsWrongLength()
    {
        byte[] raw = EscProtocol.BuildFrame(
            5, CommOpcode.LoadSweep, (byte)LoadSweepCommand.GetStatus, new byte[28], CommFrameType.Response, CommStatus.Ok);

        Assert.Throws<EscProtocolException>(() => EscProtocol.DecodeLoadSweepStatus(EscProtocol.Parse(raw)));
    }

    [Fact]
    public void DecodeLoadSweepPointsReadsPackedPoints()
    {
        byte[] payload = new byte[3 + 2 * 16];
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(0, 2), 7);
        payload[2] = 2;
        WritePoint(payload.AsSpan(3, 16), 1000, 998, 2345, 12_040, 812, 3_100, 45, 12, LoadPointFlags.None);
        WritePoint(payload.AsSpan(19, 16), 1250, 1249, 150, 11_990, -5, 0, 3, 2,
            LoadPointFlags.PartialPower);
        byte[] raw = EscProtocol.BuildFrame(
            6, CommOpcode.LoadSweep, (byte)LoadSweepCommand.GetPoints, payload, CommFrameType.Response, CommStatus.Ok);

        LoadSweepPointBatch batch = EscProtocol.DecodeLoadSweepPoints(EscProtocol.Parse(raw));

        Assert.Equal((ushort)7, batch.FirstIndex);
        Assert.Equal(2, batch.Points.Count);
        LoadSweepPoint first = batch.Points[0];
        Assert.Equal(7, first.Index);
        Assert.Equal((ushort)1000, first.ReferenceRpm);
        Assert.Equal((ushort)998, first.RpmMean);
        Assert.Equal(23.45, first.DutyPercent, 6);
        Assert.Equal(12.04, first.BusVolts, 6);
        Assert.Equal(81.2, first.BusCurrentMa, 6);
        Assert.Equal(31.0, first.DutyStdPercent, 6);
        Assert.Equal(4.5, first.BusCurrentStdMa, 6);
        Assert.Equal((byte)12, first.RpmStd);
        Assert.True(first.HasPower);
        LoadSweepPoint second = batch.Points[1];
        Assert.Equal(8, second.Index);
        Assert.Equal(-0.5, second.BusCurrentMa, 6);
        Assert.True(second.HasPower);
        Assert.Equal(LoadPointFlags.PartialPower, second.Flags);
    }

    [Fact]
    public void DecodeLoadSweepPointsAcceptsEmptyBatch()
    {
        byte[] payload = [0x80, 0x00, 0x00];
        byte[] raw = EscProtocol.BuildFrame(
            6, CommOpcode.LoadSweep, (byte)LoadSweepCommand.GetPoints, payload, CommFrameType.Response, CommStatus.Ok);

        LoadSweepPointBatch batch = EscProtocol.DecodeLoadSweepPoints(EscProtocol.Parse(raw));

        Assert.Equal((ushort)128, batch.FirstIndex);
        Assert.Empty(batch.Points);
    }

    [Theory]
    [InlineData(1, 18)]
    [InlineData(2, 3 + 3 * 16)]
    [InlineData(4, 3 + 3 * 16)]
    public void DecodeLoadSweepPointsRejectsInconsistentLength(byte count, int length)
    {
        byte[] payload = new byte[length];
        payload[2] = count;
        byte[] raw = EscProtocol.BuildFrame(
            6, CommOpcode.LoadSweep, (byte)LoadSweepCommand.GetPoints, payload, CommFrameType.Response, CommStatus.Ok);

        Assert.Throws<EscProtocolException>(() => EscProtocol.DecodeLoadSweepPoints(EscProtocol.Parse(raw)));
    }

    private static void WritePoint(
        Span<byte> target,
        ushort reference,
        ushort rpm,
        ushort dutyE4,
        ushort busMv,
        short busMaX10,
        ushort dutyStdE4,
        ushort busStdMaX10,
        byte rpmStd,
        LoadPointFlags flags)
    {
        BinaryPrimitives.WriteUInt16LittleEndian(target.Slice(0, 2), reference);
        BinaryPrimitives.WriteUInt16LittleEndian(target.Slice(2, 2), rpm);
        BinaryPrimitives.WriteUInt16LittleEndian(target.Slice(4, 2), dutyE4);
        BinaryPrimitives.WriteUInt16LittleEndian(target.Slice(6, 2), busMv);
        BinaryPrimitives.WriteInt16LittleEndian(target.Slice(8, 2), busMaX10);
        BinaryPrimitives.WriteUInt16LittleEndian(target.Slice(10, 2), dutyStdE4);
        BinaryPrimitives.WriteUInt16LittleEndian(target.Slice(12, 2), busStdMaX10);
        target[14] = rpmStd;
        target[15] = (byte)flags;
    }
}
