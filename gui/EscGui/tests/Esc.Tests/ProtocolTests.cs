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
    public void DecodeTelemetryReadsExtendedCurrentMetadata()
    {
        byte[] payload = new byte[15];
        payload[0] = (byte)LogParam.CurrentU;
        BitConverter.GetBytes(1200).CopyTo(payload, 1);
        BitConverter.GetBytes(5678u).CopyTo(payload, 5);
        BitConverter.GetBytes((ushort)700).CopyTo(payload, 9);
        payload[11] = (byte)(TelemetryQuality.Valid | TelemetryQuality.Calibrated);
        BitConverter.GetBytes((ushort)25).CopyTo(payload, 12);
        payload[14] = 3;

        byte[] raw = EscProtocol.BuildFrame(
            2, CommOpcode.TelemetryEvent, (byte)LogParam.CurrentU,
            payload, CommFrameType.Event, CommStatus.Ok);
        TelemetrySample sample = EscProtocol.DecodeTelemetry(
            EscProtocol.Parse(raw), DateTimeOffset.UnixEpoch);

        Assert.Equal("current_u", sample.Variable);
        Assert.Equal("mA", sample.Unit);
        Assert.Equal(1200, sample.RawValue);
        Assert.Equal((ushort)700, sample.AdcRawValue);
        Assert.True(sample.IsValid);
        Assert.Equal((ushort)25, sample.ValidSampleCount);
        Assert.Equal((sbyte)3, sample.CommutationStep);
    }

    [Fact]
    public void DecodeTelemetryRejectsPartialExtendedPayload()
    {
        byte[] payload = new byte[12];
        payload[0] = (byte)LogParam.CurrentU;
        byte[] raw = EscProtocol.BuildFrame(
            2, CommOpcode.TelemetryEvent, (byte)LogParam.CurrentU,
            payload, CommFrameType.Event, CommStatus.Ok);

        Assert.Throws<EscProtocolException>(() =>
            EscProtocol.DecodeTelemetry(EscProtocol.Parse(raw), DateTimeOffset.UnixEpoch));
    }

    [Fact]
    public void DecodeStatusParsesIna219PowerExtension()
    {
        byte[] payload = new byte[24];
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
    }

    [Fact]
    public void DecodeStatusLegacyPayloadHasNoPowerReading()
    {
        byte[] raw = EscProtocol.BuildFrame(3, CommOpcode.GetStatus, 0, new byte[10], CommFrameType.Response, CommStatus.Ok);

        Assert.Null(EscProtocol.DecodeStatus(EscProtocol.Parse(raw)).Power);
    }

    [Theory]
    [InlineData(LogParam.BusVoltage, "bus_voltage", "mV", 12040)]
    [InlineData(LogParam.BusCurrent, "bus_current", "mA", -250)]
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
    public void DecodeValidationReferenceReadsStructuralControllerValues()
    {
        byte[] payload = new byte[20];
        payload[0] = 1;
        payload[1] = 2;
        BitConverter.GetBytes((ushort)18_000).CopyTo(payload, 2);
        BitConverter.GetBytes((ushort)2_000).CopyTo(payload, 4);
        BitConverter.GetBytes(180_000u).CopyTo(payload, 6);
        BitConverter.GetBytes((ushort)14_000).CopyTo(payload, 10);
        BitConverter.GetBytes((ushort)200).CopyTo(payload, 12);
        BitConverter.GetBytes(2_000u).CopyTo(payload, 14);
        BitConverter.GetBytes((ushort)100).CopyTo(payload, 18);

        byte[] raw = EscProtocol.BuildFrame(
            4, CommOpcode.GetValidationReference, 0, payload, CommFrameType.Response, CommStatus.Ok);
        ValidationReference reference = EscProtocol.DecodeValidationReference(EscProtocol.Parse(raw));

        Assert.Equal((ushort)18_000, reference.PwmFrequencyHz);
        Assert.Equal((ushort)2_000, reference.PwmArrCounts);
        Assert.Equal(180_000u, reference.SpeedTimerHz);
        Assert.Equal(0.002, reference.ControllerDtSeconds);
        Assert.Equal((ushort)100, reference.MinimumPwmCounts);
    }

    [Fact]
    public void HilInputsPayloadUsesExpectedWireShape()
    {
        byte[] payload = EscProtocol.HilInputsPayload(new HilInputs(1500, -4, 0xA5, true));

        Assert.Equal([0xDC, 0x05, 0x00, 0x00, 0xFC, 0xFF, 0xA5, 0x01], payload);
    }

    [Fact]
    public void RpmPiGainUsesNewParameterAndHundredthsPayload()
    {
        byte[] payload = EscProtocol.ConfigPayload(ConfigParam.KpRpm, 0.28);

        Assert.Equal((byte)0x0A, (byte)ConfigParam.KpRpm);
        Assert.Equal([0x1C, 0x00], payload);
        Assert.Throws<ArgumentOutOfRangeException>(() => EscProtocol.ConfigPayload(ConfigParam.Kp, 0.28));
    }

    [Fact]
    public void HilStartPayloadUsesLittleEndianSessionTimeout()
    {
        byte[] payload = EscProtocol.HilStartPayload(500);

        Assert.Equal([0xF4, 0x01], payload);
    }

    [Theory]
    [InlineData(HilExecutionMode.Periodic, 0)]
    [InlineData(HilExecutionMode.Stepped, 1)]
    public void HilStartPayloadAppendsExecutionMode(HilExecutionMode mode, byte expectedMode)
    {
        byte[] payload = EscProtocol.HilStartPayload(0x1234, mode);

        Assert.Equal([0x34, 0x12, expectedMode], payload);
    }

    [Fact]
    public void HilStartPayloadRejectsUnknownExecutionMode()
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => EscProtocol.HilStartPayload(500, (HilExecutionMode)2));
    }

    [Fact]
    public void ValidatedHilInputsPayloadAppendsRunAndSourceSequence()
    {
        byte[] payload = EscProtocol.HilInputsPayload(new HilInputs(1500, -4, 0xA5, true, 0x10203040, 0x50607080));

        Assert.Equal(
            [0xDC, 0x05, 0x00, 0x00, 0xFC, 0xFF, 0xA5, 0x01, 0x40, 0x30, 0x20, 0x10, 0x80, 0x70, 0x60, 0x50],
            payload);
    }

    [Fact]
    public void DecodeHilOutputsReadsResponsePayload()
    {
        byte[] payload = new byte[15];
        BitConverter.GetBytes(1234u).CopyTo(payload, 0);
        payload[4] = 6;
        payload[5] = (byte)ControlRuntimeMode.HilSim;
        BitConverter.GetBytes((ushort)2000).CopyTo(payload, 6);
        BitConverter.GetBytes((ushort)1800).CopyTo(payload, 8);
        BitConverter.GetBytes((ushort)900).CopyTo(payload, 10);
        payload[12] = unchecked((byte)-1);
        payload[13] = 3;
        payload[14] = 0;

        byte[] raw = EscProtocol.BuildFrame(4, CommOpcode.HilGetOutputs, 0, payload, CommFrameType.Response, CommStatus.Ok);
        HilOutputs outputs = EscProtocol.DecodeHilOutputs(EscProtocol.Parse(raw));

        Assert.Equal(1234u, outputs.TargetTickMs);
        Assert.Equal(ControlRuntimeMode.HilSim, outputs.Mode);
        Assert.Equal(1800, outputs.MeasuredRpm);
        Assert.Equal(-1, outputs.CommutationStep);
    }

    [Fact]
    public void DecodeHilOutputsReadsValidationProvenanceExtension()
    {
        byte[] payload = new byte[43];
        BitConverter.GetBytes(1234u).CopyTo(payload, 0);
        payload[5] = (byte)ControlRuntimeMode.HilSim;
        BitConverter.GetBytes((ushort)900).CopyTo(payload, 10);
        BitConverter.GetBytes(11u).CopyTo(payload, 15);
        BitConverter.GetBytes(12u).CopyTo(payload, 19);
        BitConverter.GetBytes(13u).CopyTo(payload, 23);
        BitConverter.GetBytes(14u).CopyTo(payload, 27);
        BitConverter.GetBytes(15u).CopyTo(payload, 31);
        BitConverter.GetBytes(16u).CopyTo(payload, 35);
        BitConverter.GetBytes(17u).CopyTo(payload, 39);

        byte[] raw = EscProtocol.BuildFrame(4, CommOpcode.HilGetOutputs, 0, payload, CommFrameType.Response, CommStatus.Ok);
        HilOutputs outputs = EscProtocol.DecodeHilOutputs(EscProtocol.Parse(raw));

        Assert.True(outputs.HasValidationProvenance);
        Assert.Equal(11u, outputs.AcceptedRunId);
        Assert.Equal(12u, outputs.AcceptedSourceSequence);
        Assert.Equal(13u, outputs.AcceptedGeneration);
        Assert.Equal(14u, outputs.AppliedRunId);
        Assert.Equal(15u, outputs.AppliedSourceSequence);
        Assert.Equal(16u, outputs.OutputGeneration);
        Assert.Equal(17u, outputs.PwmUpdateTickMs);
    }

    [Fact]
    public void HilStepOpcodeHasAssignedWireValue()
    {
        Assert.Equal(0x44, (byte)CommOpcode.HilStep);
    }

    [Fact]
    public void HilStepPayloadUsesExactLittleEndianWireShape()
    {
        var request = new HilStepRequest(
            0x1234,
            unchecked((short)0xFEDC),
            0xA5,
            true,
            0x10203040,
            0x50607080,
            0x0304);

        byte[] payload = EscProtocol.HilStepPayload(request);

        Assert.Equal(
            [
                0x34, 0x12,
                0x00, 0x00,
                0xDC, 0xFE,
                0xA5, 0x01,
                0x40, 0x30, 0x20, 0x10,
                0x80, 0x70, 0x60, 0x50,
                0x04, 0x03
            ],
            payload);
    }

    [Theory]
    [InlineData(0u, 1u, 1)]
    [InlineData(1u, 0u, 1)]
    [InlineData(1u, 1u, 0)]
    [InlineData(1u, 1u, 1001)]
    public void HilStepPayloadRejectsInvalidProvenanceOrStepCount(uint runId, uint sourceSequence, ushort steps)
    {
        var request = new HilStepRequest(1000, -1, 0, true, runId, sourceSequence, steps);

        Assert.Throws<ArgumentOutOfRangeException>(() => EscProtocol.HilStepPayload(request));
    }

    [Theory]
    [InlineData(1)]
    [InlineData(1000)]
    public void HilStepPayloadAcceptsStepCountBoundaries(ushort steps)
    {
        byte[] payload = EscProtocol.HilStepPayload(new HilStepRequest(1000, 0, 0, true, 1, 1, steps));

        Assert.Equal(18, payload.Length);
        Assert.Equal(steps, BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(16, 2)));
    }

    [Fact]
    public void HilStepPayloadRejectsDisabledInput()
    {
        var request = new HilStepRequest(1000, 0, 0, false, 1, 1, 10);

        Assert.Throws<ArgumentException>(() => EscProtocol.HilStepPayload(request));
    }

    [Fact]
    public void DecodeHilStepResultReadsExactExtendedResponse()
    {
        byte[] payload = BuildHilStepResponsePayload();
        byte[] raw = EscProtocol.BuildFrame(9, CommOpcode.HilStep, 0, payload, CommFrameType.Response, CommStatus.Ok);

        HilStepResult result = EscProtocol.DecodeHilStepResult(EscProtocol.Parse(raw));

        Assert.Equal(0x10203040u, result.Outputs.TargetTickMs);
        Assert.Equal((byte)6, result.Outputs.AppState);
        Assert.Equal(ControlRuntimeMode.HilSim, result.Outputs.Mode);
        Assert.Equal((ushort)0x1234, result.Outputs.SetpointRpm);
        Assert.Equal((ushort)0x5678, result.Outputs.MeasuredRpm);
        Assert.Equal((ushort)0x9ABC, result.Outputs.PwmCommand);
        Assert.Equal(-2, result.Outputs.CommutationStep);
        Assert.Equal((byte)0x5A, result.Outputs.Flags);
        Assert.True(result.Outputs.TimedOut);
        Assert.True(result.Outputs.HasValidationProvenance);
        Assert.Equal(0x11121314u, result.Outputs.AcceptedRunId);
        Assert.Equal(0x21222324u, result.Outputs.AcceptedSourceSequence);
        Assert.Equal(0x31323334u, result.Outputs.AcceptedGeneration);
        Assert.Equal(0x41424344u, result.Outputs.AppliedRunId);
        Assert.Equal(0x51525354u, result.Outputs.AppliedSourceSequence);
        Assert.Equal(0x61626364u, result.Outputs.OutputGeneration);
        Assert.Equal(0x71727374u, result.Outputs.PwmUpdateTickMs);
        Assert.Equal((ushort)0x0102, result.RequestedSteps);
        Assert.Equal((ushort)0x0304, result.AppliedSteps);
        Assert.Equal((byte)0x81, result.Flags);
        Assert.True(result.Replayed);
    }

    [Fact]
    public void DecodeHilStepResultClearsReplayedWhenFlagBitIsAbsent()
    {
        byte[] payload = BuildHilStepResponsePayload();
        payload[47] = 0x80;
        byte[] raw = EscProtocol.BuildFrame(9, CommOpcode.HilStep, 0, payload, CommFrameType.Response, CommStatus.Ok);

        HilStepResult result = EscProtocol.DecodeHilStepResult(EscProtocol.Parse(raw));

        Assert.False(result.Replayed);
    }

    [Theory]
    [InlineData(47)]
    [InlineData(49)]
    public void DecodeHilStepResultRejectsNonExactPayloadLength(int length)
    {
        byte[] raw = EscProtocol.BuildFrame(
            9,
            CommOpcode.HilStep,
            0,
            new byte[length],
            CommFrameType.Response,
            CommStatus.Ok);

        Assert.Throws<EscProtocolException>(() => EscProtocol.DecodeHilStepResult(EscProtocol.Parse(raw)));
    }

    [Fact]
    public void DecodeHilStepResultRejectsWrongOpcode()
    {
        byte[] raw = EscProtocol.BuildFrame(
            9,
            CommOpcode.HilGetOutputs,
            0,
            new byte[48],
            CommFrameType.Response,
            CommStatus.Ok);

        Assert.Throws<EscProtocolException>(() => EscProtocol.DecodeHilStepResult(EscProtocol.Parse(raw)));
    }

    [Fact]
    public void DecodeHilStepResultRejectsErrorStatus()
    {
        byte[] raw = EscProtocol.BuildFrame(
            9,
            CommOpcode.HilStep,
            0,
            new byte[48],
            CommFrameType.Response,
            CommStatus.InvalidState);

        Assert.Throws<EscProtocolException>(() => EscProtocol.DecodeHilStepResult(EscProtocol.Parse(raw)));
    }

    [Fact]
    public void DecodeValidationReferenceKeepsLegacyCapabilitiesDisabled()
    {
        byte[] payload = BuildValidationReferencePayload(20);
        byte[] raw = EscProtocol.BuildFrame(
            4, CommOpcode.GetValidationReference, 0, payload, CommFrameType.Response, CommStatus.Ok);

        ValidationReference reference = EscProtocol.DecodeValidationReference(EscProtocol.Parse(raw));

        Assert.Equal((byte)0, reference.CapabilityFlags);
        Assert.Equal((byte)0, reference.HilStepOperationVersion);
        Assert.Equal((ushort)0, reference.MaximumHilSteps);
        Assert.False(reference.SupportsDeterministicHil);
    }

    [Fact]
    public void DecodeValidationReferenceReadsDeterministicHilCapabilityExtension()
    {
        byte[] payload = BuildValidationReferencePayload(24);
        payload[20] = 0x81;
        payload[21] = 3;
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(22, 2), 1000);
        byte[] raw = EscProtocol.BuildFrame(
            4, CommOpcode.GetValidationReference, 0, payload, CommFrameType.Response, CommStatus.Ok);

        ValidationReference reference = EscProtocol.DecodeValidationReference(EscProtocol.Parse(raw));

        Assert.Equal((byte)0x81, reference.CapabilityFlags);
        Assert.Equal((byte)3, reference.HilStepOperationVersion);
        Assert.Equal((ushort)1000, reference.MaximumHilSteps);
        Assert.True(reference.SupportsDeterministicHil);
    }

    [Fact]
    public void DecodeValidationReferenceDoesNotInferCapabilityFromExtensionPresence()
    {
        byte[] payload = BuildValidationReferencePayload(24);
        payload[20] = 0x80;
        payload[21] = 1;
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(22, 2), 1000);
        byte[] raw = EscProtocol.BuildFrame(
            4, CommOpcode.GetValidationReference, 0, payload, CommFrameType.Response, CommStatus.Ok);

        ValidationReference reference = EscProtocol.DecodeValidationReference(EscProtocol.Parse(raw));

        Assert.False(reference.SupportsDeterministicHil);
    }

    [Theory]
    [InlineData(19)]
    [InlineData(21)]
    [InlineData(23)]
    [InlineData(25)]
    public void DecodeValidationReferenceRejectsUndefinedPayloadLengths(int length)
    {
        byte[] raw = EscProtocol.BuildFrame(
            4,
            CommOpcode.GetValidationReference,
            0,
            new byte[length],
            CommFrameType.Response,
            CommStatus.Ok);

        Assert.Throws<EscProtocolException>(() => EscProtocol.DecodeValidationReference(EscProtocol.Parse(raw)));
    }

    private static byte[] BuildHilStepResponsePayload()
    {
        var payload = new byte[48];
        BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(0, 4), 0x10203040);
        payload[4] = 6;
        payload[5] = (byte)ControlRuntimeMode.HilSim;
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(6, 2), 0x1234);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(8, 2), 0x5678);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(10, 2), 0x9ABC);
        payload[12] = unchecked((byte)-2);
        payload[13] = 0x5A;
        payload[14] = 1;
        BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(15, 4), 0x11121314);
        BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(19, 4), 0x21222324);
        BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(23, 4), 0x31323334);
        BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(27, 4), 0x41424344);
        BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(31, 4), 0x51525354);
        BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(35, 4), 0x61626364);
        BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(39, 4), 0x71727374);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(43, 2), 0x0102);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(45, 2), 0x0304);
        payload[47] = 0x81;
        return payload;
    }

    private static byte[] BuildValidationReferencePayload(int length)
    {
        var payload = new byte[length];
        if (length < 20)
        {
            return payload;
        }

        payload[0] = 1;
        payload[1] = 2;
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(2, 2), 18_000);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(4, 2), 2_000);
        BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(6, 4), 180_000);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(10, 2), 14_000);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(12, 2), 200);
        BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(14, 4), 2_000);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(18, 2), 100);
        return payload;
    }
}
