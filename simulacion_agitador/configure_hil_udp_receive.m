function configure_hil_udp_receive()
%CONFIGURE_HIL_UDP_RECEIVE Add the HIL UDP receive and PI wiring to sim_motor.
%
% The model sends simulated speed to the GUI bridge/proxy and receives the
% decoded MCU HIL output packet back as ASCII CSV. The parsed PWM command is
% used as inverter duty. If no valid packet is available, the plant uses a
% conservative fixed duty while the local PI is only logged as a reference.

modelDir = fileparts(mfilename("fullpath"));
modelPath = fullfile(modelDir, "sim_motor.mdl");
mdl = "sim_motor";

udpTargetPort = "5057";      % proxy/logger default; use 5055 to bypass it
simulinkLocalPort = "5058";  % fixed source/receive port for UDP responses
hilSampleTime = "0.02";      % below the 50 ms firmware HIL timeout
sixStepSampleTime = "1e-3";  % matches the Ts input used by six_step_ramp_rt

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

load_system(modelPath);

set_param(mdl + "/UDP Send", ...
    "Host", "127.0.0.1", ...
    "Port", udpTargetPort, ...
    "LocalAddress", "0.0.0.0", ...
    "LocalPort", simulinkLocalPort, ...
    "EnablePortSharing", "on", ...
    "OutputDatagramPacketSize", "64", ...
    "EnableBlockingMode", "off");

counter = mdl + "/" + sprintf("Counter\nFree-Running");
set_param(counter, "tsamp", hilSampleTime);

udpReceive = mdl + "/UDP Receive MCU";
replaceBlock("instrumentlib/UDP Receive", udpReceive, [1245 175 1375 230]);
set_param(udpReceive, ...
    "LocalAddress", "0.0.0.0", ...
    "LocalPort", simulinkLocalPort, ...
    "EnablePortSharing", "on", ...
    "Host", "127.0.0.1", ...
    "Port", udpTargetPort, ...
    "GetLatestData", "on", ...
    "DataSize", "[1, 256]", ...
    "DataType", "uint8", ...
    "ByteOrder", "big-endian", ...
    "EnableBlockingMode", "off", ...
    "Timeout", "0.001");

parser = mdl + "/Parse MCU UDP PI";
replaceBlock("simulink/User-Defined Functions/MATLAB Function", parser, [835 200 1045 335]);
setMatlabFunctionScript(parser, parserScript());
set_param(parser, "SystemSampleTime", hilSampleTime);

dutyDelay = mdl + "/HIL Duty Delay";
replaceBlock("simulink/Discrete/Unit Delay", dutyDelay, [1085 188 1120 222]);
set_param(dutyDelay, ...
    "SampleTime", hilSampleTime, ...
    "InitialCondition", "0.3");

speedSampler = mdl + "/HIL Speed ZOH";
replaceBlock("simulink/Discrete/Zero-Order Hold", speedSampler, [755 257 790 293]);
set_param(speedSampler, "SampleTime", hilSampleTime);

pwmFullScale = mdl + "/HIL PWM Full Scale";
replaceBlock("simulink/Sources/Constant", pwmFullScale, [755 315 800 345]);
set_param(pwmFullScale, ...
    "Value", "2000", ...
    "SampleTime", hilSampleTime);

fallbackDuty = mdl + "/HIL Fallback Duty";
replaceBlock("simulink/Sources/Constant", fallbackDuty, [755 370 800 400]);
set_param(fallbackDuty, ...
    "Value", "0.3", ...
    "SampleTime", hilSampleTime);

udpStatusTerminator = mdl + "/UDP Receive Status Terminator";
replaceBlock("simulink/Sinks/Terminator", udpStatusTerminator, [1415 206 1435 226]);

legacyDutyTerminator = mdl + "/Legacy Duty Terminator";
replaceBlock("simulink/Sinks/Terminator", legacyDutyTerminator, [-425 -82 -405 -62]);

addSink(mdl + "/MCU Packet To Workspace", "hil_mcu_packet", [1110 214 1240 246]);
addSink(mdl + "/MCU Duty To Workspace", "hil_mcu_duty", [1110 254 1240 286]);
addSink(mdl + "/Sim PI Duty To Workspace", "hil_sim_pi_duty", [1110 294 1240 326]);
addSink(mdl + "/MCU Packet Valid To Workspace", "hil_mcu_packet_valid", [1110 334 1240 366]);
addSink(mdl + "/SixStep Dsw To Workspace", "six_step_dsw", [205 -95 335 -63]);
addSink(mdl + "/SixStep RpmRef To Workspace", "six_step_rpm_ref", [205 -55 335 -23]);
addSink(mdl + "/SixStep FeHz To Workspace", "six_step_fe_hz", [205 -15 335 17]);
addSink(mdl + "/SixStep Sector To Workspace", "six_step_sector", [205 25 335 57]);

sixStep = mdl + "/MATLAB Function";
set_param(sixStep, "SystemSampleTime", sixStepSampleTime);
disconnectInport(sixStep, 2);

connect(udpReceive, 1, parser, 1);
connect(udpReceive, 2, udpStatusTerminator, 1);
connect(mdl + "/Constant3", 1, legacyDutyTerminator, 1);
connect(mdl + "/Constant5", 1, parser, 2);
connect(mdl + "/" + sprintf("PS-Simulink\nConverter"), 1, speedSampler, 1);
connect(speedSampler, 1, parser, 3);
connect(mdl + "/Manual Switch", 1, parser, 4);
connect(pwmFullScale, 1, parser, 5);
connect(fallbackDuty, 1, parser, 6);
connect(parser, 1, dutyDelay, 1);
connect(dutyDelay, 1, sixStep, 2);
connect(parser, 2, mdl + "/MCU Packet To Workspace", 1);
connect(parser, 3, mdl + "/MCU Duty To Workspace", 1);
connect(parser, 4, mdl + "/Sim PI Duty To Workspace", 1);
connect(parser, 5, mdl + "/MCU Packet Valid To Workspace", 1);
connectBranch(sixStep, 1, mdl + "/SixStep Dsw To Workspace", 1);
connect(sixStep, 2, mdl + "/SixStep RpmRef To Workspace", 1);
connect(sixStep, 3, mdl + "/SixStep FeHz To Workspace", 1);
connect(sixStep, 4, mdl + "/SixStep Sector To Workspace", 1);

deleteDanglingSignalLines([
    mdl + "/Constant5"
    mdl + "/Manual Switch"
    mdl + "/HIL Speed ZOH"
    mdl + "/HIL PWM Full Scale"
    mdl + "/HIL Fallback Duty"
    mdl + "/Parse MCU UDP PI"
    ]);

save_system(mdl);
fprintf("Updated %s with UDP receive, MCU packet parser, and PI duty loop.\n", modelPath);

    function replaceBlock(source, destination, position)
        if blockExists(destination)
            delete_block(destination);
        end
        add_block(source, destination, "Position", position);
    end

    function addSink(path, variableName, position)
        if blockExists(path)
            delete_block(path);
        end
        add_block("simulink/Sinks/To Workspace", path, ...
            "Position", position, ...
            "VariableName", variableName, ...
            "SaveFormat", "Array", ...
            "MaxDataPoints", "5000");
    end

    function tf = blockExists(path)
        tf = getSimulinkBlockHandle(path) ~= -1;
    end

    function disconnectInport(blockPath, portNumber)
        ports = get_param(blockPath, "PortHandles");
        line = get_param(ports.Inport(portNumber), "Line");
        if line ~= -1
            delete_line(line);
        end
    end

    function connect(srcBlock, srcPort, dstBlock, dstPort)
        srcPorts = get_param(srcBlock, "PortHandles");
        dstPorts = get_param(dstBlock, "PortHandles");
        dstLine = get_param(dstPorts.Inport(dstPort), "Line");
        if dstLine ~= -1
            delete_line(dstLine);
        end
        add_line(mdl, srcPorts.Outport(srcPort), dstPorts.Inport(dstPort), "autorouting", "on");
    end

    function connectBranch(srcBlock, srcPort, dstBlock, dstPort)
        srcPorts = get_param(srcBlock, "PortHandles");
        dstPorts = get_param(dstBlock, "PortHandles");
        dstLine = get_param(dstPorts.Inport(dstPort), "Line");
        if dstLine ~= -1
            delete_line(dstLine);
        end
        add_line(mdl, srcPorts.Outport(srcPort), dstPorts.Inport(dstPort), "autorouting", "on");
    end

    function setMatlabFunctionScript(blockPath, scriptText)
        root = sfroot;
        chart = root.find("-isa", "Stateflow.EMChart", "Path", char(blockPath));
        if isempty(chart)
            error("Could not find MATLAB Function chart for %s.", blockPath);
        end
        chart.Script = scriptText;
    end

    function deleteDanglingSignalLines(sourceBlocks)
        lineHandles = find_system(mdl, "FindAll", "on", "Type", "Line");
        for lineIndex = 1:numel(lineHandles)
            try
                lineHandle = lineHandles(lineIndex);
                if ~ishandle(lineHandle) || strcmp(get_param(lineHandle, "LineType"), "Connection")
                    continue;
                end

                srcPort = get_param(lineHandle, "SrcPortHandle");
                dstPorts = get_param(lineHandle, "DstPortHandle");
                if srcPort == -1 || ~(isempty(dstPorts) || all(dstPorts == -1))
                    continue;
                end

                srcBlock = string(get_param(srcPort, "Parent"));
                if any(srcBlock == sourceBlocks)
                    delete_line(lineHandle);
                end
            catch
                % Ignore stale line handles while deleting selected dangling branches.
            end
        end
    end

    function scriptText = parserScript()
        scriptText = strjoin([
            "function [duty, packet, mcu_duty, pi_duty, valid] = hil_udp_packet_pi(bytes, target_rpm, omega_rad_s, enable, pwm_full_scale, fallback_duty)"
            "%#codegen"
            ""
            "persistent integral"
            "if isempty(integral)"
            "    integral = 0.0;"
            "end"
            ""
            "packet = zeros(1, 16);"
            "numbers = zeros(1, 15);"
            "valid = false;"
            "mcu_duty = 0.0;"
            "full_scale = max(1.0, double(pwm_full_scale));"
            "fallback = min(1.0, max(0.0, double(fallback_duty)));"
            ""
            "% Local PI reference in duty units. It is logged, not used as the"
            "% plant command unless the MCU packet path is made valid upstream."
            "Ts = 0.02;"
            "Kp = 6.0e-4;"
            "Ki = 2.0e-4;"
            "rpm_meas = omega_rad_s * 60.0 / (2.0*pi);"
            "err = target_rpm - rpm_meas;"
            "if enable == 0"
            "    integral = 0.0;"
            "end"
            "u = Kp * err + integral;"
            "pi_duty = min(1.0, max(0.0, u));"
            "if (u > 0.0 && u < 1.0) || (u <= 0.0 && err > 0.0) || (u >= 1.0 && err < 0.0)"
            "    integral = integral + Ki * err * Ts;"
            "end"
            "pi_duty = min(1.0, max(0.0, Kp * err + integral));"
            ""
            "raw = uint8(bytes(:));"
            "n = numel(raw);"
            "if n >= 4"
            "    is_ok = (raw(1) == 111 || raw(1) == 79) && ... % o/O"
            "            (raw(2) == 107 || raw(2) == 75) && ... % k/K"
            "            raw(3) == 44;"
            "    if is_ok"
            "        index = 1;"
            "        value = 0.0;"
            "        sign_value = 1.0;"
            "        fraction = 0.1;"
            "        in_fraction = false;"
            "        have_digit = false;"
            "        for k = 4:n"
            "            c = raw(k);"
            "            if c == 0 || c == 10 || c == 13"
            "                break;"
            "            elseif c == 44"
            "                if have_digit && index <= 15"
            "                    numbers(index) = sign_value * value;"
            "                    index = index + 1;"
            "                end"
            "                value = 0.0;"
            "                sign_value = 1.0;"
            "                fraction = 0.1;"
            "                in_fraction = false;"
            "                have_digit = false;"
            "            elseif c == 45"
            "                sign_value = -1.0;"
            "            elseif c == 46"
            "                in_fraction = true;"
            "            elseif c >= 48 && c <= 57"
            "                have_digit = true;"
            "                digit = double(c - 48);"
            "                if in_fraction"
            "                    value = value + digit * fraction;"
            "                    fraction = fraction * 0.1;"
            "                else"
            "                    value = value * 10.0 + digit;"
            "                end"
            "            end"
            "        end"
            "        if have_digit && index <= 15"
            "            numbers(index) = sign_value * value;"
            "            index = index + 1;"
            "        end"
            "        valid = index > 15;"
            "    end"
            "end"
            ""
            "if valid"
            "    packet(1) = 1.0;"
            "    for k = 1:15"
            "        packet(k + 1) = numbers(k);"
            "    end"
            "    mcu_duty = min(1.0, max(0.0, numbers(7) / full_scale));"
            "end"
            ""
            "if enable ~= 0"
            "    if valid"
            "        duty = mcu_duty;"
            "    else"
            "        duty = fallback;"
            "    end"
            "else"
            "    duty = 0.0;"
            "end"
            "end"
            ], newline);
    end
end
