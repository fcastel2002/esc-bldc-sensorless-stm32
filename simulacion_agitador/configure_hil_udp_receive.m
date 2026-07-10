function configure_hil_udp_receive()
%CONFIGURE_HIL_UDP_RECEIVE Configure HIL UDP and ideal rotor commutation.
%
% The model sends simulated speed to the GUI bridge/proxy and receives the
% decoded MCU HIL output packet back as ASCII CSV. The parsed PWM command is
% used as inverter duty. Sector selection is local to Simulink and follows
% the ideal mechanical rotor position, not a time ramp or BEMF crossings.

modelDir = fileparts(mfilename("fullpath"));
modelPath = fullfile(modelDir, "sim_motor.mdl");
mdl = "sim_motor";

udpTargetPort = "5057";      % proxy/logger default; use 5055 to bypass it
simulinkLocalPort = "5058";  % fixed source/receive port for UDP responses
hilSampleTime = "0.02";      % below the 50 ms firmware HIL timeout

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
ensureBlock("instrumentlib/UDP Receive", udpReceive, [1245 175 1375 230]);
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
ensureBlock("simulink/User-Defined Functions/MATLAB Function", parser, [835 200 1045 335]);
setMatlabFunctionScript(parser, parserScript());
set_param(parser, "SystemSampleTime", hilSampleTime);

dutyDelay = mdl + "/HIL Duty Delay";
ensureBlock("simulink/Discrete/Unit Delay", dutyDelay, [1085 188 1120 222]);
set_param(dutyDelay, ...
    "SampleTime", hilSampleTime, ...
    "InitialCondition", "0.3");

speedSampler = mdl + "/HIL Speed ZOH";
ensureBlock("simulink/Discrete/Zero-Order Hold", speedSampler, [755 257 790 293]);
set_param(speedSampler, "SampleTime", hilSampleTime);

pwmFullScale = mdl + "/HIL PWM Full Scale";
ensureBlock("simulink/Sources/Constant", pwmFullScale, [755 315 800 345]);
set_param(pwmFullScale, ...
    "Value", "2000", ...
    "SampleTime", hilSampleTime);

fallbackDuty = mdl + "/HIL Fallback Duty";
ensureBlock("simulink/Sources/Constant", fallbackDuty, [755 370 800 400]);
set_param(fallbackDuty, ...
    "Value", "0.3", ...
    "SampleTime", hilSampleTime);

udpStatusTerminator = mdl + "/UDP Receive Status Terminator";
ensureBlock("simulink/Sinks/Terminator", udpStatusTerminator, [1415 206 1435 226]);

addSink(mdl + "/MCU Packet To Workspace", "hil_mcu_packet", [1110 214 1240 246]);
addSink(mdl + "/MCU Duty To Workspace", "hil_mcu_duty", [1110 254 1240 286]);
addSink(mdl + "/Sim PI Duty To Workspace", "hil_sim_pi_duty", [1110 294 1240 326]);
addSink(mdl + "/MCU Packet Valid To Workspace", "hil_mcu_packet_valid", [1110 334 1240 366]);
addSink(mdl + "/SixStep Dsw To Workspace", "six_step_dsw", [205 -95 335 -63]);
addSink(mdl + "/SixStep Sector To Workspace", "six_step_sector", [205 25 335 57]);
motorRpmLog = mdl + "/Motor RPM To Workspace";
addSink(motorRpmLog, "motor_rpm", [970 -5 1100 27]);

connect(udpReceive, 1, parser, 1);
connect(udpReceive, 2, udpStatusTerminator, 1);
connect(mdl + "/Constant5", 1, parser, 2);
connect(mdl + "/" + sprintf("PS-Simulink\nConverter"), 1, speedSampler, 1);
connect(speedSampler, 1, parser, 3);
connect(mdl + "/Manual Switch", 1, parser, 4);
connect(pwmFullScale, 1, parser, 5);
connect(fallbackDuty, 1, parser, 6);
connect(parser, 1, dutyDelay, 1);
connect(parser, 2, mdl + "/MCU Packet To Workspace", 1);
connect(parser, 3, mdl + "/MCU Duty To Workspace", 1);
connect(parser, 4, mdl + "/Sim PI Duty To Workspace", 1);
connect(parser, 5, mdl + "/MCU Packet Valid To Workspace", 1);
connect(mdl + "/Angular Velocity Conversion", 1, motorRpmLog, 1);
configureIdealCommutator();
configureFixedStepSolver();

save_system(mdl);
fprintf("Updated %s with HIL UDP, MCU duty selection, and ideal rotor commutation.\n", modelPath);

    function ensureBlock(source, destination, position)
        if ~blockExists(destination)
            add_block(source, destination, "Position", position);
        end
    end

    function addSink(path, variableName, position)
        if ~blockExists(path)
            add_block("simulink/Sinks/To Workspace", path, ...
                "Position", position, ...
                "SaveFormat", "Array");
        end
        set_param(path, ...
            "VariableName", variableName, ...
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

    function configureIdealCommutator()
        legacyCommutator = mdl + "/MATLAB Function";
        commutator = mdl + "/Ideal Six-Step Commutator";
        dswLog = mdl + "/SixStep Dsw To Workspace";
        sectorLog = mdl + "/SixStep Sector To Workspace";

        % Remove all legacy inputs before Stateflow changes the function
        % signature. In particular, target_rpm remains connected to the HIL PI
        % parser but no longer selects the inverter sector.
        disconnectAllInports(legacyCommutator);
        disconnectAllInports(commutator);
        disconnectInport(sectorLog, 1);
        deleteBlockIfExists(mdl + "/SixStep RpmRef To Workspace");
        deleteBlockIfExists(mdl + "/SixStep FeHz To Workspace");
        deleteBlockIfExists(mdl + "/Constant4");
        deleteBlockIfExists(mdl + "/Constant6");

        if blockExists(commutator)
            deleteBlockIfExists(legacyCommutator);
            setMatlabFunctionScript(commutator, commutatorScript());
        elseif blockExists(legacyCommutator)
            setMatlabFunctionScript(legacyCommutator, commutatorScript());
            set_param(legacyCommutator, "Name", "Ideal Six-Step Commutator");
        else
            add_block("simulink/User-Defined Functions/MATLAB Function", ...
                commutator, "Position", [-315 -62 -190 -3]);
            setMatlabFunctionScript(commutator, commutatorScript());
        end

        % The angle source is a Simscape rotational position measurement. State
        % its physical unit at the Simulink boundary and sample it at the
        % model's existing 1 ms maximum step; this is position-based, not a
        % temporal commutation ramp.
        angleConverter = mdl + "/" + sprintf("PS-Simulink\nConverter1");
        set_param(angleConverter, "Unit", "rad");
        set_param(commutator, "SystemSampleTime", hilSampleTime);

        % Upper switch input is 0 and lower input is explicitly 1. In a Manual
        % Switch, CurrentSetting=0 selects the lower input, so local drive is
        % enabled by default while an operator can select the upper 0 input.
        set_param(mdl + "/Zero", "Value", "0");
        set_param(mdl + "/Zero1", "Value", "1");
        set_param(mdl + "/Manual Switch", "CurrentSetting", "0");

        % Position-based commutation takes angle, HIL-selected duty, and enable.
        connect(angleConverter, 1, commutator, 1);
        connect(mdl + "/HIL Duty Delay", 1, commutator, 2);
        connect(mdl + "/Manual Switch", 1, commutator, 3);
        connect(commutator, 1, mdl + "/BLDC Average-Value Inverter", 1);
        connectBranch(commutator, 1, dswLog, 1);
        connect(commutator, 2, sectorLog, 1);

        deleteBemfBranch();
        deleteDanglingSignalLines();
    end

    function disconnectAllInports(blockPath)
        if ~blockExists(blockPath)
            return;
        end
        ports = get_param(blockPath, "PortHandles");
        for portHandle = reshape(ports.Inport, 1, [])
            line = get_param(portHandle, "Line");
            if line ~= -1
                delete_line(line);
            end
        end
    end

    function deleteBlockIfExists(path)
        if blockExists(path)
            delete_block(path);
        end
    end

    function deleteBemfBranch()
        bemfBlocks = [
            mdl + "/" + sprintf("Compare\nTo Zero")
            mdl + "/" + sprintf("Compare\nTo Zero1")
            mdl + "/" + sprintf("Compare\nTo Zero2")
            mdl + "/" + sprintf("PS-Simulink\nConverter3")
            mdl + "/" + sprintf("PS-Simulink\nConverter4")
            mdl + "/" + sprintf("PS-Simulink\nConverter5")
            mdl + "/Voltage Sensor"
            mdl + "/Voltage Sensor1"
            mdl + "/Voltage Sensor2"
            mdl + "/Resistor"
            mdl + "/Resistor1"
            mdl + "/Resistor2"
            ];

        for blockPath = bemfBlocks'
            deleteBlockIfExists(blockPath);
        end

        % The neutral-voltage sum/gain only supported the removed BEMF branch.
        deleteBlockIfExists(mdl + "/Sum");
        deleteBlockIfExists(mdl + "/Gain");
        rewireLineVoltageMeasurement();
        deleteBemfOrphanConnections();
    end

    function rewireLineVoltageMeasurement()
        source = mdl + "/" + sprintf("Controlled Voltage Source\n(Three-Phase)");
        motor = mdl + "/BLDC";
        lineSensor = mdl + "/" + sprintf("Line Voltage Sensor\n(Three-Phase)");
        sourcePorts = get_param(source, "PortHandles");
        motorPorts = get_param(motor, "PortHandles");
        sensorPorts = get_param(lineSensor, "PortHandles");

        % Remove every segment of each former phase network so the stale BEMF
        % branches cannot survive as graphical stubs. Then rebuild the intended
        % three-terminal node: voltage source, BLDC phase, and line sensor.
        for phase = 1:3
            lineHandles = [
                get_param(sourcePorts.RConn(phase), "Line")
                get_param(motorPorts.LConn(phase), "Line")
                get_param(sensorPorts.LConn(phase), "Line")
                ];
            for line = unique(lineHandles(:)).'
                try
                    if line ~= -1 && ishandle(line)
                        delete_line(line);
                    end
                catch
                    % Deleting a neighboring physical segment can invalidate
                    % this handle; the rebuilt node below is authoritative.
                end
            end
        end
        for phase = 1:3
            add_line(mdl, sourcePorts.RConn(phase), motorPorts.LConn(phase), "autorouting", "on");
            add_line(mdl, sourcePorts.RConn(phase), sensorPorts.LConn(phase), "autorouting", "on");
        end
    end

    function deleteBemfOrphanConnections()
        lineHandles = find_system(mdl, "FindAll", "on", "Type", "Line");
        for lineIndex = 1:numel(lineHandles)
            try
                lineHandle = lineHandles(lineIndex);
                if ~strcmpi(get_param(lineHandle, "LineType"), "Connection")
                    continue;
                end

                points = get_param(lineHandle, "Points");
                % The deleted BEMF sensors occupied the right-hand column
                % beginning at x=500, y=160. Their residual graphic segments
                % have no terminals after the phase nodes are rebuilt.
                if ~isempty(points) && all(points(:, 1) >= 500) && all(points(:, 2) >= 160)
                    delete_line(lineHandle);
                end
            catch
                % Ignore stale physical line handles while the BEMF remnants go away.
            end
        end
    end

    function deleteDanglingSignalLines()
        lineHandles = find_system(mdl, "FindAll", "on", "Type", "Line");
        for lineIndex = 1:numel(lineHandles)
            try
                lineHandle = lineHandles(lineIndex);
                if ~ishandle(lineHandle) || strcmpi(get_param(lineHandle, "LineType"), "Connection")
                    continue;
                end

                srcPort = get_param(lineHandle, "SrcPortHandle");
                dstPorts = get_param(lineHandle, "DstPortHandle");
                if srcPort == -1 || isempty(dstPorts) || all(dstPorts == -1)
                    delete_line(lineHandle);
                end
            catch
                % Ignore stale line handles while selected dangling lines are removed.
            end
        end
    end

    function scriptText = commutatorScript()
        scriptText = strjoin([
            "function [Dsw, sector] = ideal_six_step_commutator(theta_m, duty, enable)"
            "%#codegen"
            ""
            "% BLDC block has two pole pairs. theta_m arrives in radians."
            "pole_pairs = 2.0;"
            "electrical_offset = 0.0;"
            "Dsw = zeros(1, 6);"
            "sector = 0.0;"
            ""
            "if enable == 0"
            "    return;"
            "end"
            ""
            "d = min(1.0, max(0.0, double(duty)));"
            "theta_e = mod(pole_pairs * double(theta_m) + electrical_offset, 2.0*pi);"
            "sector = floor(theta_e / (2.0*pi/6.0)) + 1.0;"
            "if sector > 6.0"
            "    sector = 6.0;"
            "end"
            ""
            "switch sector"
            "    case 1"
            "        Dsw = [d 0 0 0 0 1]; % A+ C-"
            "    case 2"
            "        Dsw = [d 0 0 1 0 0]; % A+ B-"
            "    case 3"
            "        Dsw = [0 0 0 1 d 0]; % C+ B-"
            "    case 4"
            "        Dsw = [0 1 0 0 d 0]; % C+ A-"
            "    case 5"
            "        Dsw = [0 1 d 0 0 0]; % B+ A-"
            "    otherwise"
            "        Dsw = [0 0 d 0 0 1]; % B+ C-"
            "end"
            ], newline);
    end

    function configureFixedStepSolver()
        set_param(mdl, "Solver", "FixedStepAuto");
        set_param(mdl, "FixedStep", hilSampleTime);
        set_param(mdl, "MaxStep", hilSampleTime);
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
            "start = 1;"
            "% UDP Receive prepends the datagram length to uint8 payloads."
            "if n >= 5 && raw(1) ~= 111 && raw(1) ~= 79 && ..."
            "        (raw(2) == 111 || raw(2) == 79) && ..."
            "        (raw(3) == 107 || raw(3) == 75) && raw(4) == 44"
            "    start = 2;"
            "end"
            "if n - start + 1 >= 4"
            "    is_ok = (raw(start) == 111 || raw(start) == 79) && ... % o/O"
            "            (raw(start + 1) == 107 || raw(start + 1) == 75) && ... % k/K"
            "            raw(start + 2) == 44;"
            "    if is_ok"
            "        index = 1;"
            "        value = 0.0;"
            "        sign_value = 1.0;"
            "        fraction = 0.1;"
            "        in_fraction = false;"
            "        have_digit = false;"
            "        for k = start + 3:n"
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
