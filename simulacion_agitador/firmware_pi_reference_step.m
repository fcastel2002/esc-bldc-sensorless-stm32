function [nextState, pwm] = firmware_pi_reference_step(state, speedRpm, setpointRpm, config)
%FIRMWARE_PI_REFERENCE_STEP Execute one firmware-equivalent speed PI update.
%   [NEXTSTATE, PWM] = FIRMWARE_PI_REFERENCE_STEP(STATE, SPEEDRPM,
%   SETPOINTRPM, CONFIG) mirrors the CLOSEDLOOP calculations in
%   firmware/Core/Src/motor_control.c. It is a pure function: STATE holds
%   integral, previousError, and diffSpeed, and NEXTSTATE holds their update.
%   Missing CONFIG fields use the firmware defaults for the STM32F103 target.

arguments
    state (1, 1) struct
    speedRpm (1, 1) {mustBeNumeric}
    setpointRpm (1, 1) {mustBeNumeric}
    config (1, 1) struct
end

config = localResolveConfig(config);
integralQ16 = int64(localStateField(state, "integralQ16", int32(0)));
previousError = int32(localStateField(state, "previousError", int32(0)));
speedRpm = int32(min(65535, max(0, fix(speedRpm))));
setpointRpm = int32(min(65535, max(0, fix(setpointRpm))));
errorRpm = setpointRpm - speedRpm;

kpQ16 = int64(fix(config.kp * 65536 + 0.5));
kiDtHalfQ30 = int64(fix(config.ki * config.dt * 0.5 * 1073741824 + 0.5));
proportionalQ16 = kpQ16 * int64(errorRpm);
integralDeltaQ16 = bitshift(kiDtHalfQ30 * int64(errorRpm + previousError), -14);
minimumQ16 = int64(100 * 65536);
maximumQ16 = int64(2000 * 65536);
integralQ16 = min(maximumQ16 - proportionalQ16, ...
    max(minimumQ16 - proportionalQ16, integralQ16 + integralDeltaQ16));
integralQ16 = min(int64(intmax("int32")), max(int64(intmin("int32")), integralQ16));
outputQ16 = min(maximumQ16, max(minimumQ16, proportionalQ16 + integralQ16));
canonicalPwm = bitshift(outputQ16, -16);
pwm = uint16(idivide(canonicalPwm * int64(config.pwmArr) + 1000, int64(2000), "floor"));

nextState = state;
nextState.integralQ16 = int32(integralQ16);
nextState.previousError = int32(errorRpm);
end

function config = localResolveConfig(config)
config.kp = single(localRequiredConfigField(config, "kp"));
config.ki = single(localRequiredConfigField(config, "ki"));
config.kd = single(localRequiredConfigField(config, "kd"));
config.pwmArr = localPositiveInteger(localRequiredConfigField(config, "pwmArr"), "pwmArr", 65535);
config.polePairs = localPositiveInteger(localRequiredConfigField(config, "polePairs"), "polePairs", 255);
config.timerHz = localPositiveInteger(localRequiredConfigField(config, "timerHz"), "timerHz", double(intmax("uint32")));
config.speedMinPeriod = localPositiveInteger(localRequiredConfigField(config, "speedMinPeriod"), "speedMinPeriod", 65535);
config.speedMaxPeriod = localPositiveInteger(localRequiredConfigField(config, "speedMaxPeriod"), "speedMaxPeriod", 65535);
config.minimumPwm = localPositiveInteger(localRequiredConfigField(config, "minimumPwm"), "minimumPwm", config.pwmArr);
config.dt = single(localRequiredConfigField(config, "dt"));
config.algorithmVersion = localRequiredConfigField(config, "algorithmVersion");

if config.algorithmVersion ~= 2
    error("firmware_pi_reference_step:UnsupportedAlgorithm", ...
        "Only RPM PI algorithm version 2 can generate new reference samples.");
end
if config.kd ~= 0
    error("firmware_pi_reference_step:KdNotImplemented", ...
        "KD must be zero for RPM PI algorithm version 2.");
end

if config.speedMaxPeriod > config.speedMinPeriod
    error("firmware_pi_reference_step:InvalidSpeedRange", ...
        "speedMaxPeriod must not exceed speedMinPeriod.");
end
if ~isfinite(config.dt) || config.dt <= 0
    error("firmware_pi_reference_step:InvalidDt", "dt must be finite and positive.");
end
end

function value = localRequiredConfigField(config, name)
if ~isfield(config, name) || isempty(config.(name))
    error("firmware_pi_reference_step:MissingConfig", "Config field %s is required.", name);
end
value = config.(name);
if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value)
    error("firmware_pi_reference_step:InvalidConfig", ...
        "Config field %s must be a finite numeric scalar.", name);
end
end

function value = localPositiveInteger(value, name, maximum)
if value < 1 || value > maximum || value ~= fix(value)
    error("firmware_pi_reference_step:InvalidConfig", ...
        "Config field %s must be an integer in [1, %.0f].", name, maximum);
end
value = double(value);
end

function value = localStateField(state, name, defaultValue)
if isfield(state, name) && ~isempty(state.(name))
    value = state.(name);
else
    value = defaultValue;
end
if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value)
    error("firmware_pi_reference_step:InvalidState", ...
        "State field %s must be a finite numeric scalar.", name);
end
end
