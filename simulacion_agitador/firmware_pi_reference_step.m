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
integral = single(localStateField(state, "integral", single(0)));
previousError = int32(localStateField(state, "previousError", int32(0)));
diffSpeed = int32(localStateField(state, "diffSpeed", int32(0)));

speedPeriod = localRpmToPeriod(speedRpm, config);
targetPeriod = localRpmToPeriod(setpointRpm, config);
speedMeasure = localPeriodToPwm(speedPeriod, config);
targetPwm = localPeriodToPwm(targetPeriod, config);
speedError = int32(double(targetPwm) - double(speedMeasure));

if diffSpeed == 0
    diffSpeed = int32(speedMeasure);
end

speedProportional = config.kp * single(speedError);
speedProportional = speedProportional - ...
    config.kd * single(int32(speedMeasure) - diffSpeed) / config.dt;
diffSpeed = int32(speedMeasure);
integral = integral + config.ki * single(double(speedError) + double(previousError)) * config.dt;
previousError = speedError;

maxPwm = single(config.pwmArr);
minPwm = single(uint16(fix(double(maxPwm * single(0.05)))));
if maxPwm > speedProportional
    maxIntegral = maxPwm - speedProportional;
else
    maxIntegral = single(0);
end
if minPwm < speedProportional
    minIntegral = minPwm - speedProportional;
else
    minIntegral = single(0);
end
integral = min(maxIntegral, max(minIntegral, integral));

% speed_output is int32_t in firmware, so its assignment truncates before
% the final PWM limit checks.
speedOutput = int32(fix(double(speedProportional + integral)));
speedOutput = min(int32(config.pwmArr), max(int32(uint16(minPwm)), speedOutput));

nextState = state;
nextState.integral = single(integral);
nextState.previousError = int32(previousError);
nextState.diffSpeed = int32(diffSpeed);
pwm = uint16(speedOutput);
end

function config = localResolveConfig(config)
config.kp = single(localConfigField(config, "kp", 0.75));
config.ki = single(localConfigField(config, "ki", 1.35));
config.kd = single(localConfigField(config, "kd", 0));
config.pwmArr = localPositiveInteger(localConfigField(config, "pwmArr", 2000), "pwmArr", 65535);
config.polePairs = localPositiveInteger(localConfigField(config, "polePairs", 2), "polePairs", 255);
config.timerHz = localPositiveInteger(localConfigField(config, "timerHz", 180000), "timerHz", double(intmax("uint32")));
config.speedMinPeriod = localPositiveInteger(localConfigField(config, "speedMinPeriod", 14000), "speedMinPeriod", 65535);
config.speedMaxPeriod = localPositiveInteger(localConfigField(config, "speedMaxPeriod", 200), "speedMaxPeriod", 65535);
config.dt = single(localConfigField(config, "dt", 0.002));

if config.speedMaxPeriod > config.speedMinPeriod
    error("firmware_pi_reference_step:InvalidSpeedRange", ...
        "speedMaxPeriod must not exceed speedMinPeriod.");
end
if ~isfinite(config.dt) || config.dt <= 0
    error("firmware_pi_reference_step:InvalidDt", "dt must be finite and positive.");
end
end

function value = localConfigField(config, name, defaultValue)
if isfield(config, name) && ~isempty(config.(name))
    value = config.(name);
else
    value = defaultValue;
end
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

function period = localRpmToPeriod(rpm, config)
if ~isfinite(rpm)
    error("firmware_pi_reference_step:InvalidRpm", "RPM inputs must be finite.");
end
rpm = min(65535, max(0, fix(rpm)));
if rpm == 0
    period = uint16(65535);
    return;
end

% This is the integer division and clamp order used by rpm_to_period().
ticks = floor((config.timerHz * 30) / (rpm * config.polePairs));
ticks = min(config.speedMinPeriod, max(config.speedMaxPeriod, ticks));
period = uint16(ticks);
end

function pwm = localPeriodToPwm(period, config)
rawSpeed = double(period);
if rawSpeed == 0
    rawSpeed = config.speedMinPeriod;
end
rawSpeed = min(config.speedMinPeriod, max(config.speedMaxPeriod, rawSpeed));

minPwm = uint16(fix(double(single(config.pwmArr) * single(0.05))));
rangeRelation = single(config.pwmArr - double(minPwm)) / ...
    single(config.speedMinPeriod - config.speedMaxPeriod);
mapped = single(config.speedMinPeriod - rawSpeed) * rangeRelation + single(minPwm);
pwm = uint16(fix(double(mapped)));
end
