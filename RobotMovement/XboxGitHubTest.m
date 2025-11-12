%% robotJoy.m - Refactored Xbox-driven EV3 velocity-control loop (patched)
%  Motor control uses encoder feedback and PID correction on top of feedforward command.
display('Starting Xbox Movement!');

% ------------------ TUNING PARAMETERS ------------------
% Gears (fractions of full power)
GEARS = [.33, .66, 1];

% If you know your motor deg/s at 100% power, set it here.
% Otherwise enable CALIBRATE_MAX_DEG_PER_SEC to run the small calibration routine.
MAX_DEG_PER_SEC = 960;   % deg/s at 100% motor power (estimate)
CALIBRATE_MAX_DEG_PER_SEC = false;  % set true to run calibration once at start

% Loop & safety
LOOP_DELAY = 0.10;       % seconds between main loop iterations
LOOP_MIN_DT = 0.01;      % fallback dt if encoder read is too fast

% PID gains (start here, then tune)
PID_Kp = 1.0;
PID_Ki = 0.02;
PID_Kd = 0.1;

% Integral clamp to avoid windup
INTEGRAL_CLAMP = 500;

% --- Anti-spin and input handling params ---
JOYSTICK_DEADZONE = 0.08;        % joystick deadzone ([-1..1])
TURN_DAMPING_GAIN = 0.35;        % damping on angular rate (0 = none, 1+ = strong)
BRAKE_ON_FLIP_MS    = 60;        % ms to apply short brake when direction flips
TURN_SCALE_WHEN_STOPPED = 0.6;   % scale turn when linear small (prevents runaway spin)
MAX_TURN_WHEN_LOW_LINEAR = 70;   % max turn pct if linear low
FLIP_THRESHOLD_PCT = 10;         % percent change threshold to consider a flip
% -------------------------------------------------------

% Load controller assembly (SharpDX)
controllerLibrary = NET.addAssembly([pwd '.\SharpDX.XInput-StandAlone.dll']);
myController = SharpDX.XInput.Controller(SharpDX.XInput.UserIndex.One);
if myController.IsConnected
    display('Controller connected successfully.');
else
    display('Controller not connected.');
end

% Create brick if not already in workspace (uncomment / adjust to your setup)
% brick = Brick('ioType','usb');

% Initialize runtime state
leftMotorCmd = 0;    % final integer power to motor A
rightMotorCmd = 0;   % final integer power to motor B
leftCmd = 0.0;       % floating command before rounding
rightCmd = 0.0;
currentGearIdx = length(GEARS); % start in top gear

% Initialize PID persistent state by calling once (this call is tolerant to argument order)
pidLooper('init', 0, 0, 0, PID_Kp, PID_Ki, PID_Kd);

% Prime realMotorSpeed persistent variables so first dt isn't zero
% (read current angles)
try
    lastLeftAngleInit  = brick.GetMotorAngle('A');
    lastRightAngleInit = brick.GetMotorAngle('B');
catch ME
    error('Failed to read brick motors. Ensure "brick" exists and is connected. %s', ME.message);
end
realMotorSpeed(lastLeftAngleInit, lastRightAngleInit); % initialize internals

% previous desired drive used for flip-detection
prevDesiredDrive = [0, 0];

% -------------- Main Loop --------------
display('Entering main loop. Use B button to hard stop.');
while true
    pause(LOOP_DELAY);
    State = myController.GetState();

    % --- Read controller inputs and scale to [-1..1] ---
    yInput_raw = double(State.Gamepad.LeftThumbY) / 327;   % forward/back
    xInput_raw = double(State.Gamepad.RightThumbX) / 327;  % turn

    % Apply deadzone to straps
    if abs(yInput_raw) < JOYSTICK_DEADZONE
        yInput_raw = 0;
    end
    if abs(xInput_raw) < JOYSTICK_DEADZONE
        xInput_raw = 0;
    end

    % Gear switching (left / right trigger)
    if (State.Gamepad.LeftTrigger > 10)
        currentGearIdx = max(currentGearIdx - 1, 1);
    end
    if (State.Gamepad.RightTrigger > 10)
        currentGearIdx = min(currentGearIdx + 1, length(GEARS));
    end

    % Convert joystick to percent [-100..100] using the selected gear
    linearPct = yInput_raw * GEARS(currentGearIdx) * 100;
    turnPct   = xInput_raw * GEARS(currentGearIdx) * 100;

    % Limit turn if linear is near zero (reduce tendency to spin in-place)
    if abs(linearPct) < 6
        turnPct = turnPct * TURN_SCALE_WHEN_STOPPED;
        turnPct = clamp(turnPct, -MAX_TURN_WHEN_LOW_LINEAR, MAX_TURN_WHEN_LOW_LINEAR);
    end

    % Compute desired left/right drive (feedforward) in [-100..100]
    desiredDrive = diamondDrive(linearPct, turnPct); % [leftDesiredPct, rightDesiredPct]

    % Read current encoder angles and compute real speed (deg/s) and dt
    newLeftAngle  = brick.GetMotorAngle('A');
    newRightAngle = brick.GetMotorAngle('B');
    realVals = realMotorSpeed(newLeftAngle, newRightAngle); % [leftDegPerSec,rightDegPerSec,dt]
    realLeftDegPerSec  = realVals(1);
    realRightDegPerSec = realVals(2);
    dt = realVals(3);
    if dt <= 0
        dt = LOOP_MIN_DT;
    end

    % Convert measured deg/s -> percent-of-max (-100..100)
    measLeftPct  = (realLeftDegPerSec  / MAX_DEG_PER_SEC) * 100;
    measRightPct = (realRightDegPerSec / MAX_DEG_PER_SEC) * 100;
    measLeftPct  = clamp(measLeftPct, -100, 100);
    measRightPct = clamp(measRightPct, -100, 100);

    % --- Rotational damping: fight angular motion (left-right wheel diff)
    angularRatePct = measLeftPct - measRightPct; % positive: left faster relative to right
    dampingCorrection = -TURN_DAMPING_GAIN * angularRatePct;
    dampingLeft  = 0.5 * dampingCorrection;
    dampingRight = -0.5 * dampingCorrection;

    % --- Brake on abrupt desired-direction flip (prevents coast spin)
    % For left wheel:
    if sign(desiredDrive(1)) ~= sign(prevDesiredDrive(1)) && abs(desiredDrive(1) - prevDesiredDrive(1)) > FLIP_THRESHOLD_PCT
        try
            brick.StopMotor('A', 'Brake');
            pause(BRAKE_ON_FLIP_MS / 1000);
        catch
            % ignore if StopMotor variant unavailable
        end
        pidLooper('resetMotor', 1); % reset only left integrator
    end
    % For right wheel:
    if sign(desiredDrive(2)) ~= sign(prevDesiredDrive(2)) && abs(desiredDrive(2) - prevDesiredDrive(2)) > FLIP_THRESHOLD_PCT
        try
            brick.StopMotor('B', 'Brake');
            pause(BRAKE_ON_FLIP_MS / 1000);
        catch
        end
        pidLooper('resetMotor', 2); % reset only right integrator
    end
    prevDesiredDrive = desiredDrive; % store for next iter

    % Compute PID corrections (signed values). PID expects (measured, desired, motorID, dt)
    leftCorrection  = pidLooper(measLeftPct,  desiredDrive(1), 1, dt);
    rightCorrection = pidLooper(measRightPct, desiredDrive(2), 2, dt);

    % Add damping corrections
    leftCorrection  = leftCorrection + dampingLeft;
    rightCorrection = rightCorrection + dampingRight;

    % Final motor command = feedforward desired + PID correction (then clamp)
    leftCmd  = clamp(desiredDrive(1) + leftCorrection,  -100, 100);
    rightCmd = clamp(desiredDrive(2) + rightCorrection, -100, 100);

    % Debug output (concise)
    disp("measLeft = " + round(measLeftPct));
    disp("targetLeft = "  + round(desiredDrive(1)));
    disp("leftCmd = "  + round(leftCmd));
    disp(" ");
    disp("measRight = " + round(measRightPct));
    disp("targetRight = "  + round(desiredDrive(2)));
    disp("rightCmd = "  + round(rightCmd));

    % Hard Stop: B button
    if (State.Gamepad.Buttons == SharpDX.XInput.GamepadButtonFlags.B)
        disp('B pressed - stopping motors and resetting PID integrators.');
        leftCmd = 0;
        rightCmd = 0;
        pidLooper('reset'); % clear integrators and last error for both motors
    end

    % Touch sensor beep
    checkTouchSensors();

    % Send commands to brick (round to integer)
    leftMotorCmd  = round(leftCmd);
    rightMotorCmd = round(rightCmd);
    brick.MoveMotor('A', leftMotorCmd);
    brick.MoveMotor('B', rightMotorCmd);
end

% ----------------- FUNCTIONS -----------------
function out = diamondDrive(y, x)
% diamondDrive expects y,x in [-100..100] (percent) and returns [left,right] percent
    r = sqrt((x .* x) + (y .* y));
    t = atan2(x, y);
    t = t + (pi / 4);
    left = r .* cos(t);
    right = r .* sin(t);
    left = left * sqrt(2);
    right = right * sqrt(2);
    left = clamp(left, -100, 100);
    right = clamp(right, -100, 100);
    out = [left, right];
end

function output = realMotorSpeed(newLeftAngle, newRightAngle)
% realMotorSpeed: returns [leftDegPerSec, rightDegPerSec, dt]
% uses persistent values to compute delta/time between calls.
    persistent lastLeftAngle lastRightAngle lastTime
    if isempty(lastLeftAngle)
        lastLeftAngle  = newLeftAngle;
        lastRightAngle = newRightAngle;
        lastTime = tic;
        pause(0.01); % allow minimal time so dt isn't extremely small
    end
    dt = toc(lastTime);
    if dt <= 0
        dt = 0.01;
    end
    realLeftSpeed  = (newLeftAngle  - lastLeftAngle)  / dt;
    realRightSpeed = (newRightAngle - lastRightAngle) / dt;
    lastLeftAngle  = newLeftAngle;
    lastRightAngle = newRightAngle;
    lastTime = tic;
    output = [realLeftSpeed, realRightSpeed, dt];
end

function corr = pidLooper(currentPwr, desiredPwr, motorID, dt, Kp_in, Ki_in, Kd_in)
% pidLooper: per-motor PID. Supports:
%   - pidLooper('init', ..., Kp, Ki, Kd)  to configure gains initially
%   - pidLooper('reset') to zero integrators/lastError
%   - pidLooper('resetMotor', motorID) to zero integrator/lastError for single motor
%   - pidLooper(measured, desired, motorID, dt) returns correction (signed)
    persistent I lastError Kp Ki Kd integralClamp initialized

    % special initializer / control commands
    if nargin >= 1 && ischar(currentPwr)
        cmd = currentPwr;
        switch cmd
            case 'init'
                % tolerant initializer: allow calling pidLooper('init',0,0,0,Kp,Ki,Kd)
                % typical call in this file: pidLooper('init', 0,0,0, PID_Kp, PID_Ki, PID_Kd);
                if nargin >= 6
                    Kp = Kp_in;
                    Ki = Ki_in;
                    Kd = Kd_in;
                else
                    Kp = 1.0; Ki = 0.02; Kd = 0.1;
                end
                integralClamp = 500;
                I = zeros(1,2);
                lastError = zeros(1,2);
                initialized = true;
                corr = [];
                return;
            case 'reset'
                if isempty(initialized)
                    corr = [];
                    return;
                end
                I = zeros(1,2);
                lastError = zeros(1,2);
                corr = [];
                return;
            case 'resetMotor'
                % desiredPwr holds motorID in this call
                if isempty(initialized)
                    corr = [];
                    return;
                end
                motorToReset = desiredPwr;
                if motorToReset >= 1 && motorToReset <= 2
                    I(motorToReset) = 0;
                    lastError(motorToReset) = 0;
                end
                corr = [];
                return;
            otherwise
                error('pidLooper: unknown command "%s"', cmd);
        end
    end

    % normal call: ensure initialized (if not, set default gains)
    if isempty(initialized)
        Kp = 1.0; Ki = 0.02; Kd = 0.1; integralClamp = 500;
        I = zeros(1,2);
        lastError = zeros(1,2);
        initialized = true;
    end

    % If optional gains provided in this call, override persisting ones
    if nargin >= 6 && ~isempty(Kp_in)
        Kp = Kp_in; Ki = Ki_in; Kd = Kd_in;
    end

    if nargin < 4 || isempty(dt) || dt <= 0
        dt = 0.01;
    end

    % compute error (desired - measured)
    err = desiredPwr - currentPwr;

    % integral with anti-windup
    I(motorID) = I(motorID) + err * dt;
    I(motorID) = clamp(I(motorID), -integralClamp, integralClamp);

    % derivative
    D = (err - lastError(motorID)) / dt;

    % correction output (signed)
    corr = Kp * err + Ki * I(motorID) + Kd * D;

    % clamp correction to avoid extreme jumps
    corr = clamp(corr, -100, 100);

    % store lastError
    lastError(motorID) = err;
end

function checkTouchSensors()
% beep if touch sensors pressed
    try
        if brick.TouchPressed(1) || brick.TouchPressed(2)
            brick.playTone(50, 400, 200);
        end
    catch
        % ignore if brick not available
    end
end

function val = clamp(x, lo, hi)
    val = min(max(x, lo), hi);
end
