display('Starting Xbox Movement!');


% -- Static Vars -- %

gears = [.33, .66, 1];
controllerLibrary = NET.addAssembly([pwd '.\SharpDX.XInput-StandAlone.dll']);
MAX_DEG_PER_SEC = 7.2;

% Initialize the controller and check its connection status
myController = SharpDX.XInput.Controller(SharpDX.XInput.UserIndex.One);

if myController.IsConnected
    display('Controller connected successfully.');
else
    display('Controller not connected.');
end

% Initialize other vars
leftCmd = 0;
rightCmd = 0;
currentGear = 3;

% -- Master Loop -- %
while (1) 
    pause(.1);
    State = myController.GetState();

    % get controler input and match max to 100
    yInput = double(State.Gamepad.LeftThumbY) / 327;
    xInput = double(State.Gamepad.RightThumbX) / 327 ;

    % convert the input to the proper drive
    drive = diamondDrive(yInput, xInput);

    % get the current motor speeds
    newLeftAngle = brick.GetMotorAngle('A');
    newRightAngle = brick.GetMotorAngle('B');
    realSpeeds = realMotorSpeed(newLeftAngle, newRightAngle);

    % map deg/s -> percent-of-max (-100..100)
    measLeftPct  = (realSpeeds(1) / MAX_DEG_PER_SEC);
    measRightPct = (realSpeeds(2) / MAX_DEG_PER_SEC);
    measLeftPct  = max(-100, min(100, measLeftPct));
    measRightPct = max(-100, min(100, measRightPct));

    % PID controllers (motorID 1 = left, 2 = right). Each call returns absolute power [-100..100].
    leftCmd  = leftCmd + pidLooper(measLeftPct,  drive(1), 1);
    rightCmd = leftCmd + pidLooper(measRightPct, drive(2), 2);
    

    disp("leftCmd = " + leftCmd);
    disp("rightCmd = " + rightCmd);

    leftMotor = round(leftCmd);
    rightMotor = round(rightCmd);
    
    %checkTouchSesnors();

    brick.MoveMotor('A', leftMotor);
    brick.MoveMotor('B', rightMotor);
end

% -- Functions -- %

function output = pidLooper(currentPwr, desiredPwr, motorID)
    persistent I lastError initialized lastTime;
    Kp = 0.09;
    Ki = 0.002;
    Kd = 0.005;

    if isempty(initialized)
        I = zeros(1,2);
        lastError = zeros(1,2);   % two motors: indices 1 and 2
        initialized = true;
        lastTime = tic;
    end

    % compute dt
    dt = toc(lastTime);
    if dt <= 0
        dt = 0.01;
    end
    lastTime = tic;

    err = desiredPwr - currentPwr;
    D = (err - lastError(motorID)) * dt;
    I(motorID) = I(motorID) + err * dt;


    % Gains are from persistent Kp/Ki/Kd
    u = err * Kp + Ki * I(motorID);

    lastError(motorID) = err;
    output = max(-100, min(100, u));
end


function output = diamondDrive(y, x)
    r = sqrt((x * x) + (y * y));
    t = atan2(x, y);

    t = t + (pi / 4);

    left = r * cos(t);
    right = r * sin(t);

    left = left * sqrt(2);
    right = right * sqrt(2);

    left = max(-100, min(left, 100));
    right = max(-100, min(right, 100));

    output = [left, right];
end

function output = realMotorSpeed(newLeftAngle, newRightAngle)
    persistent lastLeftAngle lastRightAngle lastTime

    if isempty(lastLeftAngle)
        lastLeftAngle = newLeftAngle;
        lastRightAngle = newRightAngle;
        lastTime = tic;
        dt = 0;
    end
   
    dt = toc(lastTime);
    
    realLeftSpeed = (newLeftAngle - lastLeftAngle) / dt;  % deg/s
    realRightSpeed = (newRightAngle - lastRightAngle) / dt;
    
    lastLeftAngle = newLeftAngle;
    lastRightAngle = newRightAngle;
    lastTime = tic;

    disp("realLeftSpeed = " + realLeftSpeed);
    disp("realRightSpeed = " + realRightSpeed);

    output = [realLeftSpeed, realRightSpeed];
end

function checkTouchSesnors()
    if brick.TouchPressed(1) || brick.TouchPressed(2)
        brick.playTone(100, 300, 500); 
    end
end
