display('Starting Xbox Movement!');


% -- Static Vars -- %

gears = [.33, .66, 1];
controllerLibrary = NET.addAssembly([pwd '.\SharpDX.XInput-StandAlone.dll']);
MAX_DEG_PER_SEC = 960;

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
beepCooldown = 0;
currentGear = 3;
gears = [.33, .66, 1];
% 

% -- Master Loop -- %
while (1) 
    pause(.1);
    State = myController.GetState();

    % get controler input and match max to 100
    yInput = double(State.Gamepad.LeftThumbY) / 327;
    xInput = double(State.Gamepad.RightThumbX) / 327 ;
    xInput = xInput * 2;

    %Reverse speed limit & Beep
    if yInput < -33
        yInput = -33;
    end

    if yInput < -1
        if beepCooldown == 0
            brick.playTone(500, 500, 500); 
            disp("BEEP!");
            beepCooldown = 10;
        end
        beepCooldown = beepCooldown - 1;
    end

    %disp("beepCooldown = " + beepCooldown);

    %Check for touch sensorers
    if brick.TouchPressed(1) || brick.TouchPressed(2)
        if beepCooldown == 0
            brick.playTone(500, 300, 500); 
            disp("BEEP 2!");
            beepCooldown = 10;
        end
        beepCooldown = beepCooldown - 1;
    end

    % gears
    % Update current gear based on controller input
    if State.Gamepad.Buttons == SharpDX.XInput.GamepadButtonFlags.Y
        currentGear = min(currentGear + 1, length(gears));
        disp("GEAR UP!");
    elseif State.Gamepad.Buttons == SharpDX.XInput.GamepadButtonFlags.X
        currentGear = max(currentGear - 1, 1);
        disp("GEAR DOWN!");
    end

    xInput = xInput * gears(currentGear);
    yInput = yInput * gears(currentGear);

    % convert the input to the proper drive
    currentSpeed = realMotorSpeed(brick.GetMotorAngle('A'), brick.GetMotorAngle('B'));
    drive = motionDrive(yInput, xInput);

    leftCmd = leftCmd + pidLooper(currentSpeed(1), drive(1));
    rightCmd = rightCmd + pidLooper(currentSpeed(2), drive(2));
    leftCmd = max(-100, min(leftCmd, 100));
    rightCmd = max(-100, min(rightCmd, 100));

    %disp("leftDesired: " + drive(1)); 
    %disp("leftCmd: " + leftCmd); 

    % prevent spinouts

    %disp("leftCmd = " + leftCmd);
    %disp("rightCmd = " + rightCmd);

    if (currentSpeed(1) > currentSpeed(2) + 50 && abs(yInput) > 10 && xInput < 80)
        leftCmd = 0;
        disp("Breaking LEFT!")
    end
    if (currentSpeed(1) + 50 < currentSpeed(2) && abs(yInput) > 10 && xInput > -80)
        rightCmd = 0;
        disp("Breaking RIGHT!")

    end

    % break Buttion 
     if (State.Gamepad.Buttons == SharpDX.XInput.GamepadButtonFlags.B)
        disp("B pressed")
        leftCmd = 0;
        rightCmd = 0;
    end

    if abs(leftCmd) < 10
        %leftCmd = 0;
    end
    if abs(rightCmd) < 10
        %rightCmd = 0;
    end

    brick.MoveMotor('A', leftCmd);
    brick.MoveMotor('B', rightCmd);
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

function output = motionDrive(V, w)
    leftOutput = 0;
    rightOutput = 0;

    leftOutput = (V - w / 3 / 2);
    rightOutput = (V + w / 3 / 2);
    
    output = [leftOutput, rightOutput];
end

function output = realMotorSpeed(newLeftAngle, newRightAngle)
    persistent lastLeftAngle lastRightAngle lastTime dt

    if isempty(lastLeftAngle)
        lastLeftAngle = 0;
        lastRightAngle = 0;
        lastTime = tic;
        dt = 0;
        output = 0;
    end
    
    dt = toc(lastTime);

    %disp("newLeftAngle = " + newLeftAngle);
    %disp("lastLeftAngle = " + lastLeftAngle);
    %disp("dt = " + dt);
    
    realLeftSpeed = (newLeftAngle - lastLeftAngle) / dt;  % deg/s
    realRightSpeed = (newRightAngle - lastRightAngle) / dt;

    %disp("realLeftSpeed = " + realLeftSpeed / 7.2);

    lastLeftAngle = newLeftAngle;
    lastRightAngle = newRightAngle;
    lastTime = tic;

    output = [realLeftSpeed / 7.2, realRightSpeed / 7.2];
end

function output = pidLooper(current, desired) 
    Kp =  .20;
    Ki = .0015;   
    Kd = .001;

    %disp("current: " + current);
    %disp("desired: " + desired);


    error = desired - current;
    tempOut = Kp * error; % Proportional term

    output = tempOut;
end