display('Starting Basic Xbox Movement');


controllerLibrary = NET.addAssembly([pwd '.\SharpDX.XInput-StandAlone.dll']);
myController = SharpDX.XInput.Controller(SharpDX.XInput.UserIndex.One);

leftMotor = 0;
rightMotor = 0;

gears = [33, 66, 100];
currentGear = 3;

lastLeftAngle = brick.GetMotorAngle('A');
lastRightAngle = brick.GetMotorAngle('B');
lastTime = tic;

while (1)
    pause(.1)
    State = myController.GetState();
        
    linearInput = double(State.Gamepad.LeftThumbY) / 327;
    turnInput = double(State.Gamepad.RightThumbX) / 327;

    % turn tuning
   % if turnInput > 0
    %    linearInput = linearInput * .90;
    %    turnInput = turnInput * .1;
    %end

    %Reverse speed limit & Beep
    if linearInput < -33
        linearInput = -33;
    end

    if linearInput < -1
        brick.beep();
    end

    %Check for touch sensorers
    if brick.TouchPressed(1) || brick.TouchPressed(2)
        brick.playTone(100, 300, 500); 
    end

    %set gear
    linearInput = max(min(linearInput, gears(currentGear)), -gears(currentGear));

    if (State.Gamepad.LeftTrigger > 10)
        currentGear = max(currentGear - 1, 0);
    end
    if (State.Gamepad.RightTrigger > 10)
        currentGear = min(currentGear + 1, length(gears)); 
    end

    newLeftAngle = brick.GetMotorAngle('A');
    newRightAngle = brick.GetMotorAngle('B');
    dt = toc(lastTime);
    
    realLeftSpeed = (newLeftAngle - lastLeftAngle) / dt;  % deg/s
    realRightSpeed = (newRightAngle - lastRightAngle) / dt;
    
    lastLeftAngle = newLeftAngle;
    lastRightAngle = newRightAngle;
    lastTime = tic;

    disp("RealleftMotor = " + realLeftSpeed);
    disp("RealrightMotor = " + realRightSpeed);

    %disp("turnInput = " + turnInput);
    %disp("linearInput = " + linearInput);

    driveSet = diamondDrive(linearInput, turnInput);

    leftMotor = leftMotor + pidLooper(realLeftSpeed, driveSet(1));
    rightMotor = rightMotor + pidLooper(realRightSpeed, driveSet(2));

    disp("leftMotor = " + leftMotor);
    disp("rightMotor = " + rightMotor);

    % Hard Stop
    if (State.Gamepad.Buttons == SharpDX.XInput.GamepadButtonFlags.B)
        disp("B pressed")
        leftMotor = 0;
        rightMotor = 0;
    end

    brick.MoveMotor('A', leftMotor);
    brick.MoveMotor('B', rightMotor);
end

function output = pidLooper(current, desired) 
    Kp =  .025;
    Ki = .0015;   
    Kd = .001;

    %disp("current: " + current);
    %disp("desired: " + desired);


    error = desired - current;
    tempOut = Kp * error; % Proportional term
    %tempOut = tempOut + Ki * error; % Integral term
    %tempOut = tempOut + Kd * (error - current); % Derivative term
    output = tempOut;
end

function output = simpleDrive(x, y)
    left = y - x;
    right = y + x;
    
    output = [left, right];
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