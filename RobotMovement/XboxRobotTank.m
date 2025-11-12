display('Starting Basic Xbox Movement');


controllerLibrary = NET.addAssembly([pwd '.\SharpDX.XInput-StandAlone.dll']);
myController = SharpDX.XInput.Controller(SharpDX.XInput.UserIndex.One);
MAX_DEG_PER_SEC = 960.0;

while (1)
    pause(.1)
    State = myController.GetState();
        
    right = double(State.Gamepad.LeftThumbY) / 327;
    left = double(State.Gamepad.RightThumbY) / 327;

    brick.MoveMotor('A', left);
    brick.MoveMotor('B', right);

    newLeftAngle = brick.GetMotorAngle('A');
    newRightAngle = brick.GetMotorAngle('B');
    realSpeeds = realMotorSpeed(newLeftAngle, newRightAngle);

    %disp("realLeftSpeed = " + realSpeeds(1) / MAX_DEG_PER_SEC);
    %disp("realRightSpeed = " + realSpeeds(1) / MAX_DEG_PER_SEC);
end

function output = realMotorSpeed(newLeftAngle, newRightAngle)
    persistent lastLeftAngle lastRightAngle lastTime dt

    if isempty(lastLeftAngle)
        lastLeftAngle = 0;
        lastRightAngle = 0;
        lastTime = tic;
        dt = 0;
    end
    
    dt = toc(lastTime);

    disp("newLeftAngle = " + newLeftAngle);
    disp("lastLeftAngle = " + lastLeftAngle);
    disp("dt = " + dt);
    
    realLeftSpeed = (newLeftAngle - lastLeftAngle) / dt;  % deg/s
    realRightSpeed = (newRightAngle - lastRightAngle) / dt;

    disp("realLeftSpeed = " + realLeftSpeed / 7.2);

    lastLeftAngle = newLeftAngle;
    lastRightAngle = newRightAngle;
    lastTime = tic;

    output = [realLeftSpeed, realRightSpeed];
end