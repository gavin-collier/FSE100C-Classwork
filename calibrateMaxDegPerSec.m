% Simple calibration: run both motors at 100% for a short time and measure steady-state deg/s.
% WARNING: This will spin motors at full power briefly. Use only when safe.
    display('Calibration will run motors at 100% briefly. Ensure robot is free to spin.');
    pause(1);
    % clear encoder counts first
    try
        brickObj.motorClrCount('A');
        brickObj.motorClrCount('B');
    catch
        % ignore if unsupported
    end
    % start motors
    brick.MoveMotor('A', 100);
    brick.MoveMotor('B', 100);
    pause(1.0); % let motors spin up
    % read speed samples for 0.5s
    samples = [];
    ts = tic;
    while toc(ts) < 0.5
        a1 = brick.GetMotorAngle('A');
        b1 = brick.GetMotorAngle('B');
        pause(0.05);
        a2 = brick.GetMotorAngle('A');
        b2 = brick.GetMotorAngle('B');
        dt = 0.05;
        sA = (a2 - a1) / dt;
        sB = (b2 - b1) / dt;
        samples(end+1, :) = [sA, sB]; %#ok<AGROW>
    end
    % stop motors
    brick.MoveMotor('A', 0);
    brick.MoveMotor('B', 0);
    % take median of samples for robustness
    medA = median(samples(:,1));
    medB = median(samples(:,2));
    maxDeg = mean([abs(medA), abs(medB)]);
    display(['Calibration raw: A=' num2str(medA) ', B=' num2str(medB) ', final MAX=' num2str(maxDeg)]);