display('Starting Basic Xbox Movement');


controllerLibrary = NET.addAssembly([pwd '.\SharpDX.XInput-StandAlone.dll']);
myController = SharpDX.XInput.Controller(SharpDX.XInput.UserIndex.One);

while (1)
    pause(.1)
    State = myController.GetState();
        
    right = double(State.Gamepad.LeftThumbY) / 327;
    left = double(State.Gamepad.RightThumbY) / 327;

    right / 2;
    left / 2;

    disp(left);
    disp(right);

    brick.MoveMotor('A', left);
    brick.MoveMotor('B', right);
end