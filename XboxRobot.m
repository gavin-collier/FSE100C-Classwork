display('Starting Basic Xbox Movement');


controllerLibrary = NET.addAssembly([pwd './SharpDX.XInput.dll']);
myController = SharpDX.XInput.Controller(SharpDX.XInput.UserIndex.One);

Left = zeros(100,1);
Right = zeros(100,1);

while (1)
    pause(.1)
    State = myController.GetState();
    
    Left(i) = double(State.Gamepad.LeftTrigger);
    Right(i) = double(State.Gamepad.RightTrigger);
    
    disp(Left);
end