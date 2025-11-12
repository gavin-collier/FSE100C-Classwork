global key
InitKeyboard();

pause(2);

display('Starting Basic Keyboard Movement');

while (1)
    pause(.1)
    switch key 
        case 'w'
            brick.MoveMotor('AB', 100);
            display('Forwards W Input');
        case 'a'
            brick.MoveMotor('A', -100);
            brick.MoveMotor('B', 100);
        case 's'
            brick.MoveMotor('AB', -100);
        case 'd'
            brick.MoveMotor('A', 100);
            brick.MoveMotor('B', -100);
        case 0
            brick.MoveMotor('AB', 0);
            display('No Input');
        case 'q'
            break;
    end
end
CloseKeyboard();