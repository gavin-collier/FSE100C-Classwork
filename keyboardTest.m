NET.addAssembly('PresentationCore')
NET.addAssembly('PresentationFramework')
import System.Windows.Input.*

pause(2);

display('Starting Basic Keyboard Movement');

while (1)
    pause(.1)
    akey = System.Windows.Input.Key.A;
    switch key 
        case 'w'
        case 'a'
        case 's'
        case 'd'
    end
    display(akey);
end
CloseKeyboard();