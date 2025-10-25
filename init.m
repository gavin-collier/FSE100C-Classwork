% Make sure you have RoboSim already running in the 'game mode aka seeing a robot in a maze' first

EV3_folder_path = 'C:\Users\gavin\Documents\FSE100\EV3'; 
% Change the above value of EV3_folder_path to your own EV3 toolbox folder in MATLAB -> HOME -> 'Set Path'

javaclasspath(EV3_folder_path);

brick = Brick('ioType','wifi','wfAddr','127.0.0.1','wfPort',5555,'wfSN','0016533dbaf5')

% You will see a success message in the Command Window of MATLAB if you have set up everything properly
