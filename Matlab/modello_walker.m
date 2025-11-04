%% Path Following for a Unicycle Robot
% This example demonstrates the disturbing factor effect using a Robot Simulator. 

% Copyright 2014-2016 The MathWorks, Inc. + prof Mariolino De Cecco

% BUG: The matlab "drive" function works only for x AND y > 0 (i.e. in the first quadrant)

% STEP 1 - Define Parameters
clear all;
close all;
clc;

addpath("data");
addpath("funzioni");

% Baseline
b = 0.5;
% Wheel radii
R = [0.1 0.1];
% Kinematic Parameters
K_param = [b R];
DK_param = 0*[0.02 0.01 0]; % deviation from ideal parameters [r, R_r R_L] credo


% Robot heading
theta0 = 0;
% wheel encoder resolution [tic/rev]
Enc_res = 4096;

% Robot heading error
theta0 = theta0; %+ 5*pi/180;


% Set the current location and the goal location of the robot as defined by the path
robotCurrentLocation = [0 0];

% Assume an initial robot orientation [rad]
initialOrientation = theta0;

% Define the current pose for the robot [x y theta]
robotInitialPose = [robotCurrentLocation initialOrientation];

% Initialize/Construct the Robot Simulator
% The inputs are: linear and angular velocities like a unicycle
robot = DiffDriveSimulator(K_param, DK_param, Enc_res, [0 0 0]);
robot.setRobotPose(robotInitialPose);

Pose0 = robot.GrabPose; % initial pose

%% STEP 2a - Simulate the robot behaviour with Encoder Tics/cycle

% Define the total simulation time [s]
Tt = 10;

% Define the While Loop update time [s]:
Tc = 0.05;
% Define the current time [s]:
T = 0;


%Straigth Line
NTic = 100 * ones(1,200);
% NTic = [NTic -NTic];


% --- Arco avanti + ritorno indietro ---
N_half = 100;   % 100 step = 5 s, totale 10 s

% fase avanti: arco di diametro 1 m
NTic_r_fwd = 154 * ones(1, N_half);   % ruota destra (più veloce)
NTic_l_fwd =  51 * ones(1, N_half);   % ruota sinistra (più lenta)

% fase indietro: stesso arco ma in retro
NTic_r_bwd = -154 * ones(1, N_half);
NTic_l_bwd = -51  * ones(1, N_half);

% unisci le due fasi
NTic_r_all = [NTic_r_fwd NTic_r_bwd];
NTic_l_all = [NTic_l_fwd NTic_l_bwd];


% Define the desired/ideal path
Vr = K_param(2) * 2*pi * NTic(2) / (Enc_res * Tc); %Vr = (distanza percorsa in Tc) / Tc
                                                   %dist in Tc = R*2pi *(Ntic/enc_res)
path = [0.00    0.00;
    Vr * Tt/2    0];

Nplot = 10; % plot number
Xlim = [-1 3];
Ylim = [-2 2];

% Simulates the robot trajectory:
for i=1:length(NTic)
     
    % MEASUREMENT: acquire the controller outputs, i.e., the inputs to the robot
    
    % Tic of the encoder wheels linea dritta
    % NTic_r = NTic(i);
    % NTic_l = NTic(i);
    
    %Cyrcle
    NTic_r = NTic_r_all(i); % ruota destra
    NTic_l = NTic_l_all(i); % ruota sinistra


    % SIMULATION: Simulate the robot using the controller outputs.
    robot.SimulateEnc(Tc, NTic_r, NTic_l);
    robot.Show(path, Nplot, Xlim, Ylim);
    
%     % PERCEPTION: Extract current location information ([X,Y]) from the 
%     % current pose of the robot and ADD NOISE simulating the sensors inaccuracy
%     robotCurrentPose = robot.robotCurrentPose + ...
%         0 * [normrnd(0, 0.01, 1, 2) normrnd(0, 2)*pi/180];
    
    % Re-compute the current time
    T = T + Tc;
        
    waitfor(Tc);
 
end

Pose = robot.GrabPose


% Close simulation.
delete(robot)





