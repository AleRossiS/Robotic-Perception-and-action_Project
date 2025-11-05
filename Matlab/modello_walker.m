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
run("estraz_dati.m");

% Baseline
b = 0.9;
% Wheel radii
R = [0.1 0.1];
% Kinematic Parameters
K_param = [b R];
DK_param = [0 0 0];

% Robot heading
theta0 = 0;
% wheel encoder resolution [tic/rev]
Enc_res = 4096;

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


%% ENCODERS DATA EXTRACTION
MOTION_START = 125;

Ntic_R = table2array(T_synced(MOTION_START:end,"Right_ENC [tic]"));
Ntic_L = table2array(T_synced(MOTION_START:end,"Left_ENC [tic]"));

%Tic steps
Ntic_R = diff(Ntic_R);
Ntic_L = diff(Ntic_L);

%% Simulate the robot behaviour with Encoder Tics/cycle

% Define the total simulation time [s]
Tt = table2array(T_synced(end, "Tempo [s]"));

% Define the While Loop update time [s]:
Tc = table2array(T_synced(end, "Tempo [s]")) - table2array(T_synced(end-1, "Tempo [s]"));
% Define the current time [s]:
T = 0;

%I don't need a path
path = [0.00  0.00; 
        0       0];

Nplot = 1; % plot number
Xlim = [-2 4];
Ylim = [-3 3];

% Simulates the robot trajectory:
for i=1:length(Ntic_R)
     
    % MEASUREMENT: acquire the controller outputs, i.e., the inputs to the robot
 
    %Cyrcle
    NTic_r = Ntic_R(i); % ruota destra
    NTic_l = Ntic_L(i); % ruota sinistra


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

Pose = robot.GrabPose;


% Close simulation.
% delete(robot)


%% HTC Data

x_HTC = table2array(T_synced(MOTION_START:end-1, "X_HTC [mm]"));
y_HTC = table2array(T_synced(MOTION_START:end-1, "Y_HTC [mm]"));
theta_HTC = table2array(T_synced(MOTION_START:end-1, "DegZ_HTC [deg]")); %deg
theta_HTC = unwrap(theta_HTC); %to solve phase problems
theta_HTC = deg2rad(theta_HTC); %rad

figure(2); hold on; grid on;
plot(x_HTC, y_HTC, 'LineWidth', 1.5, "Color", "b");
xlabel('x [mm]');
ylabel('y [mm]');
title('Traiettoria HTC');

legend('HTC');
axis("equal");

%% Encoders data from simulation

%Extract them from the simulation 

x_ENC_sim = robot.actualPath(:,1);
y_ENC_sim = robot.actualPath(:,2);

figure(3); hold on; grid on;
plot(x_ENC_sim, y_ENC_sim, 'LineWidth', 1.5, "Color", "r");
xlabel('x [mm]');
ylabel('y [mm]');
title('Traiettoria Encoders');
legend('Encoders');
axis("equal");


%% Encoders data from formulas

% pose_Enc_formulas = [x(t) y(t) theta(t)]
pose_Enc_formulas = EncodersMotion(K_param, Ntic_L, Ntic_R, Enc_res);

x_ENC_form = pose_Enc_formulas(1,:);
y_ENC_form = pose_Enc_formulas(2,:);
theta_ENC_form = pose_Enc_formulas(3,:);

figure(4); hold on; grid on;
plot(x_ENC_form, y_ENC_form, 'LineWidth', 1.5, "Color", "g");
plot(x_ENC_sim, y_ENC_sim, 'LineWidth', 1.5, "Color", "r");
xlabel('x [mm]');
ylabel('y [mm]');
title('Traiettoria Encoders');
legend('Formulas', 'Simulation');
axis("equal");

%% Plot the data of encoders and HTC together
figure(5); hold on; grid on;
plot(x_ENC_form, y_ENC_form, 'LineWidth', 1.8, 'Color', 'r');
plot(x_HTC, y_HTC, 'LineWidth', 1.8, 'Color', 'b');
xlabel('x [mm]', 'FontSize', 12);
ylabel('y [mm]', 'FontSize', 12);
title('Encoders vs HTC', 'FontSize', 14, 'FontWeight', 'bold');
legend({'Encoders', 'HTC'}, 'Location', 'best');
axis("equal");

%% Theta enc vs HTC

figure(6);clf; hold on; grid on;
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), theta_ENC_form, 'LineWidth', 1.8, 'Color', 'r');
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), theta_HTC, 'LineWidth', 1.8, 'Color', 'b');


%% Calibration

K_calib =  Calibration(K_param, Ntic_L, Ntic_R, Enc_res, x_HTC, y_HTC, theta_HTC)

% pose_Enc = [x(t) y(t) theta(t)]
pose_Enc_calib = EncodersMotion(K_calib, Ntic_L, Ntic_R, Enc_res);

x_ENC_calib = pose_Enc_calib(1,:);
y_ENC_calib = pose_Enc_calib(2,:);
theta_ENC_calib = pose_Enc_calib(3,:);

figure(7); hold on; grid on;
plot(x_ENC_calib, y_ENC_calib, 'LineWidth', 1.5, "Color", "r");
plot(x_HTC, y_HTC, 'LineWidth', 1.5, "Color", "b");
xlabel('x [mm]');
ylabel('y [mm]');
title('Traiettoria Encoders');
legend('Calib', 'HTC');
axis("equal");

figure(8); clf; hold on; grid on;
plot(x_ENC_calib, y_ENC_calib, 'LineWidth', 1.5, "Color", "r");
plot(x_ENC_form, y_ENC_form, 'LineWidth', 1.5, "Color", "b");
plot(x_HTC, y_HTC, 'LineWidth', 1.5, "Color", "g");
xlabel('x [mm]');
ylabel('y [mm]');
title('Traiettoria Encoders');
legend('Calib', 'Formula', 'HTC');
axis("equal");

