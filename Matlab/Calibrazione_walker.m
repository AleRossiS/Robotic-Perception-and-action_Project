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
b = 0.9; %m
% Wheel radii
R = [0.1 0.1]; %m
% Kinematic Parameters
K_param = [b R]; %m
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


%% Encoders Data extraction
MOTION_START = 125;

Ntic_R = table2array(T_synced(MOTION_START:end,"Right_ENC [tic]"));
Ntic_L = table2array(T_synced(MOTION_START:end,"Left_ENC [tic]"));

%Tic incremento at each step steps
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

x_HTC = table2array(T_synced(MOTION_START:end-1, "X_HTC [m]"));
y_HTC = table2array(T_synced(MOTION_START:end-1, "Y_HTC [m]"));
theta_HTC = table2array(T_synced(MOTION_START:end-1, "DegZ_HTC [deg]")); %deg around Z
theta_HTC = unwrap(theta_HTC); %to solve phase problems
theta_HTC = deg2rad(theta_HTC); %rad

figure(2); clf; hold on; grid on;
plot(x_HTC, y_HTC, 'LineWidth', 1.5, "Color", "b");
xlabel('x [m]');
ylabel('y [m]');
title('HTC Trajectory');
legend('HTC');
axis("equal");

%% Encoders data from simulation

%Extract them from the simulation 
x_ENC_sim = robot.actualPath(:,1);
y_ENC_sim = robot.actualPath(:,2);

figure(3); clf; hold on; grid on;
plot(x_ENC_sim, y_ENC_sim, 'LineWidth', 1.5, "Color", "r");
xlabel('x [m]');
ylabel('y [m]');
title('Encoders Trajectory Sim');
legend('Encoders');
axis("equal");


%% Encoders data from formulas

% pose_Enc_formulas = [x(t) y(t) theta(t)]
pose_Enc_formulas = EncodersMotion(K_param, Ntic_L, Ntic_R, Enc_res);

% remove offset between HTC and encoders data
x_ENC_form = pose_Enc_formulas(1,:) - x_HTC(1);
y_ENC_form = pose_Enc_formulas(2,:) - y_HTC(1);
theta_ENC_form = pose_Enc_formulas(3,:) - theta_HTC(1);

figure(4); clf; hold on; grid on;
plot(x_ENC_form, y_ENC_form, 'LineWidth', 1.5, "Color", "g");
plot(x_ENC_sim, y_ENC_sim, 'LineWidth', 1.5, "Color", "r");
xlabel('x [m]');
ylabel('y [m]');
title('Encoders Trajectory Formuls vs Sim');
legend('Formulas', 'Simulation');
axis("equal");

%% Plot the data of encoders and HTC together
figure(5); clf; hold on; grid on;
plot(x_ENC_form, y_ENC_form, 'LineWidth', 1.8, 'Color', 'r');
plot(x_HTC, y_HTC, 'LineWidth', 1.8, 'Color', 'b');
xlabel('x [m]', 'FontSize', 12);
ylabel('y [m]', 'FontSize', 12);
title('Encoders vs HTC', 'FontSize', 14, 'FontWeight', 'bold');
legend({'Encoders', 'HTC'}, 'Location', 'best');
axis("equal");

%% Theta enc vs HTC

figure(6);clf; hold on; grid on;
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), theta_ENC_form, 'LineWidth', 1.8, 'Color', 'r');
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), theta_HTC, 'LineWidth', 1.8, 'Color', 'b');
xlabel('t [s]', 'FontSize', 12);
ylabel('theta [rad]', 'FontSize', 12);
title('Theta Encoders vs HTC', 'FontSize', 14, 'FontWeight', 'bold');
legend({'Encoders', 'HTC'}, 'Location', 'best');
axis("equal");

%% Calibration

% Calibration function call
K_calib =  Calibration(K_param, Ntic_L, Ntic_R, Enc_res, x_HTC, y_HTC, theta_HTC)

% pose_Enc = [x(t) y(t) theta(t)]
pose_Enc_calib = EncodersMotion(K_calib, Ntic_L, Ntic_R, Enc_res);

% Data extraction
x_ENC_calib = pose_Enc_calib(1,:);
y_ENC_calib = pose_Enc_calib(2,:);
theta_ENC_calib = pose_Enc_calib(3,:);

figure(7); clf; hold on; grid on;
plot(x_ENC_calib, y_ENC_calib, 'LineWidth', 1.5, "Color", "r");
plot(x_HTC, y_HTC, 'LineWidth', 1.5, "Color", "b");
xlabel('x [m]');
ylabel('y [m]');
title('Calibrated Data');
legend('Calib', 'HTC');
axis("equal");

figure(8); clf; hold on; grid on;
plot(x_ENC_calib, y_ENC_calib, 'LineWidth', 1.5, "Color", "r");
plot(x_ENC_form, y_ENC_form, 'LineWidth', 1.5, "Color", "g", "LineStyle","--");
plot(x_HTC, y_HTC, 'LineWidth', 1.5, "Color", "b");
xlabel('x [m]');
ylabel('y [m]');
title('All data Comparison');
legend('Calib', 'Formula', 'HTC');
axis("equal");

%% Pose Enc vs HTC after calibration by single values

figure(9);clf; hold on; grid on;
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), theta_ENC_calib, 'LineWidth', 1.8, 'Color', 'r');
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), theta_HTC, 'LineWidth', 1.8, 'Color', 'b');
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), theta_ENC_form, 'LineWidth', 1.5, "Color", "g", "LineStyle","--");
xlabel('t [s]', 'FontSize', 12);
ylabel('theta [rad]', 'FontSize', 12);
title('Theta Encoders vs HTC after Calibration', 'FontSize', 14, 'FontWeight', 'bold');
legend({'Encoders Calib', 'HTC', 'Encoders NO Calib'}, 'Location', 'best');


figure(10);clf; hold on; grid on;
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), x_ENC_calib, 'LineWidth', 1.8, 'Color', 'r');
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), x_HTC, 'LineWidth', 1.8, 'Color', 'b');
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), x_ENC_form, 'LineWidth', 1.5, "Color", "g", "LineStyle","--");
xlabel('t [s]', 'FontSize', 12);
ylabel('x [m]', 'FontSize', 12);
title('X Encoders vs HTC after Calibration', 'FontSize', 14, 'FontWeight', 'bold');
legend({'Encoders Calib', 'HTC', 'Encoders NO Calib'}, 'Location', 'best');


figure(11);clf; hold on; grid on;
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), y_ENC_calib, 'LineWidth', 1.8, 'Color', 'r');
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), y_HTC, 'LineWidth', 1.8, 'Color', 'b');
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), y_ENC_form, 'LineWidth', 1.5, "Color", "g", "LineStyle","--");
xlabel('t [s]', 'FontSize', 12);
ylabel('y [m]', 'FontSize', 12);
title('Y vs HTC after Calibration', 'FontSize', 14, 'FontWeight', 'bold');
legend({'Encoders Calib', 'HTC', 'Encoders NO Calib'}, 'Location', 'best');
