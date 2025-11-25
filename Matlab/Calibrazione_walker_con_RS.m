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

% %% Simulate the robot behaviour with Encoder Tics/cycle
% 
% % Define the total simulation time [s]
% Tt = table2array(T_synced(end, "Tempo [s]"));
% 
% % Define the While Loop update time [s]:
% Tc = table2array(T_synced(end, "Tempo [s]")) - table2array(T_synced(end-1, "Tempo [s]"));
% % Define the current time [s]:
% T = 0;
% 
% %I don't need a path
% path = [0.00  0.00; 
%         0       0];
% 
% Nplot = 1; % plot number
% Xlim = [-2 4];
% Ylim = [-3 3];
% 
% % Simulates the robot trajectory:
% for i=1:length(Ntic_R)
% 
%     % MEASUREMENT: acquire the controller outputs, i.e., the inputs to the robot
% 
%     %Cyrcle
%     NTic_r = Ntic_R(i); % ruota destra
%     NTic_l = Ntic_L(i); % ruota sinistra
% 
% 
%     % SIMULATION: Simulate the robot using the controller outputs.
%     robot.SimulateEnc(Tc, NTic_r, NTic_l);
%     robot.Show(path, Nplot, Xlim, Ylim);
% 
%     % PERCEPTION: Extract current location information ([X,Y]) from the 
%     % current pose of the robot and ADD NOISE simulating the sensors inaccuracy
%     robotCurrentPose = robot.robotCurrentPose + ...
%         0 * [normrnd(0, 0.01, 1, 2) normrnd(0, 2)*pi/180];
% 
%     % Re-compute the current time
%     T = T + Tc;
% 
%     waitfor(Tc);
% 
% end
% 
% Pose = robot.GrabPose;
% 
% 
% % Close simulation.
% % delete(robot)


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

%% Aruco Data Extraction and Cleaning
% --- Da implementare in base alle colonne reali ---
x_ARUCO = table2array(T_synced(MOTION_START:end-1, "X_RS [m]"));
y_ARUCO = table2array(T_synced(MOTION_START:end-1, "Y_RS [m]"));
theta_ARUCO = table2array(T_synced(MOTION_START:end-1, "DegZ_RS [deg]"));
theta_ARUCO = unwrap(theta_ARUCO); 
theta_ARUCO = deg2rad(theta_ARUCO);

% Gestione degli Zeri/NaNs di Aruco:
% Assumiamo che i valori zero o molto vicini allo zero in Aruco indichino
% una lettura non riuscita o non valida.
% Sostituiamo questi valori non validi con i dati HTC, in modo che la fusione 
% non sia influenzata negativamente quando Aruco fallisce.
% La soglia 1e-4 è arbitraria, adattala se necessario.
idx_invalid_aruco = (abs(x_ARUCO) < 1e0 & abs(y_ARUCO) < 1e0);
x_ARUCO(idx_invalid_aruco) = x_HTC(idx_invalid_aruco);
y_ARUCO(idx_invalid_aruco) = y_HTC(idx_invalid_aruco);
theta_ARUCO(idx_invalid_aruco) = theta_HTC(idx_invalid_aruco);

 %% NUOVO RIFERIMENTO FUSO (HTC+RS):

% solo HTC
x_REF2 = x_HTC;
y_REF2 = y_HTC;
theta_REF2 = theta_HTC;

% HTC and Aruco Data Fusion (Dynamic Weighting)
% 1. Definizione dei pesi base e della soglia di affidabilità
w_HTC_base = 0.85; % Peso base di HTC (molto alto)
w_ARUCO_max = 0.45; % Peso massimo di Aruco
threshold_dist = 0.1; % [m] - Soglia: se Aruco dista più di 10 cm da HTC, è un outlier. (ADATTARE!)

% 2. Inizializzazione del riferimento fuso
x_REF1 = x_HTC;
y_REF1 = y_HTC;
theta_REF1 = theta_HTC;

% 3. Loop per la fusione dinamica punto per punto
for i = 1:length(x_HTC)
    
    % Distanza tra la misura Aruco e la misura HTC
    dist_aruco_htc = sqrt((x_ARUCO(i) - x_HTC(i))^2 + (y_ARUCO(i) - y_HTC(i))^2);
    
    if dist_aruco_htc < threshold_dist
        % Se Aruco è vicino a HTC, lo usiamo (peso dinamico)
        % Normalizziamo il peso in base alla vicinanza (più vicino = più peso)
        % Un peso semplice, ma dinamico:
        w_aruco_current = w_ARUCO_max * (1 - dist_aruco_htc / threshold_dist);
        w_htc_current = 1 - w_aruco_current;
        
        % Fusione
        x_REF1(i) = w_htc_current * x_HTC(i) + w_aruco_current * x_ARUCO(i);
        y_REF1(i) = w_htc_current * y_HTC(i) + w_aruco_current * y_ARUCO(i);
        theta_REF1(i) = w_htc_current * theta_HTC(i) + w_aruco_current * theta_ARUCO(i);
        
    else
        % Se Aruco è troppo lontano (outlier), ignoriamo la sua lettura.
        % Usiamo solo HTC. (x_REF, y_REF, theta_REF mantengono i valori HTC iniziali)
        
        % Se Aruco è stato sostituito da HTC nella pulizia iniziale (punto A della risposta precedente),
        % questo blocco non è strettamente necessario, ma è utile per una logica pulita.
    end
end

%% Plot the data of encoders and REF together
figure(5); clf; hold on; grid on;
plot(x_ENC_form, y_ENC_form, 'LineWidth', 1.8, 'Color', 'r');
plot(x_REF1, y_REF1, 'LineWidth', 1.8, 'Color', 'b');
plot(x_REF2, y_REF2, 'LineWidth', 1.8, 'Color', 'g', "LineStyle",":");
xlabel('x [m]', 'FontSize', 12);
ylabel('y [m]', 'FontSize', 12);
title('Encoders vs REF', 'FontSize', 14, 'FontWeight', 'bold');
legend({'Encoders', 'HTC+RS', 'HTC'}, 'Location', 'best');
axis("equal");

%% Theta enc vs REF

figure(6);clf; hold on; grid on;
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), theta_ENC_form, 'LineWidth', 1.8, 'Color', 'r');
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), theta_REF1, 'LineWidth', 1.8, 'Color', 'b');
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), theta_REF2, 'LineWidth', 1.8, 'Color', 'g', "LineStyle",":");
xlabel('t [s]', 'FontSize', 12);
ylabel('theta [rad]', 'FontSize', 12);
title('Theta Encoders vs REF', 'FontSize', 14, 'FontWeight', 'bold');
legend({'Encoders', 'HTC+RS', 'HTC'}, 'Location', 'best');
axis("equal");

%% Calibration

% Calibration function call
K_calib1 =  Calibration(K_param, Ntic_L, Ntic_R, Enc_res, x_REF1, y_REF1, theta_REF1);
K_calib2 =  Calibration(K_param, Ntic_L, Ntic_R, Enc_res, x_REF2, y_REF2, theta_REF2);

% pose_Enc = [x(t) y(t) theta(t)]
pose_Enc_calib1 = EncodersMotion(K_calib1, Ntic_L, Ntic_R, Enc_res);
pose_Enc_calib2 = EncodersMotion(K_calib2, Ntic_L, Ntic_R, Enc_res);


% Data extraction
x_ENC_calib1 = pose_Enc_calib1(1,:);
y_ENC_calib1 = pose_Enc_calib1(2,:);
theta_ENC_calib1 = pose_Enc_calib1(3,:);

x_ENC_calib2 = pose_Enc_calib2(1,:);
y_ENC_calib2 = pose_Enc_calib2(2,:);
theta_ENC_calib2 = pose_Enc_calib2(3,:);


figure(7); clf; hold on; grid on;
plot(x_ENC_calib1, y_ENC_calib1, 'LineWidth', 1.5, "Color", "r");
plot(x_ENC_calib2, y_ENC_calib2, 'LineWidth', 1.5, "Color", "w", "LineStyle",":");
plot(x_REF1, y_REF1, 'LineWidth', 1.5, "Color", "b");
plot(x_REF2, y_REF2, 'LineWidth', 1.5, "Color", "g", "LineStyle",":");
xlabel('x [m]');
ylabel('y [m]');
title('Calibrated Data');
legend('Calib1','Calib2', 'HTC+RS', 'HTC');
axis("equal");

figure(8); clf; hold on; grid on;
plot(x_ENC_calib1, y_ENC_calib1, 'LineWidth', 1.5, "Color", "r");
plot(x_ENC_calib2, y_ENC_calib2, 'LineWidth', 1.5, "Color", "w", "LineStyle",":");
plot(x_ENC_form, y_ENC_form, 'LineWidth', 1.5, "Color", "g", "LineStyle","--");
plot(x_REF1, y_REF1, 'LineWidth', 1.5, "Color", "b");
plot(x_REF2, y_REF2, 'LineWidth', 1.5, "Color", "y", "LineStyle",":");
xlabel('x [m]');
ylabel('y [m]');
title('All data Comparison');
legend('Calib1', 'Calib2','Formula','HTC+RS', 'HTC');
axis("equal");

%% Pose Enc vs REF after calibration by single values

figure(9);clf; hold on; grid on;
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), theta_ENC_calib1, 'LineWidth', 1.8, 'Color', 'r');
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), theta_ENC_calib2, 'LineWidth', 1.8, 'Color', 'w', "LineStyle",":");
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), theta_REF1, 'LineWidth', 1.8, 'Color', 'b');
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), theta_ENC_form, 'LineWidth', 1.5, "Color", "g", "LineStyle","--");
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), theta_REF2, 'LineWidth', 1.8, 'Color', 'y', "LineStyle",":");
xlabel('t [s]', 'FontSize', 12);
ylabel('theta [rad]', 'FontSize', 12);
title('Theta Encoders vs REF after Calibration', 'FontSize', 14, 'FontWeight', 'bold');
legend({'Encoders Calib1','Encoders Calib2', 'HTC+RS', 'Encoders NO Calib','HTC'}, 'Location', 'best');


figure(10);clf; hold on; grid on;
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), x_ENC_calib1, 'LineWidth', 1.8, 'Color', 'r');
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), x_ENC_calib2, 'LineWidth', 1.8, 'Color', 'w', "LineStyle",":");
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), x_REF1, 'LineWidth', 1.8, 'Color', 'b');
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), x_ENC_form, 'LineWidth', 1.5, "Color", "g", "LineStyle","--");
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), x_REF2, 'LineWidth', 1.8, 'Color', 'y', "LineStyle",":");
xlabel('t [s]', 'FontSize', 12);
ylabel('x [m]', 'FontSize', 12);
title('X Encoders vs HTC after Calibration', 'FontSize', 14, 'FontWeight', 'bold');
legend({'Encoders Calib1','Encoders Calib2', 'HTC+RS', 'Encoders NO Calib','HTC'}, 'Location', 'best');


figure(11);clf; hold on; grid on;
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), y_ENC_calib1, 'LineWidth', 1.8, 'Color', 'r');
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), y_ENC_calib2, 'LineWidth', 1.8, 'Color', 'w', "LineStyle",":");
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), y_REF1, 'LineWidth', 1.8, 'Color', 'b');
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), y_ENC_form, 'LineWidth', 1.5, "Color", "g", "LineStyle","--");
plot(table2array(T_synced(MOTION_START:end-1, "Tempo [s]")), y_REF2, 'LineWidth', 1.8, 'Color', 'y', "LineStyle",":");
xlabel('t [s]', 'FontSize', 12);
ylabel('y [m]', 'FontSize', 12);
title('Y vs REF after Calibration', 'FontSize', 14, 'FontWeight', 'bold');
legend({'Encoders Calib1','Encoders Calib2', 'HTC+RS', 'Encoders NO Calib','HTC'}, 'Location', 'best');

% Error
function [max_error_pos, rms_error_pos] = CalculateError(x_calib, y_calib, theta_calib, x_ref, y_ref, theta_ref)
% Calcola l'errore di posizione e orientamento tra due traiettorie.
% Assumiamo che le traiettorie siano sincronizzate e abbiano la stessa lunghezza.

for i=1:length(x_calib)
    % Errore di Posizione (Euclideo)
    error_x(i) = x_calib(i) - x_ref(i);
    error_y(i) = y_calib(i) - y_ref(i);
    error_pos(i) = sqrt(error_x(i).^2 + error_y(i).^2);

    % Errore di Orientamento (Angolare)
    % Usiamo atan2 per gestire la periodicità di theta e calcoliamo la differenza angolare minima
    error_theta(i) = atan2(sin(theta_calib(i) - theta_ref(i)), cos(theta_calib(i) - theta_ref(i)));

    % Metriche di Errore di Posizione
    max_error_pos = max(error_pos);
    rms_error_pos = sqrt(mean(error_pos.^2)); % Errore Quadratico Medio (Root Mean Square)
end
%     
% 
    % Plot dell'Errore nel tempo
    % figure; hold on; grid on;
    % t = 1:length(error_pos);
    % subplot(2,1,1);
    % plot(t, error_pos, 'LineWidth', 1.5, 'Color', 'k');
    % title('Errore di Posizione nel Tempo (Euclideo)');
    % xlabel('Passo di Tempo');
    % ylabel('Errore di Posizione [m]');
    % 
    % subplot(2,1,2);
    % plot(t, rad2deg(error_theta), 'LineWidth', 1.5, 'Color', 'r');
    % title('Errore di Orientamento nel Tempo');
    % xlabel('Passo di Tempo');
    % ylabel('Errore di Orientamento [deg]');
end

% Calcolo e Visualizzazione dell'Errore
% Usa la traiettoria calibrata rispetto al riferimento REF
[max_err_pos_calib1, rms_err_pos_calib1] = CalculateError(x_ENC_calib1, y_ENC_calib1, theta_ENC_calib1, x_REF1, y_REF1, theta_REF1);
[max_err_pos_calib2, rms_err_pos_calib2] = CalculateError(x_ENC_calib2, y_ENC_calib2, theta_ENC_calib2, x_REF2, y_REF2, theta_REF2);

% Stampa a terminale
    fprintf('\n--- Risultati Errore (Calib vs Reference) ---\n');
    fprintf('Errore di Posizione Massimo (Max Error): HTC+RS %.4f [m]\n, HTC %.4f [m]\n', max_err_pos_calib1, max_err_pos_calib2);
    fprintf('Errore di Posizione RMS (RMS Error): HTC+RS %.4f [m]\n, HTC %.4f [m]\n', rms_err_pos_calib1, rms_err_pos_calib2);

% Puoi anche calcolare l'errore prima della calibrazione per confronto:
%fprintf('\n*** ERRORE PRE-CALIBRAZIONE ***\n');
%[~, ~] = CalculateError(x_ENC_form, y_ENC_form, theta_ENC_form, x_HTC, y_HTC, theta_HTC);
%fprintf('\n*** ERRORE POST-CALIBRAZIONE ***\n');