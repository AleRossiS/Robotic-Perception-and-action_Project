%% --- 1. Caricamento Dati ---

% Specifica il nome dei tuoi file
fileNameARUCO = 'SensorFusion.H_initial_walker_aruco.csv';
fileNameENC   = 'SensorFusion.encoders.csv';
fileNameIMU   = 'SensorFusion.imu.csv';

% Imposta le opzioni di importazione (DataLines = 2 è già impostato)
optsARUCO = detectImportOptions(fileNameARUCO);
optsENC   = detectImportOptions(fileNameENC);
optsIMU   = detectImportOptions(fileNameIMU);
optsARUCO.DataLines = 2; optsENC.DataLines = 2; optsIMU.DataLines = 2;

% Leggi i dati in una tabella
try
    dataTableARUCO = readtable(fileNameARUCO, optsARUCO);
    dataTableENC   = readtable(fileNameENC, optsENC);
    dataTableIMU   = readtable(fileNameIMU, optsIMU);
catch e
    disp('Errore durante il caricamento del file:');
    disp(e.message);
    return;
end

%% --- 2. Preparazione e Pulizia Dati (Eliminazione Timestamps Duplicati) ---

% Funzione utility per aggregare i dati (calcola la media per i duplicati)
% groupsummary è l'opzione più semplice, ma usiamo la logica manuale vista prima per massima compatibilità.
aggregate_by_time = @(T_FULL, DataCols) deal(T_FULL, DataCols); % Dummy declaration

% --- Pulizia ENCODERS (Definizione del Master Time Base) ---
T_ENC_FULL = dataTableENC.message_timecode - dataTableENC.message_timecode(1);
DataCols_ENC = [dataTableENC.message_encoders_left, dataTableENC.message_encoders_right];

[T_ENC_UNIQUE, ~, idx_group_ENC] = unique(T_ENC_FULL, 'stable');
numUnique_ENC = length(T_ENC_UNIQUE);
AggregatedData_ENC = zeros(numUnique_ENC, 2);

for i = 1:numUnique_ENC
    current_indices = find(idx_group_ENC == i);
    AggregatedData_ENC(i, :) = mean(DataCols_ENC(current_indices, :), 1);
end

% Setta il Tempo Master e i Dati Encoder Puliti
Tempo_Synched = T_ENC_UNIQUE; 
encoder_left_V = AggregatedData_ENC(:, 1);
encoder_right_V = AggregatedData_ENC(:, 2);

disp('Sincronizzazione basata sul tempo degli Encoders (dati puliti).');


% --- Pulizia ARUCO ---
T_ARUCO_FULL = dataTableARUCO.message_timecode - dataTableARUCO.message_timecode(1);
DataCols_ARUCO = [
    dataTableARUCO.message_pose_position_0__0_, dataTableARUCO.message_pose_position_0__1_, ...
    dataTableARUCO.message_pose_position_0__2_, dataTableARUCO.message_pose_attitude_deg_0__0_, ...
    dataTableARUCO.message_pose_attitude_deg_0__1_, dataTableARUCO.message_pose_attitude_deg_0__2_
];

[T_ARUCO_UNIQUE, ~, idx_group_ARUCO] = unique(T_ARUCO_FULL, 'stable');
numUnique_ARUCO = length(T_ARUCO_UNIQUE);
AggregatedData_ARUCO = zeros(numUnique_ARUCO, 6);

for i = 1:numUnique_ARUCO
    current_indices = find(idx_group_ARUCO == i);
    AggregatedData_ARUCO(i, :) = mean(DataCols_ARUCO(current_indices, :), 1); 
end

% Dati Aruco Puliti per l'interpolazione
T_ARUCO_V    = T_ARUCO_UNIQUE;
X_ARUCO_V    = AggregatedData_ARUCO(:, 1);
Y_ARUCO_V    = AggregatedData_ARUCO(:, 2);
Z_ARUCO_V    = AggregatedData_ARUCO(:, 3);
DegX_ARUCO_V = AggregatedData_ARUCO(:, 4);
DegY_ARUCO_V = AggregatedData_ARUCO(:, 5);
DegZ_ARUCO_V = AggregatedData_ARUCO(:, 6);


% --- Pulizia IMU ---
T_IMU_FULL = dataTableIMU.message_timecode - dataTableIMU.message_timecode(1);
DataCols_IMU = [
    dataTableIMU.message_gyro_0_, dataTableIMU.message_gyro_1_, dataTableIMU.message_gyro_2_, ...
    dataTableIMU.message_accel_0_, dataTableIMU.message_accel_1_, dataTableIMU.message_accel_2_
];

[T_IMU_UNIQUE, ~, idx_group_IMU] = unique(T_IMU_FULL, 'stable');
numUnique_IMU = length(T_IMU_UNIQUE);
AggregatedData_IMU = zeros(numUnique_IMU, 6);

for i = 1:numUnique_IMU
    current_indices = find(idx_group_IMU == i);
    AggregatedData_IMU(i, :) = mean(DataCols_IMU(current_indices, :), 1);
end

% Dati IMU Puliti per l'interpolazione
T_IMU_V = T_IMU_UNIQUE;
OmegaX_IMU_V = AggregatedData_IMU(:, 1);
OmegaY_IMU_V = AggregatedData_IMU(:, 2);
OmegaZ_IMU_V = AggregatedData_IMU(:, 3);
AccX_IMU_V = AggregatedData_IMU(:, 4);
AccY_IMU_V = AggregatedData_IMU(:, 5);
AccZ_IMU_V = AggregatedData_IMU(:, 6);


%% --- 3. Interpolazione e Sincronizzazione sul Tempo Master (Tempo_Synched) ---

% Costante di conversione Rad/s a Deg/s
RadToDeg = 180 / pi;

% --- 3.1. Interpolazione Dati ARUCO ---

X_ARUCO = interp1(T_ARUCO_V, X_ARUCO_V, Tempo_Synched, 'pchip');
Y_ARUCO = interp1(T_ARUCO_V, Y_ARUCO_V, Tempo_Synched, 'pchip');
Z_ARUCO = interp1(T_ARUCO_V, Z_ARUCO_V, Tempo_Synched, 'pchip');

DegX_ARUCO = interp1(T_ARUCO_V, DegX_ARUCO_V, Tempo_Synched, 'pchip');
DegY_ARUCO = interp1(T_ARUCO_V, DegY_ARUCO_V, Tempo_Synched, 'pchip');
DegZ_ARUCO = interp1(T_ARUCO_V, DegZ_ARUCO_V, Tempo_Synched, 'pchip');


% --- 3.2. Interpolazione Dati IMU ---

OmegaX_IMU = interp1(T_IMU_V, OmegaX_IMU_V, Tempo_Synched, 'pchip');
OmegaY_IMU = interp1(T_IMU_V, OmegaY_IMU_V, Tempo_Synched, 'pchip');
OmegaZ_IMU = interp1(T_IMU_V, OmegaZ_IMU_V, Tempo_Synched, 'pchip');

AccX_IMU = interp1(T_IMU_V, AccX_IMU_V, Tempo_Synched, 'pchip');
AccY_IMU = interp1(T_IMU_V, AccY_IMU_V, Tempo_Synched, 'pchip');
AccZ_IMU = interp1(T_IMU_V, AccZ_IMU_V, Tempo_Synched, 'pchip');


% --- 3.3. Dati Encoders (Già Allineati e Puliti) ---
encoder_left = encoder_left_V;
encoder_right = encoder_right_V;


%% --- 4. Creazione Tabella Sincronizzata (T_synced) ---

T_synced = table();
T_synced.Tempo = Tempo_Synched;

% Dati ARUCO 
T_synced.X_ARUCO = X_ARUCO;
T_synced.Y_ARUCO = Y_ARUCO;
T_synced.Z_ARUCO = Z_ARUCO;
T_synced.DegX_ARUCO = DegX_ARUCO;
T_synced.DegY_ARUCO = DegY_ARUCO;
T_synced.DegZ_ARUCO = DegZ_ARUCO;

% Dati IMU - Velocità Angolari (Omega - Giroscopi)
T_synced.OmegaX_IMU = OmegaX_IMU;
T_synced.OmegaY_IMU = OmegaY_IMU;
T_synced.OmegaZ_IMU = OmegaZ_IMU;

% Dati IMU - Accelerazioni (Acc)
T_synced.AccX_IMU = AccX_IMU;
T_synced.AccY_IMU = AccY_IMU;
T_synced.AccZ_IMU = AccZ_IMU;

% Encoders
T_synced.encoder_left = encoder_left;
T_synced.encoder_right = encoder_right;


% Rinominiamo le colonne in modo leggibile e specifico
T_synced.Properties.VariableNames = {'Tempo [s]', ...
    'X_ARUCO [m]', 'Y_ARUCO [m]', 'Z_ARUCO [m]', ...
    'DegX_ARUCO [deg]', 'DegY_ARUCO [deg]', 'DegZ_ARUCO [deg]', ...
    'OmegaX_IMU [rad/s]', 'OmegaY_IMU [rad/s]', 'OmegaZ_IMU [rad/s]', ...
    'AccX_IMU [g]', 'AccY_IMU [g]', 'AccZ_IMU [g]', ...
    'Left_ENC [tic]', 'Right_ENC [tic]'};

T_synced
