%% --- 1. Caricamento Dati ---

addpath("SensorFusion");

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


% --- 3.3  Dati Encoders (Già Allineati e Puliti) ---
Left_ENC = encoder_left_V;
Right_ENC = encoder_right_V;


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
T_synced.encoder_left = Left_ENC;
T_synced.encoder_right = Right_ENC;


% Rinominiamo le colonne in modo leggibile e specifico
T_synced.Properties.VariableNames = {'Tempo [s]', ...
    'X_ARUCO [m]', 'Y_ARUCO [m]', 'Z_ARUCO [m]', ...
    'DegX_ARUCO [deg]', 'DegY_ARUCO [deg]', 'DegZ_ARUCO [deg]', ...
    'OmegaX_IMU [deg/s]', 'OmegaY_IMU [deg/s]', 'OmegaZ_IMU [deg/s]', ...
    'AccX_IMU [g]', 'AccY_IMU [g]', 'AccZ_IMU [g]', ...
    'Left_ENC [tic]', 'Right_ENC [tic]'};

%% --- Matrice di Rototraslazione con Formalismo 4x4 (Skew + Translation) ---
% Questo script costruisce le matrici W e H in formato 4x4 (se(3)) per ogni
% istante temporale e applica la trasformazione di similarità T * M * inv(T).

if ~exist('T_synced', 'var')
    error('Errore: La tabella T_synced non è disponibile.');
end

%% 1. Costruzione della Matrice di Trasformazione T (Costante)

% Rotazioni
alpha = - pi/2; % Z (Yaw)
beta = pi;    % X (Roll)

Rz = [cos(alpha), -sin(alpha), 0; sin(alpha),  cos(alpha), 0; 0, 0, 1];  
Rx = [1,  0, 0; 0,  cos(beta), -sin(beta); 0, sin(beta),  cos(beta)];
R = Rx * Rz; 

% Traslazione (cm -> m)
Tx_m = 70 / 100; 
Ty_m = 20 / 100; 
Tz_m = 0;
p = [Tx_m; Ty_m; Tz_m];

% Matrice T (4x4)
T = [R, p; 0, 0, 0, 1];

% Calcolo inversa di T 
T_inv = inv(T);


%% 2. Loop di Trasformazione sui Dati IMU

% Estrai i dati grezzi
Om_x = table2array(T_synced(:, "OmegaX_IMU [deg/s]"));
Om_y = table2array(T_synced(:, "OmegaY_IMU [deg/s]"));
Om_z = table2array(T_synced(:, "OmegaZ_IMU [deg/s]"));

Acc_x = table2array(T_synced(:, "AccX_IMU [g]"));
Acc_y = table2array(T_synced(:, "AccY_IMU [g]"));
Acc_z = table2array(T_synced(:, "AccZ_IMU [g]"));

N = length(Om_x);

% Preallocazione risultati
Om_Fixed = zeros(N, 3);
Acc_Fixed = zeros(N, 3);


for k = 1:N
    % --- 1. Costruzione Matrice W (Velocità Angolare) ---
    % W contiene la Skew Matrix di Omega nella parte rotazionale (3x3)
    % e zeri nella parte traslazionale 
    
    w_vec = [Om_x(k); Om_y(k); Om_z(k)];
    
    % Matrice Skew-Symmetric di omega
    Omega_Skew = [0,       -w_vec(3),  w_vec(2);
                  w_vec(3), 0,        -w_vec(1);
                 -w_vec(2), w_vec(1),  0       ];
             
    % Assemblaggio W (4x4)
    W_imu = [Omega_Skew, [0;0;0]; 
             0, 0, 0,     0     ];
         
    % --- 2. Costruzione Matrice H (Accelerazione Lineare) ---
    % H contiene l'accelerazione nella parte Traslazionale (colonna 4)
    % e zeri nella parte rotazionale 
    
    a_vec = [Acc_x(k); Acc_y(k); Acc_z(k)];
    
    % Assemblaggio H (4x4)
    H_imu = [zeros(3,3), a_vec;
             0, 0, 0,    0    ];
         
    % --- 3. Applicazione Change of RF (Similarity Transform) ---
    % Formula: M_fixed = T * M_imu * inv(T)
    
    W_fixed_mat = T * W_imu * T_inv;
    H_fixed_mat = T * H_imu * T_inv;
    
    % --- 4. Estrazione dei Vettori Trasformati ---
    
    % Estraz Omega dal skew block di W_fixed_mat
    % W_fixed_mat(3,2) è omega_x, W_fixed_mat(1,3) è omega_y, etc.
    omega_new = [W_fixed_mat(3,2); W_fixed_mat(1,3); W_fixed_mat(2,1)];
    
    % Estraz Accel dalla parte traslazionale di H_fixed_mat (prime 3 righe, col 4)
    acc_new = H_fixed_mat(1:3, 4);
    
    % Salvo nei vettori risultato
    Om_Fixed(k, :) = omega_new';
    Acc_Fixed(k, :) = acc_new';
end

%% 3. Aggiornamento Tabella

% Rimuovo vecchie colonne
T_synced(:, {'OmegaX_IMU [deg/s]', 'OmegaY_IMU [deg/s]', 'OmegaZ_IMU [deg/s]', ...
            'AccX_IMU [g]', 'AccY_IMU [g]', 'AccZ_IMU [g]'}) = [];

% Inserisco nuove colonne
T_synced.OmegaX_Fixed = Om_Fixed(:, 1);
T_synced.OmegaY_Fixed = Om_Fixed(:, 2);
T_synced.OmegaZ_Fixed = Om_Fixed(:, 3);
T_synced.AccX_Fixed = Acc_Fixed(:, 1);
T_synced.AccY_Fixed = Acc_Fixed(:, 2);
T_synced.AccZ_Fixed = Acc_Fixed(:, 3);

% Rinomino
T_synced.Properties.VariableNames{'OmegaX_Fixed'} = 'OmegaX_IMU_Fixed [deg/s]';
T_synced.Properties.VariableNames{'OmegaY_Fixed'} = 'OmegaY_IMU_Fixed [deg/s]';
T_synced.Properties.VariableNames{'OmegaZ_Fixed'} = 'OmegaZ_IMU_Fixed [deg/s]';
T_synced.Properties.VariableNames{'AccX_Fixed'} = 'AccX_IMU_Fixed [g]';
T_synced.Properties.VariableNames{'AccY_Fixed'} = 'AccY_IMU_Fixed [g]';
T_synced.Properties.VariableNames{'AccZ_Fixed'} = 'AccZ_IMU_Fixed [g]';

T_synced

