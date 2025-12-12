%% --- 1. Caricamento Dati ---
addpath("ExtrinsicCalibration");

% Specifica il nome dei tuoi file
fileNameHTC = 'extrinsic_calibration.htc.csv';
fileNameENC = 'extrinsic_calibration.encoders.csv';

% Imposta le opzioni di importazione e DataLines = 2
optsHTC = detectImportOptions(fileNameHTC);
optsENC = detectImportOptions(fileNameENC);
optsHTC.DataLines = 2; 
optsENC.DataLines = 2;

% Leggo i dati in una tabella
try
    dataTableENC   = readtable(fileNameENC, optsENC);
    dataTableHTC   = readtable(fileNameHTC, optsHTC);
catch e
    disp('Errore durante il caricamento del file:');
    disp(e.message);
    return;
end

%% --- 2. Preparazione e Pulizia Dati (Eliminazione Timestamps Duplicati) ---
% --- Pulizia ENCODERS (Definizione del Master Time Base) ---
T_ENC_FULL = dataTableENC.message_timecode - dataTableENC.message_timecode(1);
DataCols_ENC = [dataTableENC.message_encoders_left, dataTableENC.message_encoders_right];

% Aggregazione: calcola la media per i dati duplicati (richiesto da interp1)
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

disp(['Sincronizzazione completata sul tempo degli Encoders. Righe finali: ', num2str(numUnique_ENC)]);


% --- Pulizia HTC ---
T_HTC_FULL = dataTableHTC.message_timecode - dataTableHTC.message_timecode(1);
DataCols_HTC = [
    dataTableHTC.message_pose_position_0_, dataTableHTC.message_pose_position_1_, ...
    dataTableHTC.message_pose_position_2_, dataTableHTC.message_pose_attitude_deg_0_, ...
    dataTableHTC.message_pose_attitude_deg_1_, dataTableHTC.message_pose_attitude_deg_2_
];

% Aggregazione: calcola la media per i dati duplicati (richiesto da interp1)
[T_HTC_UNIQUE, ~, idx_group_HTC] = unique(T_HTC_FULL, 'stable');
numUnique_HTC = length(T_HTC_UNIQUE);
AggregatedData_HTC = zeros(numUnique_HTC, 6);

for i = 1:numUnique_HTC
    current_indices = find(idx_group_HTC == i);
    AggregatedData_HTC(i, :) = mean(DataCols_HTC(current_indices, :), 1); 
end

% Dati HTC Puliti per l'interpolazione
T_HTC_V    = T_HTC_UNIQUE;
X_HTC_V    = AggregatedData_HTC(:, 1);
Y_HTC_V    = AggregatedData_HTC(:, 2);
Z_HTC_V    = AggregatedData_HTC(:, 3);
DegX_HTC_V = AggregatedData_HTC(:, 4);
DegY_HTC_V = AggregatedData_HTC(:, 5);
DegZ_HTC_V = AggregatedData_HTC(:, 6);


%% --- 3. Interpolazione e Sincronizzazione sul Tempo Master (Tempo_Synched) ---

% --- 3.1. Interpolazione Dati HTC ---

X_HTC = interp1(T_HTC_V, X_HTC_V, Tempo_Synched, 'pchip');
Y_HTC = interp1(T_HTC_V, Y_HTC_V, Tempo_Synched, 'pchip');
Z_HTC = interp1(T_HTC_V, Z_HTC_V, Tempo_Synched, 'pchip');

DegX_HTC = interp1(T_HTC_V, DegX_HTC_V, Tempo_Synched, 'pchip');
DegY_HTC = interp1(T_HTC_V, DegY_HTC_V, Tempo_Synched, 'pchip');
DegZ_HTC = interp1(T_HTC_V, DegZ_HTC_V, Tempo_Synched, 'pchip');


% --- 3.2. Dati Encoders (Già Allineati e Puliti) ---
Left_ENC = encoder_left_V;
Right_ENC = encoder_right_V;


%% --- 4. Creazione Tabella Sincronizzata (T_synced) ---

T_synced = table();
T_synced.Tempo = Tempo_Synched;

% Dati HTC (Pose)
T_synced.X_HTC = X_HTC;
T_synced.Y_HTC = Y_HTC;
T_synced.Z_HTC = Z_HTC;
T_synced.DegX_HTC = DegX_HTC;
T_synced.DegY_HTC = DegY_HTC;
T_synced.DegZ_HTC = DegZ_HTC;

% Encoders
T_synced.encoder_left = Left_ENC;
T_synced.encoder_right = Right_ENC;


% Rinominiamo le colonne in modo leggibile e specifico
T_synced.Properties.VariableNames = {'Tempo [s]', ...
    'X_HTC [m]', 'Y_HTC [m]', 'Z_HTC [m]', ...
    'DegX_HTC [deg]', 'DegY_HTC [deg]', 'DegZ_HTC [deg]', ...
    'Left_ENC [tic]', 'Right_ENC [tic]'};


clearvars -except T_synced

T_synced

