%% --- 1. Caricamento Dati ---

addpath("data");

% Specifica il nome del tuo file
fileNameHTC = 'mads_calibration.absolute_pose_walker_for_htc.csv'; % Assicurati che abbia l'estensione giusta
fileNameRS  = 'mads_calibration.absolute_pose_walker_for_realsense.csv';
fileNameENC = 'mads_calibration.encoders.csv';
fileNameIMU = 'mads_calibration.imu.csv';

% Imposta le opzioni di importazione
optsHTC = detectImportOptions(fileNameHTC);
optsRS  = detectImportOptions(fileNameRS);
optsENC = detectImportOptions(fileNameENC);
optsIMU = detectImportOptions(fileNameIMU);

% I tuoi dati iniziano dalla seconda riga (la prima è l'header)
optsHTC.DataLines = 2; 
optsRS.DataLines  = 2; 
optsENC.DataLines = 2; 
optsIMU.DataLines = 2; 

% Leggi i dati in una tabella
try
    dataTableHTC = readtable(fileNameHTC, optsHTC);
    dataTableRS  = readtable(fileNameRS, optsRS);
    dataTableENC = readtable(fileNameENC, optsENC);
    dataTableIMU = readtable(fileNameIMU, optsIMU);
    
    % Assegna nomi alle colonne (opzionale, ma utile)
    % Dovrai contare le colonne e associare i nomi dall'header
    % Per ora, MATLAB le chiamerà Var1, Var2, ...
    disp('Dati HTC:');
    head(dataTableHTC, 5); % Mostra le prime 5 righe
    disp('Dati RealSense:');
    head(dataTableRS,  5);
    disp('Dati ENC:');
    head(dataTableENC, 5);
    disp('Dati IMU:');
    head(dataTableIMU, 5);

catch e
    disp('Errore durante il caricamento del file:');
    disp(e.message);
    disp('Controlla che il file sia nella stessa cartella o aggiungi il percorso (addpath).');
end

% LEGENDA
% H_i_j indica la posizione nella matrice omogenea di trasformazione H
% R_i_j indica la posizione nella matrice di rotazione R
% pose_attitude_deg_i/j/k indicano gli angolo di rotazione
% pose_position_i/j/k indicano le posizioni in x y z




%% --- 2. Conversione Timestamp ---

% Definiamo il formato. La 'Z' sta per UTC (Zulu time).
% .SSS significa 3 cifre per i millisecondi. Se ne hai 6, usa .SSSSSS
formatString = 'yyyy-MM-dd''T''HH:mm:ss.SSS''Z''';

% --- Master Clock (HTC) ---
% 1. Converti le stringhe in oggetti 'datetime'
dataTableHTC.Time = datetime(dataTableHTC.message_timestamp, 'InputFormat', formatString, 'TimeZone', 'UTC');

% 2. Definiamo il "tempo zero" (t0) come il primo timestamp dell'HTC
t0 = dataTableHTC.Time(1); 

% 3. Creiamo la nostra colonna di tempo 'master' in secondi
master_time_seconds = seconds(dataTableHTC.Time - t0);

% --- Dati "Slave" (RS) ---
% Fai la stessa cosa per gli altri file, ma usa lo STESSO t0!
dataTableRS.Time = datetime(dataTableRS.message_timestamp, 'InputFormat', formatString, 'TimeZone', 'UTC');
rs_time_seconds = seconds(dataTableRS.Time - t0);

% --- Dati "Slave" (IMU) ---
% Fai la stessa cosa per gli altri file, ma usa lo STESSO t0!
dataTableIMU.Time = datetime(dataTableIMU.message_timestamp, 'InputFormat', formatString, 'TimeZone', 'UTC');
imu_time_seconds = seconds(dataTableIMU.Time - t0);

% --- Dati "Slave" (Encoders) ---
dataTableENC.Time = datetime(dataTableENC.message_timestamp, 'InputFormat', formatString, 'TimeZone', 'UTC');
encoder_time_seconds = seconds(dataTableENC.Time - t0);

disp('Timestamp convertiti in secondi.')

%% --- 3. Interpolazione (Ricampionamento) ---

% La sintassi è: interp1(X_originali, Y_originali, X_desiderati)

% --- Sincronizza i dati RS ---
X_RS = interp1(rs_time_seconds, ...
                          dataTableRS.message_pose_position_0_, ...
                          master_time_seconds);
Y_RS = interp1(rs_time_seconds, ...
                          dataTableRS.message_pose_position_1_, ...
                          master_time_seconds);
Z_RS = interp1(rs_time_seconds, ...
                          dataTableRS.message_pose_position_2_, ...
                          master_time_seconds);

DegX_RS = interp1(rs_time_seconds, ...
                          dataTableRS.message_pose_attitude_deg_0_, ...
                          master_time_seconds);
DegY_RS = interp1(rs_time_seconds, ...
                          dataTableRS.message_pose_attitude_deg_1_, ...
                          master_time_seconds);
DegZ_RS = interp1(rs_time_seconds, ...
                          dataTableRS.message_pose_attitude_deg_2_, ...
                          master_time_seconds);


% --- Sincronizza i dati IMU ---
% Sincronizziamo il giroscopio X (devi usare i nomi esatti delle tue colonne)
OmegaXl_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_left_gyroscopes_x, ...
                          master_time_seconds);
OmegaXr_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_right_gyroscopes_x, ...
                          master_time_seconds);
OmegaXm_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_middle_gyroscopes_x, ...
                          master_time_seconds);

AlphaXl_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_left_accelerations_x, ...
                          master_time_seconds);
AlphaXr_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_right_accelerations_x, ...
                          master_time_seconds);
AlphaXm_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_middle_accelerations_x, ...
                          master_time_seconds);

% Sincronizziamo il giroscopio Y
OmegaYl_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_left_gyroscopes_y, ...
                          master_time_seconds);
OmegaYr_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_right_gyroscopes_y, ...
                          master_time_seconds);
OmegaYm_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_middle_gyroscopes_y, ...
                          master_time_seconds);

AlphaYl_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_left_accelerations_y, ...
                          master_time_seconds);
AlphaYr_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_right_accelerations_y, ...
                          master_time_seconds);
AlphaYm_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_middle_accelerations_y, ...
                          master_time_seconds);


% Sincronizziamo il giroscopio Z
OmegaZl_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_left_gyroscopes_z, ...
                          master_time_seconds);
OmegaZr_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_right_gyroscopes_z, ...
                          master_time_seconds);
OmegaZm_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_middle_gyroscopes_z, ...
                          master_time_seconds);

AlphaZl_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_left_accelerations_z, ...
                          master_time_seconds);
AlphaZr_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_right_accelerations_z, ...
                          master_time_seconds);
AlphaZm_IMU = interp1(imu_time_seconds, ...
                          dataTableIMU.message_imu_middle_accelerations_z, ...
                          master_time_seconds);

% ... fai questo per OGNI colonna di IMU che ti serve (gyro_z, accel_x, ecc.)


% --- Sincronizza i dati ENCODER ---
encoder_left_sync = interp1(encoder_time_seconds, ...
                            dataTableENC.message_encoders_left, ...
                            master_time_seconds);
                        
encoder_right_sync = interp1(encoder_time_seconds, ...
                             dataTableENC.message_encoders_right, ...
                             master_time_seconds);

% ... fai questo per OGNI colonna di Encoder che ti serve (currents, ecc.)

disp('Dati "slave" interpolati sul master clock.')

%% --- 4. Assemblare la Tabella Sincronizzata ---

% Iniziamo creando una tabella con il tempo e i dati 'reali' (ground truth)
% Nota l'uso di T.("...") per i nomi con caratteri speciali!
T_synced = table(master_time_seconds, ...
                 dataTableHTC.("message_pose_position_0_"), ...
                 dataTableHTC.("message_pose_position_1_"), ...
                 dataTableHTC.("message_pose_position_2_"), ...
                 dataTableHTC.("message_pose_attitude_deg_0_"), ... %SALVO SOLO ANGOLI IN GRADI, GLI ALTRI 3 SONO IN RADIANTI
                 dataTableHTC.("message_pose_attitude_deg_1_"), ...
                 dataTableHTC.("message_pose_attitude_deg_2_"));
             

% Ora aggiungiamo i dati sincronizzati (Input del modello)

T_synced.X_RS=X_RS;
T_synced.Y_RS=Y_RS;
T_synced.Z_RS=Z_RS;
T_synced.DegX_RS=DegX_RS;
T_synced.DegY_RS=DegX_RS;
T_synced.DegZ_RS=DegX_RS;

T_synced.OmegaXl_IMU = OmegaXl_IMU; % velocità angolari
T_synced.OmegaXr_IMU = OmegaXr_IMU;
T_synced.OmegaXm_IMU = OmegaXm_IMU;
T_synced.OmegaYl_IMU = OmegaYl_IMU;
T_synced.OmegaYr_IMU = OmegaYl_IMU;
T_synced.OmegaYm_IMU = OmegaYl_IMU;
T_synced.OmegaZl_IMU = OmegaZl_IMU;
T_synced.OmegaZr_IMU = OmegaZr_IMU;
T_synced.OmegaZm_IMU = OmegaZm_IMU;
T_synced.AlphaXl_IMU = AlphaXl_IMU; % accelerazioni
T_synced.AlphaXr_IMU = AlphaXr_IMU;
T_synced.AlphaXm_IMU = AlphaXm_IMU;
T_synced.AlphaYl_IMU = AlphaYl_IMU;
T_synced.AlphaYr_IMU = AlphaYl_IMU;
T_synced.AlphaYm_IMU = AlphaYl_IMU;
T_synced.AlphaZl_IMU = AlphaZl_IMU;
T_synced.AlphaZr_IMU = AlphaZr_IMU;
T_synced.AlphaZm_IMU = AlphaZm_IMU;

T_synced.encoder_left = encoder_left_sync;
T_synced.encoder_right = encoder_right_sync;

% ... aggiungi tutte le altre colonne sincronizzate che hai creato ...

% Rinominiamo le colonne in modo leggibile
T_synced.Properties.VariableNames = {'Tempo [s]', 'X_HTC [m]', 'Y_HTC [m]', 'Z_HTC [m]','DegX_HTC [deg]', 'DegY_HTC [deg]', 'DegZ_HTC [deg]', ...
    'X_RS [m]', 'Y_RS [m]', 'Z_RS [m]','DegX_RS [deg]', 'DegY_RS [deg]', 'DegZ_RS [deg]', ...
    'OmegaXl_IMU [deg/s]','OmegaXr_IMU [deg/s]','OmegaXm_IMU [deg/s]', 'OmegaYl_IMU [deg/s]','OmegaYr_IMU [deg/s]','OmegaYm_IMU [deg/s]', 'OmegaZl_IMU [deg/s]', 'OmegaZr_IMU [deg/s]', 'OmegaZm_IMU [deg/s]', ...
    'AlphaXl_IMU [deg/s]','AlphaXr_IMU [deg/s]','AlphaXm_IMU [deg/s]', 'AlphaYl_IMU [deg/s]','AlphaYr_IMU [deg/s]','AlphaYm_IMU [deg/s]', 'AlphaZl_IMU [deg/s]', 'AlphaZr_IMU [deg/s]', 'AlphaZm_IMU [deg/s]', ...
    'Left_ENC [tic]', 'Right_ENC [tic]' };

% Finito!
disp('Tabella sincronizzata creata con successo:');
head(T_synced, 5) % Mostra le prime 5 righe


% Ora puoi accedere ai dati, ad esempio alla colonna di timestamp (supponiamo sia la 2a)
% timestamps = dataTable.Var2;
% pos_x = dataTable.Var38; % Esempio: message.pose.position[0] (devi contare la colonna giusta)