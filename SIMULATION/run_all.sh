#!/bin/bash

# Configurazione
MADS_PATH="." # Siamo già nella cartella giusta

echo "Avvio Simulazione MADS Completa..."

# 1. Pulisci vecchi processi
echo "Pulizia processi precedenti..."
pkill -f mads
pkill -f visualizer.py
sleep 1

# 2. Lancia il Visualizzatore Python (in background)
echo "Avvio Visualizzatore..."
python3 visualizer.py &
VIS_PID=$!
sleep 2 # Aspetta che parta

# 3. Lancia il Broker
echo "Avvio Broker..."
mads broker > /dev/null 2>&1 & 
# Nota: nascondiamo i log del broker per pulizia, togli "> /dev/null 2>&1" se vuoi vederli
BROKER_PID=$!
sleep 1

# 4. Lancia il Filtro (Odometria)
echo "Avvio Filtro Odometria..."
mads filter odometry_filter.plugin -n "odometry_filter" &
FILTER_PID=$!

# 5. Lancia i Source (Tutti i sensori!)
echo "uD83DuDCE6 Avvio Source: Encoders..."
mads source replay.plugin -n "encoders_source" &
SRC_ENC_PID=$!

echo "uD83DuDCE6 Avvio Source: IMU..."
mads source replay.plugin -n "imu_source" &
SRC_IMU_PID=$!

echo "uD83DuDCE6 Avvio Source: Ground Truth (HTC)..."
mads source replay.plugin -n "pose_htc_source" &
SRC_HTC_PID=$!

# Funzione per chiudere tutto quando premi Ctrl+C
cleanup() {
    echo ""
    echo "Arresto simulazione..."
    kill $VIS_PID $BROKER_PID $FILTER_PID $SRC_ENC_PID $SRC_IMU_PID $SRC_HTC_PID 2>/dev/null
    pkill -f mads
    exit
}

# Intercetta Ctrl+C
trap cleanup SIGINT

echo "Tutto avviato! Premi Ctrl+C per terminare."
wait