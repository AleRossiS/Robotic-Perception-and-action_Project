import zmq
import json
import rerun as rr
import math
import sys

# --- CONFIGURAZIONE GEOMETRICA ---
# Angolo di rotazione tra Walker e Aruco/RealSense
# Il diagramma suggerisce 135 gradi? Proviamolo.
# 135 gradi in radianti = 135 * (3.14 / 180) = 2.356
# Prova anche -135 (-2.356) o 45 (0.785) se non combacia.
RS_YAW_OFFSET = 135.0 * (3.14159 / 180.0)

# Offset di posizione (Distanza dal centro)
# Se la camera è spostata rispetto al centro, inserisci qui i metri
RS_POS_OFFSET_X = 0.0 
RS_POS_OFFSET_Y = 0.0
# ---------------------------------

MADS_ENDPOINT = "tcp://localhost:9091"
TOPICS = ["encoders_source", "pose_rs_source", "imu_source"]

def get_nested(data, path):
    keys = path.strip("/").split("/")
    curr = data
    try:
        for k in keys:
            if isinstance(curr, list) and k.isdigit():
                idx = int(k)
                if idx < len(curr): curr = curr[idx]
                else: return None
            elif isinstance(curr, dict) and k in curr:
                curr = curr[k]
            else:
                return None
        return float(curr)
    except:
        return None

def rotate_point(x, y, theta):
    """Ruota un punto (x,y) di un angolo theta."""
    x_new = x * math.cos(theta) - y * math.sin(theta)
    y_new = x * math.sin(theta) + y * math.cos(theta)
    return x_new, y_new

def main():
    print(f"🚀 Visualizer: Test Rotazione {math.degrees(RS_YAW_OFFSET):.1f}°")
    rr.init("MADS_Geometry_Check", spawn=True)
    
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(MADS_ENDPOINT)
    for t in TOPICS: 
        socket.setsockopt_string(zmq.SUBSCRIBE, t)

    # Accumulatori per le scie
    path_odom = []
    path_rs_raw = []
    path_rs_aligned = []

    # Stato Odometria (Integrata a mano per confronto)
    odom_x, odom_y, odom_theta = 0.0, 0.0, 0.0
    last_enc_l, last_enc_r = None, None
    
    # Parametri Walker (per calcolare odom grezza)
    R_L = 0.0873
    R_R = 0.0857
    BASELINE = 0.8291
    TICKS = 4096.0

    print("In attesa di dati...")

    while True:
        try:
            msg = socket.recv_multipart()
            topic = msg[0].decode('utf-8')
            payload = msg[1].decode('utf-8')
            data = json.loads(payload)

            # Tempo
            time_val = data.get("sim_time")
            if time_val is None: time_val = get_nested(data, "/message/timecode")
            if time_val is not None:
                rr.set_time_seconds("sim_time", float(time_val))

            # --- 1. ODOMETRIA (Calcolata al volo per avere un riferimento) ---
            if topic == "encoders_source":
                # Leggi encoder
                enc_l = get_nested(data, "/message/encoders/left")
                enc_r = get_nested(data, "/message/encoders/right")
                
                if last_enc_l is not None and enc_l is not None:
                    # Calcola delta movimento
                    dl = ((enc_l - last_enc_l) / TICKS) * 2 * math.pi * R_L
                    dr = ((enc_r - last_enc_r) / TICKS) * 2 * math.pi * R_R
                    ds = (dr + dl) / 2.0
                    dth = (dr - dl) / BASELINE
                    
                    # Integra posizione (Semplice Eulero)
                    odom_x += ds * math.cos(odom_theta)
                    odom_y += ds * math.sin(odom_theta)
                    odom_theta += dth
                    
                    pos = [odom_x, odom_y, 0.0]
                    path_odom.append(pos)
                    if len(path_odom) > 5000: path_odom.pop(0)
                    
                    # Disegna Odometria (RIFERIMENTO - Bianco/Grigio)
                    rr.log("geometry/walker_odom", rr.LineStrips3D([path_odom], colors=[[200, 200, 200]], labels="Walker Frame (Odom)"))

                if enc_l is not None:
                    last_enc_l = enc_l
                    last_enc_r = enc_r

            # --- 2. REALSENSE (Raw vs Aligned) ---
            elif topic == "pose_rs_source":
                # Leggi posizione RealSense
                rs_x = get_nested(data, "/message/pose/position/0/0")
                rs_y = get_nested(data, "/message/pose/position/0/1")
                
                if rs_x is None: # Fallback formato piatto
                    rs_x = get_nested(data, "/message/pose/position/0")
                    rs_y = get_nested(data, "/message/pose/position/1")

                if rs_x is not None:
                    # A. Dato Grezzo (Come arriva dal sensore)
                    path_rs_raw.append([rs_x, rs_y, 0.0])
                    if len(path_rs_raw) > 5000: path_rs_raw.pop(0)
                    # Disegna Raw (Rosso - Errato)
                    rr.log("geometry/realsense_raw", rr.LineStrips3D([path_rs_raw], colors=[[255, 0, 0]], labels="RS Raw"))

                    # B. Dato Allineato (Ruotato di 135 gradi)
                    # Applichiamo la rotazione al punto
                    rot_x, rot_y = rotate_point(rs_x, rs_y, RS_YAW_OFFSET)
                    
                    # Aggiungiamo l'offset di traslazione (se serve)
                    aligned_x = rot_x + RS_POS_OFFSET_X
                    aligned_y = rot_y + RS_POS_OFFSET_Y
                    
                    path_rs_aligned.append([aligned_x, aligned_y, 0.0])
                    if len(path_rs_aligned) > 5000: path_rs_aligned.pop(0)
                    
                    # Disegna Allineato (Verde - Corretto?)
                    rr.log("geometry/realsense_aligned", rr.LineStrips3D([path_rs_aligned], colors=[[0, 255, 0]], labels=f"RS Rotated {math.degrees(RS_YAW_OFFSET):.0f}°"))

            # --- 3. IMU (Solo per controllo) ---
            elif topic == "imu_source":
                # Gyro Z
                gyro_z = get_nested(data, "/message/gyro/2")
                if gyro_z is not None:
                    try:
                        from rerun.archetypes import Scalar
                        rr.log("sensors/gyro_z", Scalar(gyro_z))
                    except:
                        pass

        except KeyboardInterrupt:
            break
        except Exception:
            pass

if __name__ == "__main__":
    main()