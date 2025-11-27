import zmq
import json
import rerun as rr
import sys

MADS_ENDPOINT = "tcp://localhost:9091"
TOPICS = ["odometry_filter", "pose_htc_source", "imu_source"]

def get_nested(data, path_str):
    """
    Naviga nel JSON usando una stringa percorso (es. "/message/imu/left/accelerations/x")
    """
    keys = path_str.strip("/").split("/")
    curr = data
    try:
        for k in keys:
            if isinstance(curr, dict) and k in curr:
                curr = curr[k]
            elif isinstance(curr, list) and k.isdigit(): # Gestione array
                idx = int(k)
                if idx < len(curr): curr = curr[idx]
                else: return None
            else:
                return None
        return float(curr) # Converte in numero per Rerun
    except:
        return None

def main():
    print("🚀 Visualizzatore MADS (Calibrato su JSON Reale)...")
    rr.init("MADS_Simulation", spawn=True)
    
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(MADS_ENDPOINT)
    for t in TOPICS: 
        socket.setsockopt_string(zmq.SUBSCRIBE, t)
        print(f"📡 Iscritto a: {t}")

    traj_odom = []
    traj_gt = []

    while True:
        try:
            msg = socket.recv_multipart()
            topic = msg[0].decode('utf-8')
            payload = msg[1].decode('utf-8')
            data = json.loads(payload)

            # --- 1. GESTIONE TEMPO (Cruciale!) ---
            # Priorità: 
            # 1. sim_time (dal filtro odometria)
            # 2. /message/timecode (dai source raw come IMU e HTC)
            time_val = data.get("sim_time")
            if time_val is None: 
                time_val = get_nested(data, "/message/timecode")
            
            if time_val is not None:
                # Impostiamo il tempo per allineare tutti i grafici
                rr.set_time_seconds("sim_time", time_val)

            # --- 2. ODOMETRIA (ROSSO) ---
            if topic == "odometry_filter" and "pose_vector" in data:
                pos = data["pose_vector"]
                traj_odom.append(pos)
                if len(traj_odom) > 5000: traj_odom.pop(0)
                
                rr.log("robot/est_body", rr.Points3D([pos], radii=0.05, colors=[255, 0, 0], labels="Odom"))
                rr.log("robot/est_path", rr.LineStrips3D([traj_odom], colors=[[255, 0, 0]], radii=0.01))

            # --- 3. GROUND TRUTH (VERDE) ---
            elif topic == "pose_htc_source":
                # JSON HTC: /message/pose/position/0 (x), 1 (y)
                x = get_nested(data, "/message/pose/position/0")
                y = get_nested(data, "/message/pose/position/1")
                
                if x is not None and y is not None:
                    pos = [x, y, 0.0]
                    traj_gt.append(pos)
                    if len(traj_gt) > 5000: traj_gt.pop(0)
                    
                    rr.log("robot/gt_body", rr.Points3D([pos], radii=0.05, colors=[0, 255, 0], labels="GT"))
                    rr.log("robot/gt_path", rr.LineStrips3D([traj_gt], colors=[[0, 255, 0]], radii=0.01))

            # --- 4. IMU (GRAFICI BLU/GIALLI) ---
            elif topic == "imu_source":
                # Basato sul tuo JSON: message -> imu -> [left/middle/right] -> [accelerations/gyroscopes] -> [x/y/z]
                
                # --- IMU SINISTRO (Accelerazioni) ---
                acc_l_x = get_nested(data, "/message/imu/left/accelerations/x")
                acc_l_y = get_nested(data, "/message/imu/left/accelerations/y")
                
                if acc_l_x is not None: rr.log("sensors/imu/left/acc/x", rr.Scalar(acc_l_x))
                if acc_l_y is not None: rr.log("sensors/imu/left/acc/y", rr.Scalar(acc_l_y))

                # --- IMU CENTRALE (Giroscopio - Importante per la rotazione) ---
                gyro_m_z = get_nested(data, "/message/imu/middle/gyroscopes/z")
                if gyro_m_z is not None: rr.log("sensors/imu/middle/gyro/z", rr.Scalar(gyro_m_z))

                # --- IMU DESTRO (Accelerazioni) ---
                acc_r_x = get_nested(data, "/message/imu/right/accelerations/x")
                if acc_r_x is not None: rr.log("sensors/imu/right/acc/x", rr.Scalar(acc_r_x))

        except KeyboardInterrupt:
            break
        except Exception as e:
            pass

if __name__ == "__main__":
    main()