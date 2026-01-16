import zmq
import json
import rerun as rr
import math
import sys
from rerun.archetypes import Scalars 

# --- CONFIGURAZIONE DEBUG ---
# Se vuoi vedere solo i grafici e non il 3D, metti False
SHOW_3D = True 
# Finestra per la media mobile (solo visualizzazione)
SMOOTH_WINDOW = 10 
RS_YAW_OFFSET = 0.0
RS_POS_OFFSET_X = 0.0 
RS_POS_OFFSET_Y = 0.0 


# ----------------------------

# Configurazione
MADS_ENDPOINT = "tcp://localhost:9091"  #Broker port
TOPIC_FILTER = ["odometry_filter", "pose_rs_source", "imu_source"]        #listening topic

class MovingAverage:
    def __init__(self, size):
        self.size = size
        self.data = []
    def update(self, val):
        self.data.append(val)
        if len(self.data) > self.size: self.data.pop(0)
        return sum(self.data) / len(self.data)
    
def get_nested(data, path):
    # 1. Prova accesso diretto (caso flattened)
    if path in data: return data[path]
    if path.startswith("/") and path[1:] in data: return data[path[1:]]

    # 2. Prova navigazione (caso nested)
    keys = path.strip("/").replace(".", "/").split("/")
    curr = data
    try:
        for k in keys:
            # Gestione indici array (es. "position/0")
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
    x_new = -(x * math.cos(theta) - y * math.sin(theta))
    y_new = (x * math.sin(theta) + y * math.cos(theta))
    return x_new, y_new

def main():
    # 1. Start Rerun
    print("Starting Python visualizer...")
    rr.init("MADS_Replication_Python", spawn=True)

    # 2. Connect to MADS (ZeroMQ)
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(MADS_ENDPOINT)
    
    
    # Subscribe to the filter topic
    for topic in TOPIC_FILTER:
        socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        print(f"Subscribed to topic: {topic}")
    
    

    #traj_odometry = []
    #traj_ground_truth = []
    #traj_rs = []
    #traj_rs_aligned = []

    # Stato Odometria (Integrata a mano per confronto)
    odom_x, odom_y, odom_theta = 0.0, 0.0, 0.0
    last_enc_l, last_enc_r = None, None

    R_L = 0.0873
    R_R = 0.0857
    BASELINE = 0.8291
    TICKS = 4096.0

       # Filtri per pulire i grafici
    ma_gyro = MovingAverage(SMOOTH_WINDOW)
    ma_fusion = MovingAverage(SMOOTH_WINDOW)

    while True:
        try:
            # 3. Receive the message
            # MADS sends multipart messages: [Topic, JSON_Payload]
            msg = socket.recv_multipart()
            topic = msg[0].decode('utf-8')
            payload = msg[1].decode('utf-8')
            data = json.loads(payload)

            # 4. Extract data (with error handling)
            time_val = data.get("sim_time")
            if time_val is None: time_val = get_nested(data, "/message/timecode")
            if time_val is not None: 
                rr.set_time_seconds("sim_time", float(time_val))
           
            # Odometry Filter Trajectory - RED
            if topic == "odometry_filter":
                """ 
                if SHOW_3D and "pose_vector" in data:
                    pos = data["pose_vector"] # [x, y, z]
                    traj_odometry.append(pos)
                    # if len(traj_odometry) > 15000: traj_odometry.pop(0)
                    ekf_yaw = data["pose"]["orientation"]["yaw"]
                    #rr.log("debug/absolute_angle/ekf_estimated_yaw", Scalars(ekf_yaw))

                    # Corpo Robot
                    traj_odometry.append(pos)
                    rr.log("robot/est_body", rr.Points3D([pos], radii=0.03, colors=[255, 0, 0], labels="Odom"))
                    # Scia
                    rr.log("robot/est_path", rr.LineStrips3D([traj_odometry], colors=[[255, 0, 0]], radii=0.005))
                """
                if "pose" in data:
                     # FULL EKF PURA        
                    pos = data["pose"]["position"]
                    # Usiamo un nome univoco per la scia raw
                    if not hasattr(main, "traj_pos"): main.traj_pos = []
                    main.traj_pos.append(pos)                    
                    rr.log("robot/FULL_EKF_body", rr.Points3D([pos], radii=0.04, colors=[255, 255, 0], labels="EKF"))
                    rr.log("robot/FULL_EKF_path", rr.LineStrips3D([main.traj_pos], colors=[[255, 255, 0]], radii=0.01)) #sistema colore
                    
                # NUOVO: Grafico Comparativo Angoli (Chi comanda?)
                if "debug" in data:
                    #th_enc = data["debug"].get("theta_enc")
                    #th_imu = data["debug"].get("theta_imu")
                    #th_fus = data["debug"].get("theta_fused")
                    
                    if "theta_enc" in data["debug"]: rr.log("fusion/debug/theta_encoder", Scalars(data["debug"]["theta_enc"]))
                    if "theta_imu" in data["debug"]: rr.log("fusion/debug/theta_imu", Scalars(data["debug"]["theta_imu"]))
                    #if th_fus is not None: rr.log("fusion/debug/theta_fused", Scalars(th_fus))

                    if "fused_velocity" in data["debug"]:
                        rr.log("fusion/debug/velocity_fused", Scalars(data["debug"]["fused_velocity"]))
                
                    # ODOMETRIA PURA        
                    raw_pos = data["debug"]["raw_encoder_only"]
                    # Usiamo un nome univoco per la scia raw
                    if not hasattr(main, "traj_raw_enc"): main.traj_raw_enc = []
                    main.traj_raw_enc.append(raw_pos)                    
                    rr.log("robot/raw_encoder_body", rr.Points3D([raw_pos], radii=0.04, colors=[255, 0, 0], labels="Odometry"))
                    rr.log("robot/raw_encoder_path", rr.LineStrips3D([main.traj_raw_enc], colors=[[255, 0, 0]], radii=0.01))

                     # partial ekf        
                    partial_pos = data["debug"]["partial_ekf"]
                    # Usiamo un nome univoco per la scia raw
                    if not hasattr(main, "traj_partial_ekf"): main.traj_partial_ekf = []
                    main.traj_partial_ekf.append(partial_pos)                    
                    rr.log("robot/partial_ekf_body", rr.Points3D([partial_pos], radii=0.04, colors=[0, 255, 0], labels="Partial EKF"))
                    rr.log("robot/partial_ekf_path", rr.LineStrips3D([main.traj_partial_ekf], colors=[[0, 255, 0]], radii=0.01))

                     # Aruco data      
                    rs_center = data["debug"]["rs_center"]
                    if not hasattr(main, "traj_rs_center"): main.traj_rs_center = []
                    main.traj_rs_center.append(rs_center)
                    rr.log("robot/rs_center_path", rr.LineStrips3D([main.traj_rs_center], colors=[[0, 255, 255]], radii=0.01))
                    rr.log("robot/rs_center_debug", rr.LineStrips3D([[rs_center[0], rs_center[1], 0.0]], colors=[[0, 255, 255]], radii=0.02, labels="RS Center Path"))

                    
                    # Htc position
                    if "htc_position" in data["debug"]:
                        htc_pos = data["debug"]["htc_position"]
                        if not hasattr(main, "traj_htc_position"): main.traj_htc_position = []
                        main.traj_htc_position.append(htc_pos)
                        rr.log("robot/htc_position_path", rr.LineStrips3D([main.traj_htc_position], colors=[[0, 255, 0]], radii=0.01))
                        rr.log("robot/htc_position_debug", rr.Points3D([htc_pos], colors=[[0, 255, 0]], radii=0.02, labels="HTC"))
                    


                    # Acceleration from enc vs imu
                    acc_enc = data["debug"].get("accel_enc")
                    acc_imu = data["debug"].get("accel_imu")
                    is_slipping = data["debug"].get("is_slipping")
                    if acc_enc is not None: rr.log("debug/slip/accel_encoder", Scalars(acc_enc))
                    if acc_imu is not None: rr.log("debug/slip/accel_imu", Scalars(acc_imu))
                    if is_slipping is not None: rr.log("debug/slip/is_slipping_flag", Scalars(is_slipping))

                    ang_rs_raw = get_nested(data, "/debug/angles/rs_raw")
                    ang_rs_unwrapped = get_nested(data, "/debug/angles/rs_unwrapped")
                    ang_fused = get_nested(data, "/debug/angles/fused")
                    ang_enc_only = get_nested(data, "/debug/angles/enc_only")
                    ang_ekf_rs = get_nested(data, "/debug/angles/ekf_rs")

                    if ang_rs_raw is not None: rr.log("debug/angles/rs_raw", Scalars(ang_rs_raw))
                    if ang_rs_unwrapped is not None: rr.log("debug/angles/rs_unwrapped", Scalars(ang_rs_unwrapped))
                    if ang_fused is not None: rr.log("debug/angles/fused", Scalars(ang_fused))
                    if ang_enc_only is not None: rr.log("debug/angles/enc_only", Scalars(ang_enc_only))
                    if ang_ekf_rs is not None: rr.log("debug/angles/ekf_rs", Scalars(ang_ekf_rs))

                """
                # Ground Truth Trajectory from HTC - GREEN
                elif topic == "pose_htc_source":
                    x = get_nested(data, "/message/pose/position/0")
                    y = get_nested(data, "/message/pose/position/1")

                    #if x is  None: x = get_nested(data, "/message/pose/position/0")
                    #if y is  None: y = get_nested(data, "/message/pose/position/1")

                    if x is not None and y is not None:
                        htc_pos = [float(x), float(y), 0.0]
                        if not hasattr(main, "traj_ground_truth"): main.traj_ground_truth = []
                        main.traj_ground_truth.append(htc_pos)

                        rr.log("robot/gt_body", rr.Points3D([htc_pos], radii=0.03, colors=[0, 255, 0], labels="GT"))
                        rr.log("robot/gt_path", rr.LineStrips3D([main.traj_ground_truth], colors=[[0, 255, 0]], radii=0.005))
                """
                    
                """
                # RealSense Trajectory - CYAN
                elif topic == "pose_rs_source":
                    # Leggi posizione RealSense
                    rs_x = get_nested(data, "/message/pose/position/0/0")
                    rs_y = - get_nested(data, "/message/pose/position/0/1")
                    
                    if rs_x is None: # Fallback formato piatto
                        rs_x = get_nested(data, "/message/pose/position/0")
                        rs_y = - get_nested(data, "/message/pose/position/1")

                    if rs_x is not None:
                        # A. Dato Grezzo (Come arriva dal sensore)
                        traj_rs.append([rs_x, rs_y, 0.0])
                        #if len(traj_rs) > 5000: traj_rs.pop(0)

                        # B. Dato Allineato (Ruotato di 135 gradi)
                        # Applichiamo la rotazione al punto
                        rot_x, rot_y = rotate_point(rs_x, rs_y, RS_YAW_OFFSET)
                        
                        # Aggiungiamo l'offset di traslazione (se serve)
                        aligned_x = rot_x + RS_POS_OFFSET_X
                        aligned_y = rot_y + RS_POS_OFFSET_Y
                        
                        traj_rs_aligned.append([aligned_x, aligned_y, 0.0])
                        #if len(traj_rs_aligned) > 5000: traj_rs_aligned.pop(0)
                        
                        # Disegna Allineato (Verde - Corretto?)
                        rr.log("geometry/realsense", rr.LineStrips3D([traj_rs_aligned], colors=[[0, 255, 0]], labels=f"Aruco"))
                        
                """

            # IMU Data Visualization - GRAPHS
            elif topic == "imu_source":
                """
                # Imu left
                acc_l_x = get_nested(data, "/message/imu/left/accelerations/x")
                acc_l_y = get_nested(data, "/message/imu/left/accelerations/y")
                if acc_l_x is not None: rr.log("sensors/imu/left/acc/x", Scalars(acc_l_x))
                if acc_l_y is not None: rr.log("sensors/imu/left/acc/y", Scalars(acc_l_y))

                # Imu middle
                gyro_z = get_nested(data, "/message/imu/middle/gyroscopes/z")
                if gyro_z is not None: rr.log("sensors/imu/middle/gyro/z", Scalars(gyro_z))

                # Imu right
                acc_r_x = get_nested(data, "/message/imu/right/accelerations/x")
                if acc_r_x is not None: rr.log("sensors/imu/right/acc/x", Scalars(acc_r_x))
                
                # Accelerometro: message -> accel -> [x, y, z]
                acc_x = get_nested(data, "/message/accel/0")
                acc_y = get_nested(data, "/message/accel/1")
                """
                # Giroscopio: message -> gyro -> [x, y, z] (Z è l'indice 2)
                gyro_z = get_nested(data, "/message/gyro/2")
                if gyro_z is not None:
                    try:
                        rr.log("sensors/imu/gyro/z_inverted", Scalars(-gyro_z))
                    except:
                        pass

                """ 
                if acc_x is not None: rr.log("sensors/imu/acc/x", Scalars(acc_x))
                if acc_y is not None: rr.log("sensors/imu/acc/y", Scalars(acc_y))

                # NUOVO: Plotta la "FusionPose" interna del chip
                fusion_yaw = get_nested(data, "/message/fusionPose/2")
                if fusion_yaw is not None:
                    # Logghiamo sia raw che smooth
                    rr.log("debug/absolute_angle/fusion_yaw_raw", Scalars(fusion_yaw))
                    rr.log("debug/absolute_angle/fusion_yaw_smooth", Scalars(ma_fusion.update(fusion_yaw)))
                
                mag_x = get_nested(data, "/message/compass/0")
                mag_y = get_nested(data, "/message/compass/1")
                if mag_x is not None: rr.log("debug/magnetometer/x", Scalars(mag_x))
                if mag_y is not None: rr.log("debug/magnetometer/y", Scalars(mag_y))
                """ 


                

        except KeyboardInterrupt:
            print("Manual interruption.")
            break
        except Exception as e:
            print(f"Parsing error: {e}")

if __name__ == "__main__":
    main()