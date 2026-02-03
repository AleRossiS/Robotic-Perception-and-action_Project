import zmq
import json
import rerun as rr
import math
import sys
from rerun.archetypes import Scalars 

# --- CONFIGURAZIONE DEBUG ---
SHOW_3D = True 
RS_YAW_OFFSET = 0.0 
RS_POS_OFFSET_X = 0.0 
RS_POS_OFFSET_Y = 0.0 
# ----------------------------

MADS_ENDPOINT = "tcp://localhost:9091"
TOPIC_FILTER = ["odometry_filter", "pose_rs_source", "imu_source", "pose_htc_source"]

def get_nested(data, path):
    if path in data: return data[path]
    if path.startswith("/") and path[1:] in data: return data[path[1:]]
    keys = path.strip("/").replace(".", "/").split("/")
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
    x_new = -(x * math.cos(theta) - y * math.sin(theta))
    y_new = (x * math.sin(theta) + y * math.cos(theta))
    return x_new, y_new

def main():
    print("Starting Python visualizer...")
    rr.init("MADS_Replication_Python", spawn=True)
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(MADS_ENDPOINT)
    
    for topic in TOPIC_FILTER:
        socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        print(f"Subscribed to topic: {topic}")

    traj_rs_aligned = []

    while True:
        try:
            msg = socket.recv_multipart()
            topic = msg[0].decode('utf-8')
            payload = msg[1].decode('utf-8')
            data = json.loads(payload)

            time_val = data.get("sim_time")
            if time_val is None: time_val = get_nested(data, "/message/timecode")
            if time_val is not None: rr.set_time_seconds("sim_time", float(time_val))
           
            if topic == "odometry_filter":
                if "pose" in data:
                    pos = data["pose"]["position"]
                    if not hasattr(main, "traj_pos"): main.traj_pos = []
                    main.traj_pos.append(pos)                    
                    rr.log("robot/FULL_EKF_body", rr.Points3D([pos], radii=0.04, colors=[255, 255, 0], labels="EKF"))
                    rr.log("robot/FULL_EKF_path", rr.LineStrips3D([main.traj_pos], colors=[[255, 255, 0]], radii=0.01))
                    
                if "debug" in data:
                    # Grafici Angoli
                    if "theta_enc" in data["debug"]: rr.log("fusion/debug/theta_encoder", Scalars(data["debug"]["theta_enc"]))
                    if "theta_imu" in data["debug"]: rr.log("fusion/debug/theta_imu", Scalars(data["debug"]["theta_imu"]))
                    
                    # Grafici Velocità
                    if "v_fused" in data["debug"]: rr.log("fusion/debug/velocity_fused", Scalars(data["debug"]["v_fused"]))

                    # Traiettorie Parziali
                    if "raw_encoder_only" in data["debug"]:
                        raw_pos = data["debug"]["raw_encoder_only"]
                        if not hasattr(main, "traj_raw_enc"): main.traj_raw_enc = []
                        main.traj_raw_enc.append(raw_pos)                    
                        rr.log("robot/raw_encoder_path", rr.LineStrips3D([main.traj_raw_enc], colors=[[255, 0, 0]], radii=0.01))

                    if "partial_ekf" in data["debug"]:
                        partial_pos = data["debug"]["partial_ekf"]
                        if not hasattr(main, "traj_partial_ekf"): main.traj_partial_ekf = []
                        main.traj_partial_ekf.append(partial_pos)                    
                        rr.log("robot/partial_ekf_path", rr.LineStrips3D([main.traj_partial_ekf], colors=[[0, 255, 0]], radii=0.01))

                    if "rs_center" in data["debug"]:
                        rs_center = data["debug"]["rs_center"]
                        if not hasattr(main, "traj_rs_center"): main.traj_rs_center = []
                        main.traj_rs_center.append(rs_center)
                        rr.log("robot/rs_center_debug", rr.LineStrips3D([[rs_center[0], rs_center[1], 0.0]], colors=[[0, 255, 255]], radii=0.02))

                    # Slip Check Graph
                    acc_enc = data["debug"].get("accel_enc")
                    acc_imu = data["debug"].get("accel_imu")
                    is_slipping = data["debug"].get("is_slipping")
                    if acc_enc is not None: rr.log("debug/slip/accel_encoder", Scalars(acc_enc))
                    if acc_imu is not None: rr.log("debug/slip/accel_imu", Scalars(acc_imu))
                    if is_slipping is not None: rr.log("debug/slip/is_slipping_flag", Scalars(is_slipping))

            # HTC Ground Truth (Green)
            elif topic == "pose_htc_source":
                x = get_nested(data, "/message/pose/position/0")
                y = get_nested(data, "/message/pose/position/1")
                if x is not None and y is not None:
                    pos = [float(x), float(y), 0.0]
                    if not hasattr(main, "traj_ground_truth"): main.traj_ground_truth = []
                    main.traj_ground_truth.append(pos)
                    rr.log("robot/gt_path", rr.LineStrips3D([main.traj_ground_truth], colors=[[0, 255, 0]], radii=0.005))

            # RealSense Raw (Cyan)
            elif topic == "pose_rs_source":
                # Lettura robusta (fix per NoneType error)
                raw_x = get_nested(data, "/message/pose/position/0/0")
                raw_y = get_nested(data, "/message/pose/position/0/1")
                
                if raw_x is None: # Fallback
                    raw_x = get_nested(data, "/message/pose/position/0")
                    raw_y = get_nested(data, "/message/pose/position/1")

                if raw_x is not None and raw_y is not None:
                    # Applica segno solo se i dati esistono
                    rs_x = raw_x
                    rs_y = -raw_y 

                    rot_x, rot_y = rotate_point(rs_x, rs_y, RS_YAW_OFFSET)
                    aligned_x = rot_x + RS_POS_OFFSET_X
                    aligned_y = rot_y + RS_POS_OFFSET_Y
                    traj_rs_aligned.append([aligned_x, aligned_y, 0.0])
                    rr.log("geometry/realsense_raw", rr.LineStrips3D([traj_rs_aligned], colors=[[0, 255, 255]], radii=0.005))

        except KeyboardInterrupt:
            break
        except Exception as e:
            pass # Ignora errori di parsing sporadici

if __name__ == "__main__":
    main()