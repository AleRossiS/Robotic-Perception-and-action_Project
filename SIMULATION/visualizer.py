import zmq
import json
import rerun as rr
import sys
from rerun.archetypes import Scalars 

# Configurazione
MADS_ENDPOINT = "tcp://localhost:9091"  #Broker port
TOPIC_FILTER = ["odometry_filter", "pose_htc_source", "pose_rs_source", "imu_source"]        #listening topic

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
            if isinstance(curr, list):
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
    

    traj_odometry = []
    traj_ground_truth = []
    traj_rs = []

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
            if time_val is None: time_val = get_nested(data, "/timecode")
            if time_val is not None: 
                rr.set_time_seconds("sim_time", float(time_val))

            
            # Odometry Filter Trajectory - RED
            if topic == "odometry_filter":
                pos = data["pose_vector"] # [x, y, z]
                traj_odometry.append(pos)
                if len(traj_odometry) > 6000: traj_odometry.pop(0)

                # Corpo Robot
                rr.log("robot/est_body", rr.Points3D([pos], radii=0.03, colors=[255, 0, 0], labels="Odom"))
                # Scia
                rr.log("robot/est_path", rr.LineStrips3D([traj_odometry], colors=[[255, 0, 0]], radii=0.005))

            # Ground Truth Trajectory from HTC - GREEN
            elif topic == "pose_htc_source":
                x = get_nested(data, "/message/pose/position/0")
                y = get_nested(data, "/message/pose/position/1")
                z = get_nested(data, "/message/pose/position/2")
                
                if x is not None and y is not None:
                    pos = [float(x), float(y), float(z) if z else 0.0]
                    traj_ground_truth.append(pos)
                    if len(traj_ground_truth)>6000: traj_ground_truth.pop(0)
                    rr.log("robot/gt_body", rr.Points3D([pos], radii=0.03, colors=[0, 255, 0], labels="GT"))
                    rr.log("robot/gt_path", rr.LineStrips3D([traj_ground_truth], colors=[[0, 255, 0]], radii=0.005))
            
            # RealSense Trajectory - CYAN
            elif topic == "pose_rs_source":
                x = get_nested(data, "/message/pose/position/0")
                y = get_nested(data, "/message/pose/position/1")

                yaw = get_nested(data, "/message/pose/attitude_along_z")
                
                if x is not None and y is not None:
                    pos = [float(x), float(y), 0.0]
                    traj_rs.append(pos)
                    if len(traj_rs)>6000: traj_rs.pop(0)
                    rr.log("robot/rs_body", rr.Points3D([pos], radii=0.03, colors=[0, 255, 255], labels="RS"))
                    rr.log("robot/rs_path", rr.LineStrips3D([traj_rs], colors=[[0, 255, 255]], radii=0.005))
                    
                

            # IMU Data Visualization - GRAPHS
            elif topic == "imu_source":
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
        except KeyboardInterrupt:
            print("Manual interruption.")
            break
        except Exception as e:
            print(f"Parsing error: {e}")

if __name__ == "__main__":
    main()