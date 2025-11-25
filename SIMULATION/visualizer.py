import zmq
import json
import rerun as rr
import sys

# Configurazione
MADS_ENDPOINT = "tcp://localhost:9091"  #Broker port
TOPIC_FILTER = "odometry_filter"        #listening topic

def main():
    # 1. Start Rerun
    print("Starting Python visualizer...")
    rr.init("MADS_Replication_Python", spawn=True)

    # 2. Connect to MADS (ZeroMQ)
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(MADS_ENDPOINT)
    
    # Subscribe to the filter topic
    socket.setsockopt_string(zmq.SUBSCRIBE, TOPIC_FILTER)

    trajectory_history = []

    print(f"Listening on {MADS_ENDPOINT} for topic '{TOPIC_FILTER}'")

    while True:
        try:
            # 3. Receive the message
            # MADS sends multipart messages: [Topic, JSON_Payload]
            msg = socket.recv_multipart()
            topic = msg[0].decode('utf-8')
            payload = msg[1].decode('utf-8')
            
            data = json.loads(payload)

            # 4. Extract data (with error handling)
            if "pose" not in data:
                continue

            # Extract time (sim_time that we created in the filter)
            time_val = 0.0
            if "sim_time" in data:
                time_val = data["sim_time"]
            elif "timecode" in data:
                time_val = data["timecode"]
            
            # Set the time in the timeline
            rr.set_time_seconds("sim_time", time_val)

            # 5. Visualize on Rerun
            pos = data["pose"]["position"]
            orient = data["pose"]["orientation"]

            # Draw the robot body (3D point with arrow)
            rr.log(
                "robot/body",
                rr.Transform3D(
                    translation=[pos["x"], pos["y"], 0.0],
                    rotation=rr.RotationAxisAngle(axis=[0, 0, 1], angle=orient["yaw"])
                )
            )

            rr.log("robot/body/arrow", rr.Arrows3D(vectors=[[0.2, 0, 0]], colors=[255, 255, 0]))

            if "pose_vector" in data:
                current_point = data["pose_vector"]
                trajectory_history.append(current_point)
                if len(trajectory_history) > 10000:
                    trajectory_history.pop(0)

                rr.log(
                    "robot/trajectory",
                    rr.LineStrips3D([trajectory_history], colors=[[255, 0, 0]], radii=[0.005])
                )

        except KeyboardInterrupt:
            print("Manual interruption.")
            break
        except Exception as e:
            print(f"Parsing error: {e}")

if __name__ == "__main__":
    main()