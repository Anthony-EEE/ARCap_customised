import socket
import time
from argparse import ArgumentParser
from scipy.spatial.transform import Rotation
import pybullet as pb
import joblib
import numpy as np
from ip_config import *
from quest_robot_module import QuestHumanoidModule

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--frequency", type=int, default=30)
    parser.add_argument("--key", type=str, default="0-CMU_55_55_02_poses")
    args = parser.parse_args()
    
    c = pb.connect(pb.GUI)
    quest = QuestHumanoidModule(VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT)
    data = joblib.load("amass_test.pkl")
    traj = data[args.key]
    traj_len = len(traj["dof"])
    current_ts = time.time()
    start_time = current_ts
    cnt = 0
    fps_counter = 0
    packet_counter = 0
    print("Initialization completed")
    while True:
        now = time.time()
        if now - current_ts < 1 / args.frequency: 
            continue
        else:
            current_ts = now
        
        try:
            #quest.receive()
            # if quest.is_connected:
            q = traj["dof"][cnt%traj_len]
            root_pos = traj["root_trans_offset"][cnt%traj_len]
            root_rot = traj["root_rot"][cnt%traj_len]
            # q = np.zeros(29)
            # root_pos = np.zeros(3)
            # root_rot = Rotation.from_euler("xyz", [0, 0, 0]).as_quat()
            quest.send_ik_result(q, root_pos, root_rot, dry_run=True)
            cnt += 1
        except socket.error as e:
            print(e)
            pass
        except KeyboardInterrupt:
            quest.close()
            break
        else:
            packet_time = time.time()
            fps_counter += 1
            packet_counter += 1
            if (packet_time - start_time) > 1.0:
                print(f"received {fps_counter} packets in a second", end="\r")
                start_time += 1.0
                fps_counter = 0
        
