import socket
import time
from argparse import ArgumentParser
import numpy as np
from scipy.spatial.transform import Rotation
from ip_config import *
from realsense_module import DepthCameraModule
from quest_robot_module import QuestNaviModule

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--frequency", type=int, default=30)
    parser.add_argument("--handedness", type=str, default="right")
    parser.add_argument("--no_camera", action="store_true", default=False)
    args = parser.parse_args()
    if not args.no_camera:
        camera = DepthCameraModule(is_decimate=False, visualize=False)
    
    quest = QuestNaviModule(VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT)

    start_time = time.time()
    fps_counter = 0
    packet_counter = 0
    print("Initialization completed")
    current_ts = time.time()
    while True:
        now = time.time()
        # TODO: May cause communication issues, need to tune on AR side.
        if now - current_ts < 1 / args.frequency: 
            continue
        else:
            current_ts = now
        try:
            if not args.no_camera:
                color_image, depth_image = camera.receive_image()
            head_pose= quest.receive()
            if head_pose is not None:
                head_pos = head_pose[0]
                head_orn = Rotation.from_quat(head_pose[1])
                if quest.data_dir is not None:
                    if args.no_camera:
                        color_image = np.zeros((224,224,3),dtype=np.uint8) # dummy point cloud
                        depth_image = np.zeros((224,224), dtype=np.uint16 ) # dummy depth image
                    np.savez(f"{quest.data_dir}/navi_data_{time.time()}.npz", head_pos=head_pos, head_orn=head_orn.as_quat(),
                                                                              color_image=color_image, depth_image=depth_image)
        except socket.error as e:
            print(e)
            pass
        except KeyboardInterrupt:
            if not args.no_camera:
                camera.close()
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
        

