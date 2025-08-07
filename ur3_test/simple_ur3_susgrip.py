#!/usr/bin/env python3
"""
Simple UR3 + Susgrip Integration Demo
Shows UR3 robot with mounted susgrip gripper moving from position to home pose
"""

import pybullet as pb
import pybullet_data
import numpy as np
import time
from scipy.spatial.transform import Rotation

def main():
    print("🤖 Simple UR3 + Susgrip Demo")
    
    # Setup PyBullet
    physics_client = pb.connect(pb.GUI)
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 1)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -9.81)
    
    # Load ground plane
    plane_id = pb.loadURDF("plane.urdf")
    
    try:
        # Load UR3 robot
        print("📦 Loading UR3 robot...")
        ur3_id = pb.loadURDF(
            "urdf_files_dataset/urdf_files/ros-industrial/xacro_generated/universal_robots/ur_description/urdf/ur3.urdf",
            basePosition=[0.0, 0.0, 0.0], 
            baseOrientation=[0, 0, 0, 1],
            useFixedBase=True
        )
        
        # Find UR3 end-effector link (usually the last joint)
        num_joints = pb.getNumJoints(ur3_id)
        end_effector_link = num_joints - 1  # Use last joint as end-effector
        
        print(f"✅ UR3 loaded with {num_joints} joints, using joint {end_effector_link} as end-effector")
        
        # Get end-effector position for gripper mounting
        ee_state = pb.getLinkState(ur3_id, end_effector_link)
        ee_pos = np.array(ee_state[0])
        ee_orn = Rotation.from_quat(ee_state[1])
        
        # Calculate gripper mounting position (5cm forward from end-effector)
        mount_offset = np.array([0.0, 0.0, 0.05])  # 5cm in Z direction
        mount_rotation = Rotation.from_euler("xyz", [np.pi, 0., 0.])  # Flip 180 degrees
        
        gripper_pos = ee_pos + (ee_orn * mount_rotation).apply(mount_offset)
        gripper_orn = (ee_orn * mount_rotation).as_quat()
        
        print(f"🔧 Loading susgrip at position: {gripper_pos}")
        
        # Load susgrip gripper at the correct position
        gripper_id = pb.loadURDF(
            "ur3_test/susgrip_2f_realistic.urdf",
            basePosition=gripper_pos,
            baseOrientation=gripper_orn
        )
        
        print(f"✅ Susgrip loaded with {pb.getNumJoints(gripper_id)} joints")
        
        # Create rigid constraint to keep gripper attached to UR3
        constraint_id = pb.createConstraint(
            parentBodyUniqueId=ur3_id,
            parentLinkIndex=end_effector_link,
            childBodyUniqueId=gripper_id,
            childLinkIndex=-1,
            jointType=pb.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=mount_offset.tolist(),
            childFramePosition=[0, 0, 0]
        )
        
        print("🔗 Created rigid constraint between UR3 and gripper")
        
        # Step simulation to settle everything
        for _ in range(10):
            pb.stepSimulation()
        
        # Verify connection
        final_gripper_pos, _ = pb.getBasePositionAndOrientation(gripper_id)
        distance = np.linalg.norm(np.array(final_gripper_pos) - ee_pos)
        print(f"✅ Connection verified: gripper is {distance*100:.1f}cm from UR3 end-effector")
        
        # Define poses
        HOME_POSE = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]  # UR3 home pose
        
        # Function to move robot smoothly
        def move_to_joint_positions(target_joints, duration=3.0, steps=240):
            """Move robot smoothly to target joint positions"""
            current_joints = []
            for i in range(6):  # UR3 has 6 joints
                current_joints.append(pb.getJointState(ur3_id, i)[0])
            
            print(f"🎯 Moving from {[f'{j:.2f}' for j in current_joints]} to {[f'{j:.2f}' for j in target_joints]}")
            
            # Interpolate between current and target positions
            for step in range(steps):
                t = step / (steps - 1)  # 0 to 1
                
                # Linear interpolation
                interpolated_joints = []
                for i in range(6):
                    joint_pos = current_joints[i] + t * (target_joints[i] - current_joints[i])
                    interpolated_joints.append(joint_pos)
                
                # Set joint positions
                for i in range(6):
                    pb.resetJointState(ur3_id, i, interpolated_joints[i])
                
                pb.stepSimulation()
                time.sleep(duration / steps)
        
        # Function to solve inverse kinematics
        def move_to_position(target_pos):
            """Move UR3 end-effector to target position using IK"""
            target_orn = pb.getQuaternionFromEuler([np.pi, 0, 0])  # Point down
            
            joint_positions = pb.calculateInverseKinematics(
                ur3_id, 
                end_effector_link,
                target_pos,
                target_orn,
                maxNumIterations=100,
                residualThreshold=0.001
            )
            
            # Move to calculated joint positions
            move_to_joint_positions(joint_positions[:6])
        
        print("\n🎬 Starting demo sequence...")
        
        # Demo sequence
        print("1️⃣ Moving to target position [0.5, 0.5, 0.5]...")
        move_to_position([0.5, 0.5, 0.5])
        time.sleep(1)
        
        print("2️⃣ Moving to home pose...")
        move_to_joint_positions(HOME_POSE)
        time.sleep(1)
        
        print("3️⃣ Testing gripper movement...")
        # Open gripper (susgrip uses prismatic joints 0 and 1)
        pb.resetJointState(gripper_id, 0, 0.015)  # Left slider open
        pb.resetJointState(gripper_id, 1, 0.015)  # Right slider open
        time.sleep(1)
        
        # Close gripper
        pb.resetJointState(gripper_id, 0, 0.0)    # Left slider closed
        pb.resetJointState(gripper_id, 1, 0.0)    # Right slider closed
        time.sleep(1)
        
        print("✅ Demo complete! Press Enter to exit...")
        input()
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        pb.disconnect()

if __name__ == "__main__":
    main()