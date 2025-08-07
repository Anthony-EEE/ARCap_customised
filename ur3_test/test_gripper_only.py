#!/usr/bin/env python3
"""Simple test to verify susgrip_2f.urdf loads correctly"""

import pybullet as pb
import pybullet_data
import time

def test_gripper_only():
    print("🔧 Testing susgrip_2f gripper URDF...")
    
    # Setup PyBullet
    physics_client = pb.connect(pb.GUI)
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 1)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -9.81)
    
    # Load ground plane
    plane_id = pb.loadURDF("plane.urdf")
    
    try:
        # Load realistic gripper URDF with 3D meshes
        print("📁 Loading ur3_test/susgrip_2f_realistic.urdf...")
        gripper_id = pb.loadURDF("ur3_test/susgrip_2f_realistic.urdf", 
                                basePosition=[0, 0, 0.1],  # Position it above ground
                                baseOrientation=[0, 0, 0, 1])
        
        print(f"✅ Gripper loaded successfully! ID: {gripper_id}")
        print(f"📊 Number of joints: {pb.getNumJoints(gripper_id)}")
        
        # Print joint information
        print("\n🔍 Joint details:")
        for i in range(pb.getNumJoints(gripper_id)):
            joint_info = pb.getJointInfo(gripper_id, i)
            joint_name = joint_info[1].decode('utf-8')
            link_name = joint_info[12].decode('utf-8')
            joint_type = joint_info[2]
            print(f"  Joint {i}: {joint_name} -> {link_name} (type: {joint_type})")
        
        # Get initial position
        pos, orn = pb.getBasePositionAndOrientation(gripper_id)
        print(f"\n📍 Gripper position: {pos}")
        print(f"📍 Gripper orientation: {orn}")
        
        # Test gripper movement
        print("\n🎮 Testing gripper movement...")
        
        # Open gripper (use susgrip slider limits: 0.0 to 0.015)
        print("  Opening gripper...")
        pb.resetJointState(gripper_id, 0, 0.015)  # left slider - maximum extension
        pb.resetJointState(gripper_id, 1, 0.015)  # right slider - maximum extension
        
        # Wait and step simulation
        for i in range(120):  # 0.5 seconds at 240 Hz
            pb.stepSimulation()
            time.sleep(1/240)
        
        # Close gripper
        print("  Closing gripper...")
        pb.resetJointState(gripper_id, 0, 0.0)   # left slider - minimum position
        pb.resetJointState(gripper_id, 1, 0.0)   # right slider - minimum position
        
        # Wait and step simulation
        for i in range(120):  # 0.5 seconds at 240 Hz
            pb.stepSimulation()
            time.sleep(1/240)
        
        print("\n✅ Gripper test complete!")
        print("🎯 You should see a gripper floating above the ground plane")
        print("🎯 Base should be visible, with two fingers that opened and closed")
        print("\nPress Enter to exit...")
        input()
        
    except Exception as e:
        print(f"❌ Error loading gripper: {e}")
        print("💡 Check that ur3_test/susgrip_2f_realistic.urdf exists and is valid")
        
    finally:
        pb.disconnect()

if __name__ == "__main__":
    test_gripper_only()