import pybullet as pb
import pybullet_data
import time
import numpy as np
from scipy.spatial.transform import Rotation

class SimplePandaArm:
    """Simplified Panda arm with gripper for pick and place visualization"""
    
    # Panda arm rest position (7 DOF)
    ARM_REST = [0.0, -0.498, -0.02, -2.473, -0.013, 2.004, -0.723]
    GRIPPER_OPEN = [0.04, 0.04]   # Gripper open position
    GRIPPER_CLOSE = [0.0, 0.0]    # Gripper closed position
    
    def __init__(self, visualize=True):
        self.visualize = visualize
        
        # Initialize PyBullet
        if self.visualize:
            self.physics_client = pb.connect(pb.GUI)
            pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 1)
        else:
            self.physics_client = pb.connect(pb.DIRECT)
            
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0, 0, -9.81)
        
        # Load ground plane
        self.plane_id = pb.loadURDF("plane.urdf")
        
        # Load Panda arm and gripper
        self.load_robot()
        
        # Set initial positions
        self.reset_to_home()
        
    def load_robot(self):
        """Load Panda arm and gripper URDFs"""
        # Load Panda arm
        self.arm_id = pb.loadURDF(
            "assets/franka_arm/panda_gripper.urdf", 
            basePosition=[0.0, 0.0, 0.0], 
            baseOrientation=[0, 0, 0.7071068, 0.7071068], 
            useFixedBase=True
        )
        
        # Load gripper separately
        self.gripper_id = pb.loadURDF("assets/gripper/franka_panda_tri_gripper.urdf")
        
        # Get joint limits for arm
        self.arm_lower_limits, self.arm_upper_limits, self.arm_joint_ranges = self.get_joint_limits(self.arm_id)
        
        # Gripper mount offsets (matching original code)
        self.gripper_mount_pos_offset = np.array([0.0, 0.0, 0.0])
        self.gripper_mount_orn_offset = Rotation.from_euler("xyz", [0., 0., 0.])
        
        print(f"Loaded Panda arm with {pb.getNumJoints(self.arm_id)} joints")
        print(f"Loaded gripper with {pb.getNumJoints(self.gripper_id)} joints")
        
        # Print link information to understand the structure
        print("\nArm link structure:")
        for i in range(pb.getNumJoints(self.arm_id)):
            link_info = pb.getJointInfo(self.arm_id, i)
            print(f"  Link {i}: {link_info[12].decode('utf-8')} -> {link_info[1].decode('utf-8')}")
        
    def get_joint_limits(self, robot_id):
        """Get joint limits for the robot"""
        lower_limits, upper_limits, joint_ranges = [], [], []
        for i in range(pb.getNumJoints(robot_id)):
            joint_info = pb.getJointInfo(robot_id, i)
            if joint_info[2] == pb.JOINT_FIXED:
                continue
            lower_limits.append(joint_info[8])
            upper_limits.append(joint_info[9])
            joint_ranges.append(joint_info[9] - joint_info[8])
        return lower_limits, upper_limits, joint_ranges
    
    def set_joint_positions(self, robot_id, joint_positions):
        """Set joint positions for the robot"""
        joint_idx = 0
        for i in range(len(joint_positions)):
            joint_info = pb.getJointInfo(robot_id, joint_idx)
            if joint_info[2] != pb.JOINT_FIXED:
                pb.resetJointState(robot_id, joint_idx, joint_positions[i])
            else:
                joint_idx += 1
                if joint_idx < pb.getNumJoints(robot_id):
                    pb.resetJointState(robot_id, joint_idx, joint_positions[i])
            joint_idx += 1
    
    def update_gripper_position(self):
        """Update gripper position to match arm end-effector (matching original code)"""
        # Get end-effector state from link 8 (panda_hand)
        ee_state = pb.getLinkState(self.arm_id, 8)
        ee_pos = np.array(ee_state[0])
        ee_orn = Rotation.from_quat(ee_state[1])
        
        # Apply mount offsets (same as original code line 525)
        gripper_pos = ee_pos + (ee_orn * self.gripper_mount_orn_offset).apply(self.gripper_mount_pos_offset)
        gripper_orn = (ee_orn * self.gripper_mount_orn_offset).as_quat()
        
        # Position gripper at end-effector
        pb.resetBasePositionAndOrientation(self.gripper_id, gripper_pos, gripper_orn)
    
    def reset_to_home(self):
        """Reset robot to home position"""
        self.set_joint_positions(self.arm_id, self.ARM_REST)
        self.set_joint_positions(self.gripper_id, self.GRIPPER_OPEN)
        self.update_gripper_position()  # Mount gripper properly
        pb.stepSimulation()
        
    def solve_ik(self, target_position, target_orientation=None):
        """Solve inverse kinematics for target position and orientation"""
        if target_orientation is None:
            # Default downward orientation
            target_orientation = pb.getQuaternionFromEuler([np.pi, 0, 0])
        
        # End-effector is at link 8 (panda_hand) - matching original code
        joint_positions = pb.calculateInverseKinematics(
            self.arm_id, 
            8,  # End-effector link index (panda_hand)
            target_position,
            target_orientation,
            lowerLimits=self.arm_lower_limits,
            upperLimits=self.arm_upper_limits,
            jointRanges=self.arm_joint_ranges,
            restPoses=self.ARM_REST,
            maxNumIterations=100,
            residualThreshold=0.001
        )
        
        return joint_positions[:7]  # Only return arm joints
    
    def move_to_position(self, target_position, target_orientation=None):
        """Move arm to target position"""
        joint_positions = self.solve_ik(target_position, target_orientation)
        self.set_joint_positions(self.arm_id, joint_positions)
        
        # Update gripper position to follow arm end-effector
        self.update_gripper_position()
        
        pb.stepSimulation()
        return joint_positions
    
    def open_gripper(self):
        """Open the gripper"""
        self.set_joint_positions(self.gripper_id, self.GRIPPER_OPEN)
        self.update_gripper_position()  # Ensure gripper stays mounted
        pb.stepSimulation()
        
    def close_gripper(self):
        """Close the gripper"""
        self.set_joint_positions(self.gripper_id, self.GRIPPER_CLOSE)
        self.update_gripper_position()  # Ensure gripper stays mounted
        pb.stepSimulation()
    
    def pick_and_place_demo(self):
        """Demonstrate a simple pick and place operation"""
        print("Starting pick and place demonstration...")
        
        # Add a simple object to pick up
        box_id = pb.loadURDF("cube_small.urdf", [0.5, 0.0, 0.05])
        pb.changeVisualShape(box_id, -1, rgbaColor=[1, 0, 0, 1])  # Red box
        
        # Home position
        print("1. Moving to home position...")
        self.reset_to_home()
        time.sleep(1)
        
        # Move above object
        print("2. Moving above object...")
        above_object = [0.5, 0.0, 0.3]
        self.move_to_position(above_object)
        time.sleep(1)
        
        # Move down to object
        print("3. Moving down to object...")
        at_object = [0.5, 0.0, 0.1]
        self.move_to_position(at_object)
        time.sleep(1)
        
        # Close gripper to pick up
        print("4. Closing gripper...")
        self.close_gripper()
        time.sleep(1)
        
        # Lift object
        print("5. Lifting object...")
        self.move_to_position(above_object)
        time.sleep(1)
        
        # Move to place location
        print("6. Moving to place location...")
        place_above = [-0.3, 0.3, 0.3]
        self.move_to_position(place_above)
        time.sleep(1)
        
        # Place object
        print("7. Placing object...")
        place_location = [-0.3, 0.3, 0.1]
        self.move_to_position(place_location)
        time.sleep(1)
        
        # Open gripper to release
        print("8. Opening gripper...")
        self.open_gripper()
        time.sleep(1)
        
        # Move away
        print("9. Moving away...")
        self.move_to_position(place_above)
        time.sleep(1)
        
        # Return home
        print("10. Returning home...")
        self.reset_to_home()
        
        print("Pick and place demonstration complete!")
    
    def interactive_control(self):
        """Interactive control for manual testing"""
        print("\n=== Interactive Panda Control ===")
        print("Commands:")
        print("  'home' - Return to home position")
        print("  'open' - Open gripper")
        print("  'close' - Close gripper") 
        print("  'demo' - Run pick and place demo")
        print("  'move x y z' - Move to position (e.g., 'move 0.5 0.0 0.3')")
        print("  'quit' - Exit")
        
        while True:
            try:
                cmd = input("\nEnter command: ").strip().lower()
                
                if cmd == 'quit':
                    break
                elif cmd == 'home':
                    self.reset_to_home()
                    print("Moved to home position")
                elif cmd == 'open':
                    self.open_gripper()
                    print("Gripper opened")
                elif cmd == 'close':
                    self.close_gripper()
                    print("Gripper closed")
                elif cmd == 'demo':
                    self.pick_and_place_demo()
                elif cmd.startswith('move'):
                    parts = cmd.split()
                    if len(parts) == 4:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        self.move_to_position([x, y, z])
                        print(f"Moved to position [{x}, {y}, {z}]")
                    else:
                        print("Invalid move command. Use: move x y z")
                else:
                    print("Unknown command")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
    
    def close(self):
        """Close PyBullet connection"""
        pb.disconnect()

# Create and test the simplified Panda arm
if __name__ == "__main__":
    # Initialize simplified Panda arm
    panda = SimplePandaArm(visualize=True)
    
    # Run interactive control
    try:
        panda.interactive_control()
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        panda.close()



