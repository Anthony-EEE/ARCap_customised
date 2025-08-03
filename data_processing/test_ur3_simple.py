import pybullet as pb
import pybullet_data
import time
import numpy as np
from scipy.spatial.transform import Rotation

class SimpleUR3ArmWithPrimitiveGripper:
    """UR3 arm with simple primitive gripper for testing"""
    
    # UR3 home position (6 DOF)
    UR3_ARM_REST = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
    GRIPPER_OPEN_WIDTH = 0.08   # 8cm open
    GRIPPER_CLOSE_WIDTH = 0.02  # 2cm closed
    
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
        
        # Load robot and create gripper
        self.load_robot()
        self.create_primitive_gripper()
        
        # Set initial positions
        self.reset_to_home()
        
    def load_robot(self):
        """Load UR3 arm - you'll need to get UR3 URDF"""
        print("Loading UR3 arm...")
        # For now, let's use Panda but with UR3 joint limits for testing
        self.arm_id = pb.loadURDF(
            "assets/franka_arm/panda_gripper.urdf",  # Replace with UR3 when available
            basePosition=[0.0, 0.0, 0.0], 
            baseOrientation=[0, 0, 0, 1], 
            useFixedBase=True
        )
        
        # Get joint limits
        self.arm_lower_limits, self.arm_upper_limits, self.arm_joint_ranges = self.get_joint_limits(self.arm_id)
        
        print(f"Loaded arm with {pb.getNumJoints(self.arm_id)} joints")
        
    def create_primitive_gripper(self):
        """Create a simple two-finger gripper using primitives"""
        
        # Create gripper base (palm)
        palm_collision = pb.createCollisionShape(
            pb.GEOM_BOX, 
            halfExtents=[0.02, 0.05, 0.01]
        )
        palm_visual = pb.createVisualShape(
            pb.GEOM_BOX, 
            halfExtents=[0.02, 0.05, 0.01],
            rgbaColor=[0.2, 0.2, 0.8, 1.0]
        )
        
        # Create left finger
        finger_collision = pb.createCollisionShape(
            pb.GEOM_BOX,
            halfExtents=[0.01, 0.02, 0.03]
        )
        finger_visual = pb.createVisualShape(
            pb.GEOM_BOX,
            halfExtents=[0.01, 0.02, 0.03],
            rgbaColor=[0.8, 0.2, 0.2, 1.0]
        )
        
        # Create gripper body with two fingers
        self.gripper_id = pb.createMultiBody(
            baseMass=0.1,
            baseCollisionShapeIndex=palm_collision,
            baseVisualShapeIndex=palm_visual,
            basePosition=[0, 0, 0],
            linkMasses=[0.05, 0.05],
            linkCollisionShapeIndices=[finger_collision, finger_collision],
            linkVisualShapeIndices=[finger_visual, finger_visual],
            linkPositions=[[0, 0.04, 0], [0, -0.04, 0]],  # Finger positions
            linkOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],
            linkInertialFramePositions=[[0, 0, 0], [0, 0, 0]],
            linkInertialFrameOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],
            linkParentIndices=[0, 0],  # Both fingers attach to base
            linkJointTypes=[pb.JOINT_PRISMATIC, pb.JOINT_PRISMATIC],
            linkJointAxis=[[0, 1, 0], [0, -1, 0]]  # Move fingers in/out
        )
        
        # Set gripper joint limits
        for i in range(2):
            pb.changeDynamics(self.gripper_id, i, lateralFriction=1.0)
            
        self.gripper_finger_indices = [0, 1]  # Joint indices for fingers
        
        print("Created primitive gripper with 2 fingers")
    
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
        """Update gripper position to match arm end-effector"""
        # Get end-effector state (using link 8 for now, adjust for UR3)
        ee_state = pb.getLinkState(self.arm_id, 8)
        ee_pos = np.array(ee_state[0])
        ee_orn = ee_state[1]
        
        # Mount gripper at end-effector
        pb.resetBasePositionAndOrientation(self.gripper_id, ee_pos, ee_orn)
    
    def reset_to_home(self):
        """Reset robot to home position"""
        # Use Panda home for now (replace with UR3 home when available)
        panda_home = [0.0, -0.498, -0.02, -2.473, -0.013, 2.004, -0.723]
        self.set_joint_positions(self.arm_id, panda_home)
        self.open_gripper()
        self.update_gripper_position()
        pb.stepSimulation()
    
    def solve_ik(self, target_position, target_orientation=None):
        """Solve inverse kinematics for target position"""
        if target_orientation is None:
            target_orientation = pb.getQuaternionFromEuler([np.pi, 0, 0])
        
        joint_positions = pb.calculateInverseKinematics(
            self.arm_id, 
            8,  # End-effector link
            target_position,
            target_orientation,
            lowerLimits=self.arm_lower_limits,
            upperLimits=self.arm_upper_limits,
            jointRanges=self.arm_joint_ranges,
            restPoses=[0.0, -0.498, -0.02, -2.473, -0.013, 2.004, -0.723],
            maxNumIterations=100,
            residualThreshold=0.001
        )
        
        return joint_positions[:7]  # Return arm joints only
    
    def move_to_position(self, target_position, target_orientation=None):
        """Move arm to target position"""
        joint_positions = self.solve_ik(target_position, target_orientation)
        self.set_joint_positions(self.arm_id, joint_positions)
        self.update_gripper_position()
        pb.stepSimulation()
        return joint_positions
    
    def open_gripper(self):
        """Open the primitive gripper"""
        # Move fingers outward
        pb.resetJointState(self.gripper_id, 0, self.GRIPPER_OPEN_WIDTH/2)
        pb.resetJointState(self.gripper_id, 1, self.GRIPPER_OPEN_WIDTH/2)
        pb.stepSimulation()
        
    def close_gripper(self):
        """Close the primitive gripper"""
        # Move fingers inward
        pb.resetJointState(self.gripper_id, 0, self.GRIPPER_CLOSE_WIDTH/2)
        pb.resetJointState(self.gripper_id, 1, self.GRIPPER_CLOSE_WIDTH/2)
        pb.stepSimulation()
    
    def pick_and_place_demo(self):
        """Demonstrate pick and place with primitive gripper"""
        print("Starting UR3 pick and place demo with primitive gripper...")
        
        # Add object to pick
        box_id = pb.loadURDF("cube_small.urdf", [0.5, 0.0, 0.05])
        pb.changeVisualShape(box_id, -1, rgbaColor=[1, 0, 0, 1])
        
        # Demo sequence
        print("1. Home position...")
        self.reset_to_home()
        time.sleep(1)
        
        print("2. Move above object...")
        self.move_to_position([0.5, 0.0, 0.3])
        time.sleep(1)
        
        print("3. Move down to object...")
        self.move_to_position([0.5, 0.0, 0.1])
        time.sleep(1)
        
        print("4. Close gripper...")
        self.close_gripper()
        time.sleep(1)
        
        print("5. Lift object...")
        self.move_to_position([0.5, 0.0, 0.3])
        time.sleep(1)
        
        print("6. Move to place location...")
        self.move_to_position([-0.3, 0.3, 0.3])
        time.sleep(1)
        
        print("7. Place object...")
        self.move_to_position([-0.3, 0.3, 0.1])
        time.sleep(1)
        
        print("8. Open gripper...")
        self.open_gripper()
        time.sleep(1)
        
        print("9. Return home...")
        self.reset_to_home()
        print("Demo complete!")
    
    def interactive_control(self):
        """Interactive control interface"""
        print("\n=== UR3 with Primitive Gripper Control ===")
        print("Commands:")
        print("  'home' - Return to home position")
        print("  'open' - Open gripper")
        print("  'close' - Close gripper") 
        print("  'demo' - Run pick and place demo")
        print("  'move x y z' - Move to position")
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

# Test the UR3 with primitive gripper
if __name__ == "__main__":
    ur3 = SimpleUR3ArmWithPrimitiveGripper(visualize=True)
    
    try:
        ur3.interactive_control()
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ur3.close()