import pybullet as pb
import pybullet_data
import numpy as np
import time
from scipy.spatial.transform import Rotation

class RobotTester:
    """Universal robot testing platform - supports Panda now, UR3 later"""
    
    def __init__(self, robot_type="panda"):
        self.robot_type = robot_type
        self.setup_environment()
        self.load_robot()
        self.reset_to_home()
        
    def setup_environment(self):
        """Setup PyBullet environment"""
        self.physics_client = pb.connect(pb.GUI)
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 1)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0, 0, -9.81)
        
        # Load ground plane
        self.plane_id = pb.loadURDF("plane.urdf")
        
    def load_robot(self):
        """Load robot based on type"""
        if self.robot_type == "panda":
            self.load_panda()
        elif self.robot_type == "ur3":
            self.load_ur3()
        else:
            raise ValueError(f"Unknown robot type: {self.robot_type}")
            
    def load_panda(self):
        """Load Panda arm with proper gripper mounting"""
        print("Loading Franka Panda arm...")
        
        # Load Panda arm
        self.arm_id = pb.loadURDF(
            "data_processing/assets/franka_arm/panda_gripper.urdf", 
            basePosition=[0.0, 0.0, 0.0], 
            baseOrientation=[0, 0, 0.7071068, 0.7071068], 
            useFixedBase=True
        )
        
        # Load gripper
        self.gripper_id = pb.loadURDF("data_processing/assets/gripper/franka_panda_tri_gripper.urdf")
        
        # Panda-specific parameters
        self.ARM_REST = [0.0, -0.498, -0.02, -2.473, -0.013, 2.004, -0.723]
        self.GRIPPER_OPEN = [0.04, 0.04]
        self.GRIPPER_CLOSE = [0.0, 0.0]
        self.end_effector_link = 8  # panda_hand link
        
        # Gripper mount offsets
        self.gripper_mount_pos_offset = np.array([0.0, 0.0, 0.0])
        self.gripper_mount_orn_offset = Rotation.from_euler("xyz", [0., 0., 0.])
        
        # Get joint limits
        self.joint_lower_limits, self.joint_upper_limits, self.joint_ranges = self.get_joint_limits(self.arm_id)
        
        print(f"Loaded Panda: {pb.getNumJoints(self.arm_id)} arm joints, {pb.getNumJoints(self.gripper_id)} gripper joints")
        self.print_joint_info()
        
    def load_ur3(self):
        """Load UR3 arm using real UR3 URDF from urdf_files_dataset"""
        print("Loading Universal Robot UR3 arm...")
        
        try:
            # Load real UR3 URDF from urdf_files_dataset
            self.arm_id = pb.loadURDF(
                "urdf_files_dataset/urdf_files/ros-industrial/xacro_generated/universal_robots/ur_description/urdf/ur3.urdf",
                basePosition=[0.0, 0.0, 0.0], 
                baseOrientation=[0, 0, 0, 1],
                useFixedBase=True
            )
            
            # First, get the UR3 end-effector position for gripper mounting
            ee_state = pb.getLinkState(self.arm_id, self.end_effector_link)
            ee_pos = np.array(ee_state[0])
            ee_orn = Rotation.from_quat(ee_state[1])
            
            # Calculate gripper mounting position
            gripper_pos = ee_pos + (ee_orn * self.gripper_mount_orn_offset).apply(self.gripper_mount_pos_offset)
            gripper_orn = (ee_orn * self.gripper_mount_orn_offset).as_quat()
            
            print(f"🔍 UR3 end-effector at: {ee_pos}")
            print(f"🔧 Loading gripper at calculated position: {gripper_pos}")
            
            # Load gripper (using realistic susgrip_2f gripper with 3D meshes) AT THE CORRECT POSITION
            print("📦 Loading realistic susgrip_2f gripper with detailed 3D meshes...")
            self.gripper_id = pb.loadURDF(
                "ur3_test/susgrip_2f_realistic.urdf",
                basePosition=gripper_pos,
                baseOrientation=gripper_orn
            )
            
            # Make gripper more visible by changing colors to bright/distinct ones
            try:
                pb.changeVisualShape(self.gripper_id, -1, rgbaColor=[1, 1, 0, 1])  # Yellow base
                for i in range(pb.getNumJoints(self.gripper_id)):
                    if i == 0:  # left finger
                        pb.changeVisualShape(self.gripper_id, i, rgbaColor=[0, 1, 0, 1])  # Bright green
                    elif i == 1:  # right finger  
                        pb.changeVisualShape(self.gripper_id, i, rgbaColor=[1, 0, 1, 1])  # Bright magenta
            except Exception as e:
                print(f"⚠️ Could not change gripper colors: {e}")
            
            # Disable gravity for gripper to prevent flying away
            pb.changeDynamics(self.gripper_id, -1, mass=0)
            
            # Disable collisions between gripper and arm
            for i in range(pb.getNumJoints(self.arm_id)):
                pb.setCollisionFilterPair(self.arm_id, self.gripper_id, i, -1, 0)
                for j in range(pb.getNumJoints(self.gripper_id)):
                    pb.setCollisionFilterPair(self.arm_id, self.gripper_id, i, j, 0)
            
            # UR3-specific parameters
            self.ARM_REST = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
            
            # Susgrip_2f gripper parameters (prismatic slider joints)
            # Based on URDF joint limits: lower="0.0" upper="0.015"
            self.GRIPPER_OPEN = [0.015, 0.015]   # Maximum slider extension for open position
            self.GRIPPER_CLOSE = [0.0, 0.0]     # Minimum slider position for closed position
            
            # Map gripper joints: 0=left slider, 1=right slider (prismatic)
            # Additional joints (2-9) are continuous rotation joints for finger segments
            
            # Find end-effector link for UR3
            num_joints = pb.getNumJoints(self.arm_id)
            self.end_effector_link = None
            
            print(f"\n🔍 Searching for UR3 end-effector among {num_joints} joints:")
            for i in range(num_joints):
                joint_info = pb.getJointInfo(self.arm_id, i)
                link_name = joint_info[12].decode('utf-8')
                parent_name = joint_info[1].decode('utf-8')
                print(f"  Joint {i}: {parent_name} -> {link_name}")
                
                # Priority: tool0 > flange > wrist_3_link
                if 'tool0' in link_name.lower():
                    self.end_effector_link = i
                    print(f"✅ Found tool0 link at index {i}: {link_name}")
                    break
                elif 'flange' in link_name.lower():
                    self.end_effector_link = i
                    print(f"✅ Found flange link at index {i}: {link_name}")
                elif 'wrist_3_link' in link_name.lower():
                    self.end_effector_link = i
                    print(f"✅ Found wrist_3_link at index {i}: {link_name}")
            
            # If not found, use a reasonable default
            if self.end_effector_link is None:
                self.end_effector_link = max(0, num_joints - 2)
                print(f"⚠️ Using fallback link as end-effector: index {self.end_effector_link}")
            
            # UR3-specific gripper mount offsets (create proper flange connection)
            # Mount gripper slightly forward from flange center to create realistic connection
            self.gripper_mount_pos_offset = np.array([0.0, 0.0, 0.05])  # 5cm forward from flange
            # Rotate gripper to align properly with UR3 end-effector (susgrip facing forward)
            self.gripper_mount_orn_offset = Rotation.from_euler("xyz", [np.pi, 0., 0.])
            
            # Get joint limits
            self.joint_lower_limits, self.joint_upper_limits, self.joint_ranges = self.get_joint_limits(self.arm_id)
            
            print(f"✅ Loaded UR3: {pb.getNumJoints(self.arm_id)} total joints, End-effector link: {self.end_effector_link}")
            print(f"✅ Gripper: {pb.getNumJoints(self.gripper_id)} joints")
            
            # Debug: Print gripper initial position
            gripper_pos, gripper_orn = pb.getBasePositionAndOrientation(self.gripper_id)
            print(f"🔍 Gripper initial position: {gripper_pos}")
            print(f"🔍 Gripper initial orientation: {gripper_orn}")
            
            # Debug: Print gripper joint information
            print(f"🔍 Gripper joint details:")
            for i in range(pb.getNumJoints(self.gripper_id)):
                joint_info = pb.getJointInfo(self.gripper_id, i)
                print(f"  Joint {i}: {joint_info[1].decode('utf-8')} -> {joint_info[12].decode('utf-8')}")
            
            # IMPORTANT: Create constraint to keep gripper attached
            print("🔧 Creating rigid constraint between UR3 and gripper...")
            
            print(f"🔧 Gripper already positioned at: {gripper_pos}")
            print(f"🔧 Gripper orientation: {gripper_orn}")
            
            # Create simple direct constraint between UR3 and gripper
            self.gripper_constraint = pb.createConstraint(
                parentBodyUniqueId=self.arm_id,
                parentLinkIndex=self.end_effector_link,
                childBodyUniqueId=self.gripper_id,
                childLinkIndex=-1,
                jointType=pb.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=self.gripper_mount_pos_offset.tolist(),
                childFramePosition=[0, 0, 0]
            )
            
            # Create visual flange at the connection point
            flange_visual = pb.createVisualShape(
                shapeType=pb.GEOM_CYLINDER,
                radius=0.015,  # 1.5cm radius flange
                length=0.02,   # 2cm thick
                rgbaColor=[0.8, 0.8, 0.8, 1.0]  # Light gray
            )
            
            # Position flange at the connection point
            flange_pos = ee_pos + (ee_orn).apply(self.gripper_mount_pos_offset * 0.5)  # Halfway point
            
            self.flange_id = pb.createMultiBody(
                baseMass=0.001,  # Very light
                baseVisualShapeIndex=flange_visual,
                basePosition=flange_pos,
                baseOrientation=ee_state[1]
            )
            
            # Add visual connection line from UR3 to gripper
            pb.addUserDebugLine(
                ee_pos.tolist(), 
                gripper_pos.tolist(), 
                lineColorRGB=[1, 0, 0], 
                lineWidth=5, 
                lifeTime=0
            )
            
            # Step simulation to ensure constraint takes effect
            for _ in range(10):
                pb.stepSimulation()
            
            print("✅ Gripper mounted with flange adapter and constrained successfully!")
            
            # Debug: Print final positions after mounting
            final_gripper_pos, final_gripper_orn = pb.getBasePositionAndOrientation(self.gripper_id)
            flange_pos, flange_orn = pb.getBasePositionAndOrientation(self.flange_id)
            
            print(f"🔍 UR3 end-effector position: {ee_state[0]}")
            print(f"🔍 Flange adapter position: {flange_pos}")
            print(f"🔍 Gripper final position: {final_gripper_pos}")
            print(f"🔍 Mount chain: UR3 → Gripper (with flange visual)")
            
            # Verify they are close to each other
            distance = np.linalg.norm(np.array(final_gripper_pos) - ee_pos)
            print(f"🔍 Distance between UR3 end-effector and gripper: {distance:.4f}m")
            if distance > 0.1:  # 10cm
                print(f"⚠️ WARNING: Gripper is {distance:.2f}m away from UR3! Connection may have failed.")
            else:
                print(f"✅ Connection successful: gripper is {distance*100:.1f}cm from UR3 end-effector")
            
            self.print_joint_info()
            
        except Exception as e:
            print(f"❌ Failed to load UR3: {e}")
            print("📋 Falling back to Panda for testing...")
            self.robot_type = "panda"
            self.load_panda()
        
    def get_joint_limits(self, robot_id):
        """Get joint limits for robot"""
        joint_lower_limits = []
        joint_upper_limits = []
        joint_ranges = []
        for i in range(pb.getNumJoints(robot_id)):
            joint_info = pb.getJointInfo(robot_id, i)
            if joint_info[2] == pb.JOINT_FIXED:
                continue
            joint_lower_limits.append(joint_info[8])
            joint_upper_limits.append(joint_info[9])
            joint_ranges.append(joint_info[9] - joint_info[8])
        return joint_lower_limits, joint_upper_limits, joint_ranges
        
    def set_joint_positions(self, robot_id, joint_positions):
        """Set joint positions for robot"""
        jid = 0
        for i in range(len(joint_positions)):
            if pb.getJointInfo(robot_id, jid)[2] != pb.JOINT_FIXED:
                pb.resetJointState(robot_id, jid, joint_positions[i])
            else:
                jid += 1
                if jid < pb.getNumJoints(robot_id):
                    pb.resetJointState(robot_id, jid, joint_positions[i])
            jid += 1


        
    def reset_to_home(self):
        """Reset robot to home position"""
        self.set_joint_positions(self.arm_id, self.ARM_REST)
        self.set_joint_positions(self.gripper_id, self.GRIPPER_OPEN)
        pb.stepSimulation()
        
    def solve_ik(self, target_position, target_orientation=None):
        """Solve inverse kinematics"""
        if target_orientation is None:
            target_orientation = pb.getQuaternionFromEuler([np.pi, 0, 0])
        
        joint_positions = pb.calculateInverseKinematics(
            self.arm_id, 
            self.end_effector_link,
            target_position,
            target_orientation,
            lowerLimits=self.joint_lower_limits,
            upperLimits=self.joint_upper_limits,
            jointRanges=self.joint_ranges,
            restPoses=self.ARM_REST,
            maxNumIterations=100,
            residualThreshold=0.001
        )
        
        return joint_positions[:len(self.ARM_REST)]
        
    def move_to_position(self, target_position, target_orientation=None):
        """Move arm to target position"""
        joint_positions = self.solve_ik(target_position, target_orientation)
        self.set_joint_positions(self.arm_id, joint_positions)
        pb.stepSimulation()
        return joint_positions
        
    def open_gripper(self):
        """Open gripper"""
        self.set_joint_positions(self.gripper_id, self.GRIPPER_OPEN)
        pb.stepSimulation()
        
    def close_gripper(self):
        """Close gripper"""
        self.set_joint_positions(self.gripper_id, self.GRIPPER_CLOSE)
        pb.stepSimulation()
        
    def print_joint_info(self):
        """Print robot joint information"""
        print(f"\n=== {self.robot_type.upper()} Joint Information ===")
        print("Joint limits:", self.joint_lower_limits)
        print("Joint ranges:", self.joint_ranges)
        print("\nArm joint structure:")
        for i in range(pb.getNumJoints(self.arm_id)):
            joint_info = pb.getJointInfo(self.arm_id, i)
            print(f"  Joint {i}: {joint_info[1].decode('utf-8')} ({joint_info[12].decode('utf-8')})")
            
    def pick_and_place_demo(self):
        """Demonstrate pick and place operation"""
        print(f"\nStarting {self.robot_type.upper()} pick and place demo...")
        
        # Add object to pick
        box_id = pb.loadURDF("cube_small.urdf", [0.5, 0.0, 0.05])
        pb.changeVisualShape(box_id, -1, rgbaColor=[1, 0, 0, 1])
        
        # Demo sequence
        steps = [
            ("Home position", lambda: self.reset_to_home()),
            ("Move above object", lambda: self.move_to_position([0.5, 0.0, 0.3])),
            ("Move down to object", lambda: self.move_to_position([0.5, 0.0, 0.1])),
            ("Close gripper", lambda: self.close_gripper()),
            ("Lift object", lambda: self.move_to_position([0.5, 0.0, 0.3])),
            ("Move to place location", lambda: self.move_to_position([-0.3, 0.3, 0.3])),
            ("Place object", lambda: self.move_to_position([-0.3, 0.3, 0.1])),
            ("Open gripper", lambda: self.open_gripper()),
            ("Move away", lambda: self.move_to_position([-0.3, 0.3, 0.3])),
            ("Return home", lambda: self.reset_to_home())
        ]
        
        for i, (description, action) in enumerate(steps, 1):
            print(f"{i}. {description}...")
            action()
            time.sleep(1.5)
            
        print("Pick and place demo complete!")
        
    def interactive_control(self):
        """Interactive control interface"""
        print(f"\n=== {self.robot_type.upper()} Interactive Control ===")
        print("Commands:")
        print("  'home' - Return to home position")
        print("  'open' - Open gripper")
        print("  'close' - Close gripper") 
        print("  'demo' - Run pick and place demo")
        print("  'info' - Show robot information")
        print("  'move x y z' - Move to position (e.g., 'move 0.5 0.0 0.3')")
        print("  'link x' - Debug: Change end-effector link (e.g., 'link 5')")
        print("  'quit' - Exit")
        
        while True:
            try:
                cmd = input(f"\n[{self.robot_type.upper()}] Enter command: ").strip().lower()
                
                if cmd == 'quit':
                    break
                elif cmd == 'home':
                    self.reset_to_home()
                    print("✅ Moved to home position")
                elif cmd == 'open':
                    self.open_gripper()
                    print("✅ Gripper opened")
                elif cmd == 'close':
                    self.close_gripper()
                    print("✅ Gripper closed")
                elif cmd == 'demo':
                    self.pick_and_place_demo()
                elif cmd == 'info':
                    self.print_joint_info()
                elif cmd.startswith('move'):
                    parts = cmd.split()
                    if len(parts) == 4:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        self.move_to_position([x, y, z])
                        print(f"✅ Moved to position [{x:.2f}, {y:.2f}, {z:.2f}]")
                    else:
                        print("❌ Invalid move command. Use: move x y z")
                elif cmd.startswith('link'):
                    parts = cmd.split()
                    if len(parts) == 2:
                        try:
                            link_idx = int(parts[1])
                            if 0 <= link_idx < pb.getNumJoints(self.arm_id):
                                self.end_effector_link = link_idx
                                print(f"🔧 Changed end-effector to link {link_idx}")
                                self.update_gripper_position()
                                print(f"✅ Gripper remounted to link {link_idx}")
                            else:
                                print(f"❌ Link index {link_idx} out of range (0-{pb.getNumJoints(self.arm_id)-1})")
                        except ValueError:
                            print("❌ Invalid link index. Use: link <number>")
                    else:
                        print("❌ Usage: link <index> (e.g., 'link 5')")
                else:
                    print("❌ Unknown command")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"❌ Error: {e}")
                
    def run_basic_test(self, duration=10):
        """Run basic test for specified duration"""
        print(f"\nRunning basic {self.robot_type.upper()} test for {duration} seconds...")
        print("Press Ctrl+C to exit early and enter interactive mode")
        
        try:
            for i in range(int(240 * duration)):
                pb.stepSimulation()
                time.sleep(1/240)
                        
            print(f"Basic test complete after {duration} seconds")
            
        except KeyboardInterrupt:
            print("\nBasic test interrupted by user")
            
        # Enter interactive mode after basic test
        self.interactive_control()
        
    def close(self):
        """Close PyBullet connection and cleanup"""
        try:
            # Cleanup constraints if they exist
            if hasattr(self, 'gripper_constraint'):
                pb.removeConstraint(self.gripper_constraint)
            
            # Cleanup flange adapter if it exists
            if hasattr(self, 'flange_id'):
                pb.removeBody(self.flange_id)
                
            print("🧹 Cleaned up constraints and flange adapter")
        except Exception as e:
            print(f"⚠️ Cleanup warning: {e}")
        finally:
            pb.disconnect()

# Main execution
if __name__ == "__main__":
    print("🤖 Universal Robot Testing Platform")
    print("Supports: Panda, UR3 (with real URDF from urdf_files_dataset)")
    
    # NOW USING REAL UR3!
    robot_type = "ur3"  # Using real UR3 from urdf_files_dataset!
    
    tester = RobotTester(robot_type=robot_type)
    
    try:
        # Run basic test then enter interactive mode
        tester.run_basic_test(duration=3)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        tester.close()