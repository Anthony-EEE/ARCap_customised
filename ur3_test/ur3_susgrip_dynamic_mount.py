#!/usr/bin/env python3
"""
UR3 + Susgrip Integration with Dynamic Mounting
Uses the same mounting technique as the original Franka Panda code
"""

import pybullet as pb
import pybullet_data
import numpy as np
import time
from scipy.spatial.transform import Rotation

class UR3SusgripSystem:
    # UR3 configuration (matching Quest module style)
    ARM_REST = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]  # UR3 home pose
    JOINT_DAMPING = [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Joint damping for IK
    
    # Gripper configuration (matching Quest module style)
    GRIPPER_OPEN_Q = [100.0]   # mm - gripper open distance
    GRIPPER_CLOSE_Q = [10.0]   # mm - gripper close distance
    
    # Susgrip mounting offsets (matching Quest module style)
    gripper_mount_pos_offset = np.array([0.0, 0.0, 0.0])  # 6cm forward in y direction
    gripper_mount_orn_offset = Rotation.from_euler("xyz", [0., 0., 0.])  # 90° rotation
    
    # Gripper position and orientation offsets (for compatibility)
    gripper_pos_offset = np.array([0.0, 0.0, 0.0])
    gripper_orn_offset = Rotation.from_euler("xyz", [0., 0., 0.])
    
    # Tool Center Point (TCP) offset for IK - Compensate gripper being below wrist
    # Gripper is 6cm BELOW wrist, so we offset the IK target DOWN by 6cm
    # This makes the wrist go 6cm higher, bringing gripper to desired position
    offset_gripper = np.array([0.0, 0.0, -0.15, 0.0, 0.0, 0.0])  # -15cm in Z to lift wrist
    
    def __init__(self):
        # Robot IDs (will be set in load_robots)
        self.ur3_id = None
        self.gripper_id = None
        self.END_EFFECTOR_LINK = None
        
        # Joint limits (will be set after loading robots)
        self.arm_lower_limits = None
        self.arm_upper_limits = None
        self.arm_joint_ranges = None
        self.gripper_lower_limits = None
        self.gripper_upper_limits = None
        self.gripper_joint_ranges = None
        
        # Gripper state tracking (matching Quest module)
        self.is_open = True
        self.last_arm_q = None
        self.last_gripper_q = None
        self.last_action = -1  # -1 = open, 1 = close
        self.last_action_t = time.time()
        
    def setup_environment(self):
        """Setup PyBullet environment"""
        print("🌍 Setting up PyBullet environment...")
        self.physics_client = pb.connect(pb.GUI)
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 1)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0, 0, -9.81)
        pb.resetDebugVisualizerCamera(
            cameraDistance=0.7,          # Zoom in (try 0.7 ~ 1.2)
            cameraYaw=50,                # Horizontal angle around robot
            cameraPitch=-35,            # Vertical angle (negative looks down)
            cameraTargetPosition=[0.4, 0, 0.4]  # Focus on robot base or flange
        )

        
        # Load ground plane
        self.plane_id = pb.loadURDF("plane.urdf")
        
        # Object storage for pick and place
        self.cube_id = None
    
    def get_link_index_by_name(self, robot_id, target_name):
        for i in range(pb.getNumJoints(robot_id)):
            joint_info = pb.getJointInfo(robot_id, i)
            link_name = joint_info[12].decode('utf-8')
            if link_name == target_name:
                return i
        raise ValueError(f"Link '{target_name}' not found in robot.")
    
    def get_joint_limits(self, robot_id):
        """Get joint limits for robot (matching Quest module style)"""
        lower_limits = []
        upper_limits = []
        joint_ranges = []
        
        for i in range(pb.getNumJoints(robot_id)):
            joint_info = pb.getJointInfo(robot_id, i)
            if joint_info[2] != pb.JOINT_FIXED:  # Skip fixed joints
                lower_limits.append(joint_info[8])  # Lower limit
                upper_limits.append(joint_info[9])  # Upper limit
                joint_ranges.append(joint_info[9] - joint_info[8])
        
        return lower_limits, upper_limits, joint_ranges

    def load_robots(self):
        """Load UR3 arm and susgrip gripper"""
        print("🤖 Loading UR3 robot...")
        
        # Load UR3 arm (using your URDF dataset)
        self.ur3_id = pb.loadURDF(
            "urdf_files_dataset/urdf_files/ros-industrial/xacro_generated/universal_robots/ur_description/urdf/ur3.urdf",
            basePosition=[0.0, 0.0, 0.0],  # Base at origin
            baseOrientation=[0, 0, 0, 1],
            useFixedBase=True
        )
        
        # Find end-effector link (auto-detect)
        num_joints = pb.getNumJoints(self.ur3_id)
        # self.END_EFFECTOR_LINK = num_joints - 1  # Usually the last joint
        self.END_EFFECTOR_LINK = self.get_link_index_by_name(self.ur3_id, "tool0")
        
        print(f"✅ UR3 loaded with {num_joints} joints, end-effector at joint {self.END_EFFECTOR_LINK}")
        
        # Store joint configuration for reference
        self.movable_joints = []
        for i in range(num_joints):
            joint_info = pb.getJointInfo(self.ur3_id, i)
            if joint_info[2] != pb.JOINT_FIXED:
                self.movable_joints.append(i)
        
        # Load susgrip gripper (initially at origin - will be positioned dynamically)
        print("🦾 Loading susgrip gripper...")
        self.gripper_id = pb.loadURDF("susgrip/susgrip_description/urdf/susgrip_2f.urdf")
        
        print(f"✅ Susgrip loaded with {pb.getNumJoints(self.gripper_id)} joints")
        
        # Set joint limits (matching Quest module style)
        self.arm_lower_limits, self.arm_upper_limits, self.arm_joint_ranges = self.get_joint_limits(self.ur3_id)
        # Susgrip limits are distance-based (mm), set manually
        self.gripper_lower_limits = [0.0]   # Fully closed
        self.gripper_upper_limits = [130.0]  # Fully open  
        self.gripper_joint_ranges = [130.0]
        
        # Set initial positions
        self.set_joint_positions(self.ur3_id, UR3SusgripSystem.ARM_REST)
        self.control_gripper('open')  # Initialize gripper in open position
        
        # Initial gripper mounting using solve_system_world
        initial_pos = [0.5, 0.0, 0.3]  # Safe initial position
        initial_orn = Rotation.from_euler("xyz", [np.pi, 0, 0])
        self.solve_system_world(initial_pos, initial_orn)
        
    def set_joint_positions(self, robot_id, joint_positions):
        """Set joint positions for robot - handle fixed joints correctly"""
        joint_idx = 0  # Index for joint_positions array
        
        for i in range(pb.getNumJoints(robot_id)):
            joint_info = pb.getJointInfo(robot_id, i)
            if joint_info[2] != pb.JOINT_FIXED:  # Only set non-fixed joints
                if joint_idx < len(joint_positions):
                    pb.resetJointState(robot_id, i, joint_positions[joint_idx])
                    joint_idx += 1
    
    
    def move_to_position(self, target_pos):
        """Move UR3 end-effector to target position using Quest module style"""
        target_orn = Rotation.from_euler("xyz", [np.pi, 0, 0])  # Point down
        
        # Use solve_system_world for consistent behavior
        arm_q, gripper_q, ee_pos, ee_orn = self.solve_system_world(target_pos, target_orn)
        time.sleep(1)  # Allow movement to complete
    
    def solve_arm_ik(self, wrist_pos, wrist_orn, wrist_offset=None):
        """Solve IK for arm (matching Quest module style)"""
        # Apply wrist offset if provided
        if wrist_offset is not None:
            wrist_pos_ = wrist_orn.apply(wrist_offset[:3]) + wrist_pos  # In world frame
            wrist_orn_ = wrist_orn * Rotation.from_euler("xyz", wrist_offset[3:])
        else:
            wrist_pos_ = wrist_pos
            wrist_orn_ = wrist_orn
            
        # Calculate IK with joint limits and damping
        target_q = pb.calculateInverseKinematics(
            self.ur3_id, 
            self.END_EFFECTOR_LINK, 
            wrist_pos_, 
            wrist_orn_.as_quat(),
            lowerLimits=self.arm_lower_limits, 
            upperLimits=self.arm_upper_limits,
            jointRanges=self.arm_joint_ranges, 
            restPoses=UR3SusgripSystem.ARM_REST,
            jointDamping=UR3SusgripSystem.JOINT_DAMPING,
            maxNumIterations=200, 
            residualThreshold=0.0001
        )
        return target_q
    
    def solve_fingertip_ik(self, fingertip_pos):
        """Solve gripper kinematics (matching Quest module style)"""
        # Return gripper distance based on state
        if self.is_open:
            return np.array(UR3SusgripSystem.GRIPPER_OPEN_Q), None
        else:
            return np.array(UR3SusgripSystem.GRIPPER_CLOSE_Q), None
    
    def solve_system_world(self, wrist_pos=None, wrist_orn=None, tip_poses=None):
        """Solve complete system kinematics (matching Quest module style)"""
        # Solve gripper configuration
        gripper_q, midpoint_pos = self.solve_fingertip_ik(tip_poses)

        # Use TCP offset for position compensation only (no orientation change)
        tcp_offset = UR3SusgripSystem.offset_gripper
        arm_q = self.solve_arm_ik(wrist_pos, wrist_orn, wrist_offset=tcp_offset)
        
        # Apply arm configuration
        self.set_joint_positions(self.ur3_id, arm_q)
        
        # Brief physics step to ensure joint states are updated
        for _ in range(5):
            pb.stepSimulation()
            time.sleep(0.001)
        
        # Get end-effector state for gripper mounting (matching Quest line 522-524)
        ee_state = pb.getLinkState(self.ur3_id, self.END_EFFECTOR_LINK)
        hand_xyz = np.asarray(ee_state[0])
        hand_orn = Rotation.from_quat(ee_state[1])
        
        # Mount gripper dynamically (exact Quest module style)
        gripper_pos = hand_xyz + (hand_orn * UR3SusgripSystem.gripper_mount_orn_offset).apply(UR3SusgripSystem.gripper_mount_pos_offset)
        gripper_orn = (hand_orn * UR3SusgripSystem.gripper_mount_orn_offset).as_quat()
        pb.resetBasePositionAndOrientation(self.gripper_id, gripper_pos, gripper_orn)
    
        # Apply gripper configuration
        self.control_gripper_by_distance(gripper_q[0])
        
        # Store current configurations
        self.this_arm_q = arm_q
        self.this_gripper_q = gripper_q
        
        # Return arm_q, gripper_q, end_effector_pos, end_effector_orn (Quest format)
        return arm_q, gripper_q, np.asarray(pb.getLinkState(self.ur3_id, self.END_EFFECTOR_LINK)[0]), np.asarray(pb.getLinkState(self.ur3_id, self.END_EFFECTOR_LINK)[1])

    def get_joint_index_by_name(self, robot_id, target_name):
        for i in range(pb.getNumJoints(robot_id)):
            joint_info = pb.getJointInfo(robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            if joint_name == target_name:
                return i
        raise ValueError(f"Joint '{target_name}' not found in robot.")

    def calculate_gripper_joint_positions(self, gripper_distance_mm):
        """
        Calculate all joint positions for susgrip based on finger distance
        Based on the kinematic equations from susgrip driver
        """
        import math
        
        # Kinematic calculation from susgrip driver
        radian = math.acos((63.45 - gripper_distance_mm) / 108)
        yF = math.sqrt(2916 - (31.725 - gripper_distance_mm/2)**2) + 3.5
        slider = (57.5 - yF) / 1000  # Convert to meters
        buff = -math.pi/2 + radian
        
        # Joint positions based on susgrip driver mapping
        joint_positions = {}
        joint_positions["base_slider_l_joint"] = slider
        joint_positions["base_slider_r_joint"] = slider
        joint_positions["slider_outer_l_joint"] = -buff
        joint_positions["slider_outer_r_joint"] = buff
        joint_positions["finger_outer_l_joint"] = buff
        joint_positions["finger_outer_r_joint"] = buff
        
        return joint_positions

    def control_gripper(self, action):
        """Control gripper: 'open' or 'close' using proper susgrip kinematics"""
        if action == 'open':
            target_distance = UR3SusgripSystem.GRIPPER_OPEN_Q[0]
            print("✋ Opening gripper...")
        elif action == 'close':
            target_distance = UR3SusgripSystem.GRIPPER_CLOSE_Q[0]
            print("✊ Closing gripper...")
        else:
            print(f"❌ Unknown gripper action: {action}")
            return
        
        # Calculate joint positions for target distance
        joint_positions = self.calculate_gripper_joint_positions(target_distance)
        
        # Apply positions to all relevant joints
        for joint_name, position in joint_positions.items():
            try:
                joint_idx = self.get_joint_index_by_name(self.gripper_id, joint_name)
                pb.resetJointState(self.gripper_id, joint_idx, position)
            except ValueError:
                print(f"⚠️  Joint {joint_name} not found in gripper model")
        
        print(f"🎯 Gripper set to {target_distance}mm distance")
    
    def control_gripper_by_distance(self, distance_mm):
        """Control gripper by distance in mm (for Quest module compatibility)"""
        # Calculate joint positions for target distance
        joint_positions = self.calculate_gripper_joint_positions(distance_mm)
        
        # Apply positions to all relevant joints
        for joint_name, position in joint_positions.items():
            try:
                joint_idx = self.get_joint_index_by_name(self.gripper_id, joint_name)
                pb.resetJointState(self.gripper_id, joint_idx, position)
            except ValueError:
                pass  # Joint not found, skip silently for compatibility
    
    def check_delta_joints(self, this_q, prev_q, threshold=0.1):
        """Check if joint change is within threshold (matching Quest module)"""
        if prev_q is None:
            return True
        delta_q = np.abs(np.array(this_q) - np.array(prev_q))
        return np.all(delta_q < threshold)
    
    def is_closer_than_threshold(self, gripper_q, threshold=None):
        """Check if gripper is closed enough (matching Quest module style)"""
        if threshold is None:
            threshold = self.gripper_joint_ranges[0] * 0.95  # 95% of range
        # For susgrip, smaller distance means more closed
        distance = gripper_q[0] if isinstance(gripper_q, (list, np.ndarray)) else gripper_q
        return distance < threshold
    
    def compute_action(self, this_gripper_q):
        """Compute gripper action -1=open, 1=close (matching Quest module)"""
        # Use button-based gripper state if available
        if hasattr(self, 'is_open'):
            if self.is_open:  # gripper is open
                if self.last_action != -1:
                    self.last_action_t = time.time()
                self.last_action = -1
                return -1
            else:  # gripper is closed
                if self.last_action != 1:
                    self.last_action_t = time.time()
                self.last_action = 1
                return 1
        
        # Fallback to threshold-based method
        if time.time() - self.last_action_t < 1.5:
            return self.last_action
            
        if self.is_closer_than_threshold(this_gripper_q, 30.0):  # 30mm threshold for "closed"
            if self.last_action != 1:
                self.last_action_t = time.time()
            self.last_action = 1
            return 1
        else:
            if self.last_action != -1:
                self.last_action_t = time.time()
            self.last_action = -1
            return -1
    
    def spawn_cube(self, position=[0.6, 0.2, 0.05], size=0.05, color=[1, 0, 0, 1]):
        """Spawn a cube for pick and place demo"""
        # Create a cube using PyBullet's primitive shapes
        cube_visual = pb.createVisualShape(
            pb.GEOM_BOX,
            halfExtents=[size/2, size/2, size/2],
            rgbaColor=color
        )
        cube_collision = pb.createCollisionShape(
            pb.GEOM_BOX,
            halfExtents=[size/2, size/2, size/2]
        )
        
        self.cube_id = pb.createMultiBody(
            baseMass=0.1,  # Light cube
            baseCollisionShapeIndex=cube_collision,
            baseVisualShapeIndex=cube_visual,
            basePosition=position,
            baseOrientation=[0, 0, 0, 1]
        )
        
        print(f"📦 Spawned cube at position {position}")
        return self.cube_id
    
    def get_cube_position(self):
        """Get current cube position"""
        if self.cube_id is not None:
            pos, _ = pb.getBasePositionAndOrientation(self.cube_id)
            return np.array(pos)
        return None
    
    def approach_position(self, target_pos, offset_z=0.15):
        """Move to approach position above target"""
        approach_pos = np.array(target_pos) + np.array([0, 0, offset_z])
        print(f"🎯 Approaching position: {approach_pos}")
        
        # Use solve_system_world for consistent behavior
        approach_orn = Rotation.from_euler("xyz", [np.pi, 0, 0])  # Point down
        arm_q, gripper_q, ee_pos, ee_orn = self.solve_system_world(approach_pos, approach_orn)
        time.sleep(2)
        return approach_pos
    
    def move_to_grasp_position(self, target_pos, offset_z=0.03):
        """Move down to grasp position - closer to cube"""
        grasp_pos = np.array(target_pos) + np.array([0, 0, offset_z])
        print(f"🎯 Moving to grasp position: {grasp_pos}")
        
        grasp_orn = Rotation.from_euler("xyz", [np.pi, 0, 0])  # Point down
        arm_q, gripper_q, ee_pos, ee_orn = self.solve_system_world(grasp_pos, grasp_orn)
        time.sleep(2)
        
        # Check actual end-effector position
        actual_ee_pos = np.asarray(pb.getLinkState(self.ur3_id, self.END_EFFECTOR_LINK)[0])
        print(f"   📍 Actual EE position: {actual_ee_pos}")
        print(f"   📏 Distance to cube: {np.linalg.norm(actual_ee_pos - target_pos):.4f}m")
        
        return grasp_pos
    
    def lift_object(self, current_pos, lift_height=0.15):
        """Lift object up after grasping"""
        lift_pos = np.array(current_pos) + np.array([0, 0, lift_height])
        print(f"⬆️ Lifting to position: {lift_pos}")
        
        lift_orn = Rotation.from_euler("xyz", [np.pi, 0, 0])  # Point down
        arm_q, gripper_q, ee_pos, ee_orn = self.solve_system_world(lift_pos, lift_orn)
        time.sleep(2)
        return lift_pos
    
    def check_cube_grasped(self, initial_pos, tolerance=0.01):
        """Check if cube has been successfully grasped by checking if it moved"""
        current_pos = self.get_cube_position()
        if current_pos is None:
            return False
        
        distance_moved = np.linalg.norm(current_pos - initial_pos)
        is_grasped = distance_moved > tolerance
        
        print(f"   🔍 Grasp check: {'✅ SUCCESS' if is_grasped else '❌ FAILED'}")
        print(f"   📏 Cube moved: {distance_moved:.4f}m (threshold: {tolerance}m)")
        
        return is_grasped
    
    def run_pick_place_demo(self):
        """Complete pick and place demonstration"""
        print("\n🎬 Starting UR3 + Susgrip Pick & Place Demo")
        print("=" * 50)
        
        # Step 1: Spawn cube
        print("1️⃣ Spawning target cube...")
        cube_position = [0.4, 0.3, 0.025]  # Cube on ground level (table height)
        self.spawn_cube(position=cube_position, size=0.05, color=[1, 0, 0, 1])  # Red cube
        time.sleep(1)
        
        # Step 2: Go to initial position
        initial_pos = [0.3, 0.0, 0.3]  # Normal working height
        print(f"2️⃣ Moving to initial position: {initial_pos}")
        self.move_to_position(initial_pos)
        # Verify position reached
        current_ee_pos = np.asarray(pb.getLinkState(self.ur3_id, self.END_EFFECTOR_LINK)[0])
        position_error = np.linalg.norm(current_ee_pos - initial_pos)
        print(f"   ✅ Position reached with {position_error*1000:.1f}mm accuracy")
        time.sleep(1)
        
        # Step 3: Open gripper
        print("3️⃣ Opening gripper...")
        self.is_open = True
        self.control_gripper('open')
        time.sleep(1)
        
        # step 4: approach cube from above
        print("4️⃣ Approaching cube...")
        approach_pos = [cube_position[0], cube_position[1], cube_position[2] + 0.1]  # 10cm above cube
        self.move_to_position(approach_pos)
        time.sleep(1)
        
        # step 4b: move down to grasp position
        print("4b️⃣ Moving to grasp position...")
        grasp_pos = [cube_position[0], cube_position[1], cube_position[2] + 0.02]  # 2cm above cube
        self.move_to_position(grasp_pos)
        time.sleep(1)
        
        # DEBUG: Check gripper position before grasp
        gripper_state = pb.getBasePositionAndOrientation(self.gripper_id)
        gripper_pos = gripper_state[0]
        print(f"🔍 Gripper position: {gripper_pos}")
        print(f"🔍 Cube position: {cube_position}")
        print(f"🔍 Distance to cube: {np.linalg.norm(np.array(gripper_pos) - np.array(cube_position)):.3f}m")
        
        # step 5: grasp cube
        print("5️⃣ Grasping cube...")
        initial_cube_pos = self.get_cube_position()
        
        # DEBUG: Check gripper joint positions before and after closing
        print("🔍 Gripper joints before closing:")
        for i in range(pb.getNumJoints(self.gripper_id)):
            joint_state = pb.getJointState(self.gripper_id, i)
            joint_info = pb.getJointInfo(self.gripper_id, i)
            print(f"   Joint {i} ({joint_info[1].decode()}): {joint_state[0]:.4f}")
        
        self.control_gripper('close')
        time.sleep(2)  # Give more time for gripper to close
        
        print("🔍 Gripper joints after closing:")
        for i in range(pb.getNumJoints(self.gripper_id)):
            joint_state = pb.getJointState(self.gripper_id, i)
            joint_info = pb.getJointInfo(self.gripper_id, i)
            print(f"   Joint {i} ({joint_info[1].decode()}): {joint_state[0]:.4f}")
        
        # DEBUG: Check for contact points between gripper and cube
        contact_points = pb.getContactPoints(bodyA=self.gripper_id, bodyB=self.cube_id)
        print(f"🔍 Contact points found: {len(contact_points)}")
        if contact_points:
            for i, contact in enumerate(contact_points[:3]):  # Show first 3 contacts
                print(f"   Contact {i+1}: Force={contact[9]:.3f}")
        else:
            print("❌ No contact points - gripper and cube not colliding!")
        
        # Check if cube moved (indicating successful grasp)
        post_grasp_cube_pos = self.get_cube_position()
        if self.check_cube_grasped(initial_cube_pos, tolerance=0.005):
            print("✅ Cube successfully grasped!")
        else:
            print("❌ Cube not grasped - trying to get closer...")
            # Try moving closer
            closer_pos = [cube_position[0], cube_position[1], cube_position[2] + 0.005]  # 5mm above cube
            self.move_to_position(closer_pos)
            time.sleep(1)
            self.control_gripper('close')
            time.sleep(2)

        # step 6: lift cube
        print("6️⃣ Lifting cube...")
        self.lift_object(cube_position, lift_height=0.15)
        time.sleep(1)

        # step 7: move to initial position
        print("7️⃣ Moving to initial position...")
        self.move_to_position(initial_pos)
        time.sleep(1)

        # step 8: open gripper
        print("8️⃣ Opening gripper...")
        self.control_gripper('open')
        time.sleep(1)

        # Final verification
        final_cube_pos = self.get_cube_position()
        print(f"✅ Pick & Place Complete!")
        print(f"   📦 Cube moved from {cube_position} to {final_cube_pos[:3]}")
        print(f"   📏 Distance moved: {np.linalg.norm(np.array(final_cube_pos[:3]) - np.array(cube_position)):.3f}m")
        
        print("\nPress Enter to exit...")
        input()
    

    
    def close(self):
        """Clean up PyBullet"""
        pb.disconnect()

def main():
    print("🤖 UR3 + Susgrip Pick & Place Demo")
    print("Testing precision of UR3 + Susgrip system")
    
    system = UR3SusgripSystem()
    
    try:
        system.setup_environment()
        system.load_robots()
        system.run_pick_place_demo()
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        system.close()

if __name__ == "__main__":
    main()