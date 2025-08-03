#!/usr/bin/env python3
"""
Test script for QuestUR3ArmModule - validates UR3 arm control and trajectory tracking
without VR interface (standalone testing)
"""

import pybullet as pb
import numpy as np
import time
from scipy.spatial.transform import Rotation

# Import the new UR3 module
from quest_robot_module import QuestUR3ArmModule
from rigidbodySento import create_primitive_shape

class UR3TrajectoryTester:
    """Test UR3 arm trajectory tracking capabilities"""
    
    def __init__(self, visualize=True):
        self.visualize = visualize
        
        # Setup PyBullet environment
        if visualize:
            self.physics_client = pb.connect(pb.GUI)
            pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 1)
        else:
            self.physics_client = pb.connect(pb.DIRECT)
        
        pb.setGravity(0, 0, -9.81)
        # pb.loadURDF("plane.urdf")
        
        # Create visualization spheres for trajectory targets
        self.vis_spheres = []
        colors = [[1,0,0,1], [0,1,0,1], [0,0,1,1], [1,1,0,1]]
        for i in range(4):
            sphere = create_primitive_shape(pb, 0.02, pb.GEOM_SPHERE, [0.02], color=colors[i])
            self.vis_spheres.append(sphere)
        
        print("🤖 Initializing UR3 Quest Module...")
        
        # Initialize UR3 module (using dummy network parameters for testing)
        try:
            self.ur3_module = QuestUR3ArmModule(
                vr_ip="127.0.0.1", 
                local_ip="127.0.0.1", 
                pose_cmd_port=12346, 
                ik_result_port=None,  # No network for testing
                vis_sp=self.vis_spheres
            )
            print("✅ UR3 module initialized successfully!")
            self.print_ur3_info()
        except Exception as e:
            print(f"❌ Failed to initialize UR3 module: {e}")
            raise
    
    def print_ur3_info(self):
        """Print UR3 robot information"""
        print("\n=== UR3 Robot Information ===")
        print(f"End-effector link: {self.ur3_module.end_effector_link}")
        print(f"Joint limits (lower): {[f'{x:.2f}' for x in self.ur3_module.ur3_lower_limits]}")
        print(f"Joint limits (upper): {[f'{x:.2f}' for x in self.ur3_module.ur3_upper_limits]}")
        print(f"Rest pose: {[f'{x:.2f}' for x in self.ur3_module.ARM_REST]}")
        print(f"DOF: {len(self.ur3_module.ur3_lower_limits)}")
    
    def test_basic_ik(self):
        """Test basic IK functionality"""
        print("\n🧪 Testing Basic IK...")
        
        # Test positions around the robot
        test_positions = [
            [0.3, 0.0, 0.3],   # Front
            [0.0, 0.3, 0.3],   # Side
            [0.2, 0.2, 0.5],   # High front-side
            [-0.1, 0.0, 0.2],  # Behind base
        ]
        
        for i, target_pos in enumerate(test_positions):
            print(f"  Testing position {i+1}: {target_pos}")
            
            # Default orientation (pointing down)
            target_orn = Rotation.from_euler('xyz', [np.pi, 0, 0])
            
            try:
                # Solve IK
                joint_angles = self.ur3_module.solve_arm_ik(
                    np.array(target_pos), 
                    target_orn
                )
                
                # Set position and get actual result
                arm_q, _, actual_pos, actual_orn = self.ur3_module.solve_system_world(
                    np.array(target_pos), 
                    target_orn
                )
                
                # Calculate position error
                pos_error = np.linalg.norm(np.array(target_pos) - actual_pos)
                
                print(f"    Joint angles: {[f'{x:.2f}' for x in joint_angles]}")
                print(f"    Position error: {pos_error:.4f}m")
                
                # Visualize target
                if self.visualize and i < len(self.vis_spheres):
                    pb.resetBasePositionAndOrientation(
                        self.vis_spheres[i], 
                        target_pos, 
                        [0, 0, 0, 1]
                    )
                
                if pos_error < 0.01:  # 1cm tolerance
                    print("    ✅ IK solution within tolerance")
                else:
                    print("    ⚠️ IK solution outside tolerance")
                
                pb.stepSimulation()
                time.sleep(1)
                
            except Exception as e:
                print(f"    ❌ IK failed: {e}")
    
    def test_circular_trajectory(self, radius=0.2, height=0.3, num_points=20):
        """Test circular trajectory following"""
        print(f"\n🔄 Testing Circular Trajectory (r={radius}m, h={height}m, {num_points} points)")
        
        # Reset to home
        self.ur3_module.set_joint_positions(self.ur3_module.ur3_arm, self.ur3_module.ARM_REST)
        
        trajectory_points = []
        
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            x = 0.3 + radius * np.cos(angle)
            y = radius * np.sin(angle)
            z = height
            
            target_pos = np.array([x, y, z])
            target_orn = Rotation.from_euler('xyz', [np.pi, 0, 0])
            
            try:
                # Solve IK and execute
                arm_q, _, actual_pos, actual_orn = self.ur3_module.solve_system_world(
                    target_pos, 
                    target_orn
                )
                
                # Store trajectory point
                trajectory_points.append({
                    'target': np.array(target_pos).copy(),
                    'actual': np.array(actual_pos).copy(),
                    'joints': np.array(arm_q).copy(),
                    'error': np.linalg.norm(target_pos - actual_pos)
                })
                
                print(f"  Point {i+1:2d}: target={target_pos}, error={trajectory_points[-1]['error']:.4f}m")
                
                pb.stepSimulation()
                time.sleep(0.1)
                
            except Exception as e:
                print(f"  ❌ Point {i+1} failed: {e}")
        
        # Calculate trajectory statistics
        errors = [p['error'] for p in trajectory_points]
        if errors:
            print(f"\n📊 Trajectory Statistics:")
            print(f"  Average error: {np.mean(errors):.4f}m")
            print(f"  Max error: {np.max(errors):.4f}m")
            print(f"  Min error: {np.min(errors):.4f}m")
            print(f"  Success rate: {sum(1 for e in errors if e < 0.01) / len(errors) * 100:.1f}%")
    
    def test_joint_limits(self):
        """Test robot joint limits"""
        print("\n⚙️ Testing Joint Limits...")
        
        # Test each joint at its limits
        for joint_idx in range(len(self.ur3_module.ARM_REST)):
            print(f"  Testing joint {joint_idx + 1}:")
            
            # Test lower limit
            test_pose = self.ur3_module.ARM_REST.copy()
            test_pose[joint_idx] = self.ur3_module.ur3_lower_limits[joint_idx]
            
            try:
                self.ur3_module.set_joint_positions(self.ur3_module.ur3_arm, test_pose)
                print(f"    Lower limit ({test_pose[joint_idx]:.2f}) ✅")
                pb.stepSimulation()
                time.sleep(0.5)
            except Exception as e:
                print(f"    Lower limit failed: {e}")
            
            # Test upper limit
            test_pose[joint_idx] = self.ur3_module.ur3_upper_limits[joint_idx]
            
            try:
                self.ur3_module.set_joint_positions(self.ur3_module.ur3_arm, test_pose)
                print(f"    Upper limit ({test_pose[joint_idx]:.2f}) ✅")
                pb.stepSimulation()
                time.sleep(0.5)
            except Exception as e:
                print(f"    Upper limit failed: {e}")
        
        # Return to home
        self.ur3_module.set_joint_positions(self.ur3_module.ur3_arm, self.ur3_module.ARM_REST)
        print("  Returned to home position ✅")
    
    def run_comprehensive_test(self):
        """Run all tests"""
        print("\n🚀 Starting UR3 Comprehensive Test Suite")
        print("=" * 50)
        
        try:
            # Basic tests
            self.test_basic_ik()
            self.test_joint_limits()
            self.test_circular_trajectory()
            
            print("\n🎉 All tests completed successfully!")
            print("=" * 50)
            
        except Exception as e:
            print(f"\n❌ Test suite failed: {e}")
            raise
    
    def interactive_mode(self):
        """Interactive control mode"""
        print("\n🎮 Interactive UR3 Control Mode")
        print("Commands:")
        print("  'home' - Return to home position")
        print("  'test' - Run comprehensive test suite")
        print("  'ik' - Test basic IK")
        print("  'circle' - Test circular trajectory")
        print("  'limits' - Test joint limits")
        print("  'move x y z' - Move to position")
        print("  'info' - Show robot info")
        print("  'quit' - Exit")
        
        while True:
            try:
                cmd = input("\n[UR3] Enter command: ").strip().lower()
                
                if cmd == 'quit':
                    break
                elif cmd == 'home':
                    self.ur3_module.set_joint_positions(self.ur3_module.ur3_arm, self.ur3_module.ARM_REST)
                    print("✅ Moved to home position")
                elif cmd == 'test':
                    self.run_comprehensive_test()
                elif cmd == 'ik':
                    self.test_basic_ik()
                elif cmd == 'circle':
                    self.test_circular_trajectory()
                elif cmd == 'limits':
                    self.test_joint_limits()
                elif cmd == 'info':
                    self.print_ur3_info()
                elif cmd.startswith('move'):
                    parts = cmd.split()
                    if len(parts) == 4:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        target_pos = np.array([x, y, z])
                        target_orn = Rotation.from_euler('xyz', [np.pi, 0, 0])
                        
                        arm_q, _, actual_pos, _ = self.ur3_module.solve_system_world(target_pos, target_orn)
                        error = np.linalg.norm(target_pos - actual_pos)
                        print(f"✅ Moved to [{x:.2f}, {y:.2f}, {z:.2f}], error: {error:.4f}m")
                    else:
                        print("❌ Usage: move x y z")
                else:
                    print("❌ Unknown command")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"❌ Error: {e}")
    
    def close(self):
        """Close PyBullet connection"""
        pb.disconnect()

def main():
    """Main test execution"""
    print("🤖 UR3 Quest Module Test Suite")
    print("Testing UR3 arm control and trajectory tracking capabilities")
    
    tester = UR3TrajectoryTester(visualize=True)
    
    try:
        # Run quick validation
        print("\n🔧 Quick validation...")
        tester.test_basic_ik()
        
        # Enter interactive mode
        tester.interactive_mode()
        
    except KeyboardInterrupt:
        print("\n👋 Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tester.close()

if __name__ == "__main__":
    main()