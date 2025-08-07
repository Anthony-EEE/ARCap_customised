#!/usr/bin/env python3
import sys, os
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import String
from std_msgs.msg import Float32,Int32
from sensor_msgs.msg import JointState

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from susgrip_2f_utility.SusGripLib import SusGrip2F

class SusGrip2FDriver(Node):
    def __init__(self, serial_port="/dev/ttyUSB0", *args, **kwargs):
        super().__init__('susgrip_2f_driver')
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.is_connected = False

        try:
            self.sus_2f = SusGrip2F(port=serial_port)
            self.sus_2f.set_rtu_mode()
            self.get_logger().info("Connected to gripper successfully at " + serial_port + " !")
            self.is_connected = True
        except:
            self.get_logger().error(f"Failed to connect to gripper:")
            self.is_connected = False
            exit(1)

        # create publisher to load datas to moveit
        self.publisher = self.create_publisher(
            JointState, 
            '/joint_states', 
            10
        )
        self.sus2f_pos_subscription = self.create_subscription(
            Float32, 
            'susgrip_2f/dis',
            self.pos_command_callback,
            qos_profile
        )
        self.sus2f_vel_subscription = self.create_subscription(
            Int32, 
            'susgrip_2f/vel',
            self.vel_command_callback,
            qos_profile
        )
        self.sus2f_force_subscription = self.create_subscription(
            Int32, 
            'susgrip_2f/force',
            self.force_command_callback,
            qos_profile
        )
        self.create_timer(0.05, self.susgrip_2f_get_data)

    def pos_command_callback(self, msg):
        """Send command to gripper via Serial"""
        command = msg.data
        self.get_logger().info(f">> SusGrip 2F Move to : {command} mm")
        try:
            self.sus_2f.rtu_set_pos(float(command))
        except Exception as e:
            self.get_logger().error(f"Error sending command: {e}")
        
    def vel_command_callback(self, msg):
        """Send command to gripper via Serial"""
        command = msg.data
        self.get_logger().info(f">> SusGrip 2F Set Velocity : {command} %")
        try:
            self.sus_2f.rtu_set_vel(float(command))
        except Exception as e:
            self.get_logger().error(f"Error sending command: {e}")
            
    def force_command_callback(self, msg):
        """Send command to gripper via Serial"""
        command = msg.data
        self.get_logger().info(f">> SusGrip 2F Set Force : {command} %")
        try:
            self.sus_2f.rtu_set_tor(float(command))
        except Exception as e:
            self.get_logger().error(f"Error sending command: {e}")


    def susgrip_2f_get_data(self):
        data = self.sus_2f.reload_data()
        # status = data[0]
        # if ((status>>7)&0x01 == 0x00) and (abs(data[2] - self.pos_list[self.pos_index]) <= 1):
        #     self.pos_index += 1
        #     self.pos_index %= 2
        #     self.sus_2f.rtu_write_pos2(self.pos_list[self.pos_index])

        # self.get_logger().info(f"Sus2F : {data }")
        
        self.gripper_dis = data[2]/2
        radian = math.acos((63.45-self.gripper_dis)/108)
        yF = math.sqrt(2916-(31.725-self.gripper_dis/2)**2)+3.5
        slider = (57.5-yF)/1000
        buff = -math.pi/2+radian
        
        joint_sus2f = [0]*12
        """Update joint positions from the UI and publish."""
        joint_sus2f[0] = slider                    # base_slider_l_joint
        joint_sus2f[1] = -buff                     # finger_outer_l_joint
        joint_sus2f[2] = buff                      # outet_dummy_l_joint       
        joint_sus2f[3] = -buff                     # finger_inner_l_joint
        joint_sus2f[4] = 2*buff                    # pad_inner_l_joint
        joint_sus2f[5] = 2*buff                    # passive_pad_inner_l_joint
        joint_sus2f[6] = slider                    # sus2f_slider_r_link
        joint_sus2f[7] = buff                      # slider_outer_r_joint
        joint_sus2f[8] = buff                      # finger_outer_r_joint
        joint_sus2f[9] = buff                      # finger_inner_r_joint
        joint_sus2f[10] = -2*buff                   # pad_inner_r_joint
        joint_sus2f[11] = -2*buff                   # passive_pad_inner_r_joint
        
        """Publish the latest joint positions as a ROS 2 message."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = joint_sus2f
        msg.name = [
            "base_slider_l_joint",
            "slider_outer_l_joint",
            "finger_outer_l_joint",
            "finger_inner_l_joint",
            "pad_inner_l_joint",
            "passive_pad_inner_l_joint",
            "base_slider_r_joint",
            "slider_outer_r_joint",
            "finger_outer_r_joint",
            "finger_inner_r_joint",
            "pad_inner_r_joint",
            "passive_pad_inner_r_joint",
        ]
        self.publisher.publish(msg)

    def close_serial(self):
        """Close serial connection when shutting down"""
        if self.sus_2f.is_connect:
            self.sus_2f.disconnect()
            self.get_logger().info("Closed gripper serial connection.")

    def destroy_node(self):
        """Shutdown procedure"""
        self.close_serial()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SusGrip2FDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down SusGrip 2F Driver...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()