
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32,Int32
import argparse

class SusGrip2FControl(Node):
    def __init__(self, pos:Float32, vel:Int32, force:Int32):
        super().__init__('susgrip_2f_control_cmd_node')

        self.pos_publisher = self.create_publisher(
            Float32, 
            '/susgrip_2f/dis', 
            10)

        self.vel_publisher = self.create_publisher(
            Int32, 
            '/susgrip_2f/vel', 
            10)

        self.tor_publisher = self.create_publisher(
            Int32, 
            '/susgrip_2f/force', 
            10)

        self.pos_value = pos
        self.vel_value = vel
        self.force_value = force
        self.publish_command()

    def publish_command(self):
        if self.vel_value != None:
            msg = Int32()
            msg.data = int(self.vel_value)
            time.sleep(0.002)
            self.vel_publisher.publish(msg)
            self.get_logger().info(f'Published: {msg.data} to /susgrip_2f/vel')
        if self.vel_value != None:
            msg = Int32()
            msg.data = int(self.force_value)
            time.sleep(0.002)
            self.tor_publisher.publish(msg)
            self.get_logger().info(f'Published: {msg.data} to /susgrip_2f/tor')
        if self.pos_value != None:    
            msg = Float32()
            time.sleep(0.002)
            msg.data = self.pos_value
            self.pos_publisher.publish(msg)
            self.get_logger().info(f'Published: {msg.data} to /susgrip_2f/dis')

def parse_args():
    parser = argparse.ArgumentParser(description="SusGrip 2F Control Node")
    parser.add_argument('--m', type=float, default=10.0, help="Move value for the susgrip 2f (default: 65.0mm)")
    parser.add_argument('--v', type=float, default=50.0, help="Set velocity for the susgrip 2f (default: 50%)")
    parser.add_argument('--f', type=float, default=50.0, help="Set force for the susgrip 2f (default: 50%)")
    args = parser.parse_args()
    return args

def main(args=None):
    rclpy.init(args=args)

    args = parse_args()
    print(args)

    node = SusGrip2FControl(pos=args.m, vel=args.v, force=args.f)
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
