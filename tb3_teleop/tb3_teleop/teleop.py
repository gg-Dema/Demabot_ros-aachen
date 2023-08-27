#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TeleopNode(Node):

    def __init__(self):
        super().__init__('TeleopNode')
        buffer_size = 1
        
        self.create_subscription(Joy, '/joy', self.joy_subscription, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', buffer_size)
        #self.create_timer(0.1, self.publisher_callback)


    def joy_subscription(self, msg):
        print(f'get command:\nx: {msg.axes[0]}, z: {msg.axes[1]}')
        msg_cmd_vel = Twist()
        msg_cmd_vel.linear.x = msg.axes[0]
        msg_cmd_vel.angular.z = msg.axes[1]
        self.publisher.publish(msg_cmd_vel)


    def publisher_callback(self): 
        pass
        
def entry_point():

    rclpy.init()
    teleop_node = TeleopNode()

    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass

    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    entry_point()