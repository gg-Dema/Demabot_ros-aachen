#!/usr/bin/env python3

import rclpy
import tf2_ros

import tf_transformations  # euler <--> quaternion 
from geometry_msgs.msg import TransformStamped, Twist


# generic value: init condition 
left_wheel_yaw = 0.0        # rotation around z left
right_wheel_yaw = 0.0       # rotation around z right
x_vel = 0.0                 # linear velocity for robot
yaw_vel = 0.0               # angular velocity for robot               
AXIS_L = 0.15
WHEEL_R = 0.03 


# ------------------------------------------
# KINEMATIC AND DIFF. KINEMATIC FUNCT
# ------------------------------------------

def calculate_wheel_rotation(): 
    global x_vel, yaw_vel, AXIS_L, WHEEL_R
    left_vel = 1 / WHEEL_R * (x_vel - (AXIS_L / 2) * yaw_vel)
    right_vel = 1 / WHEEL_R * (x_vel + (AXIS_L / 2) * yaw_vel)
    return left_vel, right_vel

def calculate_wheel_position():
    global left_wheel_yaw, right_wheel_yaw, this_instant_time, dynamic_tf_node

    left_wheel_vel, right_wheel_vel = calculate_wheel_rotation()
    delta_t = ( dynamic_tf_node.get_clock().now().nanoseconds / (10**9) ) - this_instant_time
    left_wheel_yaw = left_wheel_yaw + (delta_t * left_wheel_vel)
    right_wheel_yaw = right_wheel_yaw + (delta_t * right_wheel_vel)

    this_instant_time = dynamic_tf_node.get_clock().now().nanoseconds / (10 ** 9)
    
    return left_wheel_yaw, right_wheel_yaw

# ------------------------------------------
# ------------------------------------------

def quaternion_from_euler(roll, pitch, yaw) -> TransformStamped.transform: #rotation  
    global dynamic_tf_node, left_wheel_yaw, right_wheel_yaw

    quaternion_rot = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    transformation_msg = TransformStamped()
    transformation_msg.transform.rotation.x = quaternion_rot[0]
    transformation_msg.transform.rotation.y = quaternion_rot[1]
    transformation_msg.transform.rotation.z = quaternion_rot[2]
    transformation_msg.transform.rotation.w = quaternion_rot[3]
    
    return transformation_msg.transform.rotation

# TransformStamped ~ geometry_msgs.msg.TransofrmStamped
def calculate_tf() -> TransformStamped: 
    global dynamic_tf_node, left_wheel_yaw, right_wheel_yaw
    
    transform_msg = TransformStamped()
    # get wheel (encoder?) + update it
    left_wheel_yaw, right_wheel_yaw = calculate_wheel_position()
    
    #fill msg: 

    # header: time + ref_frame 
    stamp = dynamic_tf_node.get_clock().now().to_msg()
    transform_msg.header.stamp = stamp
    transform_msg.header.frame_id = 'base_link' 

    # msg body: Vector3 translation + Quaternion rotation
    transform_msg.transform.translation.x = 0.10
    transform_msg.transform.translation.z = -0.3

    return transform_msg

# setting global var for vel
def get_bot_velocity_callback(msg:Twist):
    global x_vel, yaw_vel
    x_vel = msg.linear.x
    yaw_vel = msg.angular.z


def send_transform_callback():
    global dynamic_tf_node

    broadcaster = tf2_ros.transform_broadcaster.TransformBroadcaster(dynamic_tf_node)
    base_2_wheel_tf = calculate_tf() # still msg Transform Stamped

    # left wheels
    base_2_wheel_tf.child_frame_id = 'left_front_wheel'
    base_2_wheel_tf.transform.translation.y = 0.1
    base_2_wheel_tf.transform.rotation = quaternion_from_euler(-1.57, left_wheel_yaw, 0.0)
    broadcaster.sendTransform(base_2_wheel_tf)

    # right wheels
    base_2_wheel_tf.child_frame_id = 'right_front_wheel'
    base_2_wheel_tf.transform.translation.y = -0.1
    base_2_wheel_tf.transform.rotation = quaternion_from_euler(1.57, right_wheel_yaw, 0.0)
    broadcaster.sendTransform(base_2_wheel_tf)



# -------------------------------------------------------



def entry_point():

    global dynamic_tf_node, this_instant_time

    rclpy.init()

    dynamic_tf_node = rclpy.create_node('dynamic_tf')
    dynamic_tf_node.create_subscription(Twist, '/cmd_vel', get_bot_velocity_callback, 10)
    dynamic_tf_node.create_timer(1.0 /1000.0, send_transform_callback)
    this_instant_time = dynamic_tf_node.get_clock().now().nanoseconds / (10 ** 9)

    try:
        rclpy.spin(dynamic_tf_node)
    except KeyboardInterrupt:
        pass

    dynamic_tf_node.destroy_node()
    rclpy.shutdown()

if __name__== "__main__":
    entry_point()
