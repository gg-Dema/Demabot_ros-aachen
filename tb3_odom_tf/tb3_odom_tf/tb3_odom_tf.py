
import rclpy
import tf2_ros
import tf_transformations
from geometry_msgs.msg import TransformStamped, Twist
from math import cos, sin 

# GLOBAL VAL: 
robot_x = 0.0
robot_y = 0.0
robot_yaw = 0.0


def quaternion_from_euler(roll, pitch, yaw): 
    global dynamic_tf_node, left_wheel_yaw, right_wheel_yaw

    quaternion_rot = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    transformation_msg = TransformStamped()
    transformation_msg.transform.rotation.x = quaternion_rot[0]
    transformation_msg.transform.rotation.y = quaternion_rot[1]
    transformation_msg.transform.rotation.z = quaternion_rot[2]
    transformation_msg.transform.rotation.w = quaternion_rot[3]
    
    return transformation_msg.transform.rotation


def send_robot_position_callback():
    global odom_node
    broadcaster = tf2_ros.transform_broadcaster.TransformBroadcaster(odom_node)
    
    odom_msg = TransformStamped()
    odom_msg.header.stamp = odom_node.get_clock().now().to_msg()
    odom_msg.header.frame_id = "odom"

    # in theory i should integrate the new position
    odom_msg.transform.translation.x = robot_x
    odom_msg.transform.translation.y = robot_y
    odom_msg.transform.rotation = quaternion_from_euler(0.0, 0.0, robot_yaw)

    odom_msg.child_frame_id = "base_link"
    
    broadcaster.sendTransform(odom_msg)


def update_robot_position_callback(msg:Twist):
    global robot_x, robot_y, robot_yaw
    global odom_node, time_last_update

    x_vel = msg.linear.x * 0.01
    yaw_vel = msg.angular.z * 0.01
    delta_time =  ((odom_node.get_clock().now().nanoseconds / (10**9)) - time_last_update)


    delta_yaw = yaw_vel * delta_time

    robot_yaw = robot_yaw + delta_yaw
    robot_x += (x_vel * cos(robot_yaw)) * delta_time
    robot_y += (x_vel * sin(robot_yaw)) * delta_time


def entry_point():

    global odom_node, time_last_update

    rclpy.init()
    odom_node = rclpy.create_node('odom')
    odom_node.create_subscription(Twist, '/cmd_vel', update_robot_position_callback, 10)
    odom_node.create_timer(1.0/1000.0, send_robot_position_callback)

    time_last_update = odom_node.get_clock().now().nanoseconds / (10**9)

    try:
        rclpy.spin(odom_node)
    except KeyboardInterrupt:
        pass

    odom_node.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    entry_point()