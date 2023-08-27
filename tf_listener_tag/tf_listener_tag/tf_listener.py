#! /usr/bin/env python3

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def printer(msg_tf : TransformStamped):
    print("----------------------------------------")
    print("Position: ")
    print("  X: " + str(msg_tf.transform.translation.x))
    print("  Y: " + str(msg_tf.transform.translation.y))
    print("  Z: " + str(msg_tf.transform.translation.z))
    print("Orientation: ")
    print("  X: " + str(msg_tf.transform.rotation.x))
    print("  Y: " + str(msg_tf.transform.rotation.y))
    print("  Z: " + str(msg_tf.transform.rotation.z))
    print("  W: " + str(msg_tf.transform.rotation.w))


def timer_callback():
    global tf_buffer
    origin_frame = 'base_link'
    to_frame = 'marker'

    try: 
        transform = tf_buffer.lookup_transform(origin_frame, to_frame, rclpy.duration.Duration())
        printer(transform)
        tf_buffer.clear()
    except: 
        print("nothing to read")


def entry_point(): 
    global tf_buffer
    rclpy.init()
    
    node_listener = rclpy.create_node('tf2_listener')
    tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(nanoseconds=1**9))
    tf2_ros.TransformListener(tf_buffer, node_listener)
    node_listener.create_timer(0.25, timer_callback)

    try: 
        rclpy.spin(node_listener)
    except KeyboardInterrupt:
        node_listener.destroy_node()
        rclpy.shutdown()
        

if __name__=="__main__":
    entry_point()
