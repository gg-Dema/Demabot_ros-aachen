import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import Int32

pub = None


def entry_point():
    global pub
    rclpy.init()
    node = rclpy.create_node('detector')
    node.create_subscription(AprilTagDetectionArray, 'detections', callback, 10)
    pub = node.create_publisher(Int32, 'tag_ids', 1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


def callback(msg):
    for detection in msg.detections:
        id = detection.id
        pub_msg = Int32()
        pub_msg.data = int(id)
        pub.publish(pub_msg)


if __name__ == '__main__':
    entry_point()
