#!/usr/bin/env python3
import math
from math import sqrt, pi
import sys
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

import random

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Quaternion


# Map Bounds
map_min_x = -1.0
map_max_x = 3.0
map_min_y = 0.0
map_max_y = 4.0

success = True


poses = [
[-1.2, -0.2,  0., -.5],
[0., 0., 0., 0.],
[1., .25, 0., -.5],
[2., 0.0, 0.0, 0],
[3., -1, 0, -1.7],
[3.2, -2.5, 0.0, -1.75],
[1., -2.5, 0.0, 3.16],
[2., -1.5, 0.0, 3.16],
[1.0, -1.0, 0.0, 3.16],
[0.0, -2.5, 0.0, 0.0],
[-1.0, -2, 0.0, 0.0]
]

pose_pointer = 0


def main():
  global auto_chaos
  global nav_to_pose_client
  rclpy.init()

  auto_chaos = rclpy.create_node('auto_goals')

  # create Action Client object with desired message type and action name
  nav_to_pose_client = ActionClient(auto_chaos, NavigateToPose, 'navigate_to_pose')

  # wait for action server to come up
  while not nav_to_pose_client.wait_for_server(timeout_sec=2.0):
    print("Server still not available; waiting...")

  while rclpy.ok():
    try:
      position = generatePosition()
      orientation = generateOrientation()
      goal_handle = sendGoal(position, orientation)
      status = checkResult(goal_handle)
    except IndexError:
      print('MAP COMPLETE')
      break
    except KeyboardInterrupt:
      print("Shutdown requested... complying...")
      break
  nav_to_pose_client.destroy()
  auto_chaos.destroy_node()
  rclpy.shutdown()


def sendGoal(position, orientation):
  """Create action goal object and send to action server, check if goal accepted"""
  global auto_chaos
  global nav_to_pose_client

  goal = NavigateToPose.Goal()
  goal.pose.header.frame_id = "map"

  goal.pose.header.stamp = auto_chaos.get_clock().now().to_msg()
  
  goal.pose.pose.position = position
  goal.pose.pose.orientation = orientation

  print("Received new goal => X: " + str(goal.pose.pose.position.x) + " Y: " + str(goal.pose.pose.position.y) + "omega:" + str(goal.pose.pose.orientation))

  send_goal_future = nav_to_pose_client.send_goal_async(goal)
  rclpy.spin_until_future_complete(auto_chaos, send_goal_future)

  goal_handle = send_goal_future.result()

  if not goal_handle.accepted:
    print("Goal was rejected")
    nav_to_pose_client.destroy()
    auto_chaos.destroy_node()
    rclpy.shutdown()
    sys.exit(0)
  print("Goal Accepted!")

  return goal_handle


def checkResult(goal_handle):
  global pose_pointer
  """Check for task completion while blocking further execution"""
  get_result_future = goal_handle.get_result_async()
  rclpy.spin_until_future_complete(auto_chaos, get_result_future)
  status = get_result_future.result().status

  if status == GoalStatus.STATUS_SUCCEEDED:
    pose_pointer = pose_pointer + 1 

  return status


def generatePosition():
  global poses, pose_pointer
  pose = poses[pose_pointer]
  position = Point()
  position.x = pose[0]
  position.y = pose[1]
  position.z = pose[2]
  return position


def generateOrientation():
  global poses, pose_pointer
  pose = poses[pose_pointer]
  q = quaternion_from_euler(pose[3])
  quat = Quaternion()
  quat.w = q[3]
  quat.x = q[0]
  quat.y = q[1]
  quat.z = q[2]
  return quat


def quaternion_from_euler(yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    roll, pitch = 0, 0
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr
    return q

if __name__ == '__main__':
    main()