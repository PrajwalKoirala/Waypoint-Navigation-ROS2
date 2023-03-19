#!/usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler

"""
Basic navigation demo to go to poses.
"""


def poses_from_list(pose_list, navigator):
    poses = []
    for pose in pose_list:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = navigator.get_clock().now().to_msg()
        pose_stamped.pose.position.x = float(pose[0])
        pose_stamped.pose.position.y = float(pose[1])
        orientation = quaternion_from_euler(0, 0, float(pose[2]))
        pose_stamped.pose.orientation.x = orientation[0]
        pose_stamped.pose.orientation.y = orientation[1]
        pose_stamped.pose.orientation.z = orientation[2]
        pose_stamped.pose.orientation.w = orientation[3]
        poses.append(pose_stamped)
    return poses


def main():
    rclpy.init()

    navigator = BasicNavigator()

    initial_pose = [[0.0, 0.0, 0.0]]
    initial_pose = poses_from_list(initial_pose, navigator)
    initial_pose = initial_pose[0]
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    goal_poses = [[1.5, 0.5, 0.0], [1.5, 2.0, 3.14], [2.0, 1.0, 0.0]]
    goal_poses = poses_from_list(goal_poses, navigator)

    nav_start = navigator.get_clock().now()
    last = nav_start
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        now = navigator.get_clock().now()
        if now - last > Duration(seconds=3.0):
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            if now - nav_start > Duration(seconds=600.0):
                navigator.cancelTask()
                print('Navigation took too long, canceling the goal')
            last = now

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
