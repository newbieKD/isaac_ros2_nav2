#!/usr/bin/env python3

# Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time

# for carter
# class TestROS2Bridge(Node):
#     def __init__(self):

#         super().__init__("test_ros2bridge")

#         # Create the publisher. This publisher will publish a JointState message to the /joint_command topic.
#         self.publisher_ = self.create_publisher(JointState, "joint_command", 10)

#         # Create a JointState message
#         self.joint_state = JointState()

#         self.joint_state.name = [
#             "panda_joint1",
#             "panda_joint2",
#             "panda_joint3",
#             "panda_joint4",
#             "panda_joint5",
#             "panda_joint6",
#             "panda_joint7",
#             "panda_finger_joint1",
#             "panda_finger_joint2",
#         ]

#         num_joints = len(self.joint_state.name)

#         # make sure kit's editor is playing for receiving messages
#         self.joint_state.position = np.array([0.0] * num_joints, dtype=np.float64).tolist()
#         self.default_joints = [0.0, -1.16, -0.0, -2.3, -0.0, 1.6, 1.1, 0.4, 0.4]

#         # limiting the movements to a smaller range (this is not the range of the robot, just the range of the movement
#         self.max_joints = np.array(self.default_joints) + 0.5
#         self.min_joints = np.array(self.default_joints) - 0.5

#         # position control the robot to wiggle around each joint
#         self.time_start = time.time()

#         timer_period = 0.05  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)

#     def timer_callback(self):
#         self.joint_state.header.stamp = self.get_clock().now().to_msg()

#         joint_position = (
#             np.sin(time.time() - self.time_start) * (self.max_joints - self.min_joints) * 0.5 + self.default_joints
#         )
#         self.joint_state.position = joint_position.tolist()

#         # Publish the message to the topic
#         self.publisher_.publish(self.joint_state)

# for humanoid
class TestROS2Bridge(Node):
    def __init__(self):
        super().__init__("test_ros2bridge")

        self.publisher_ = self.create_publisher(JointState, "joint_command", 10)

        # Human robot joints
        self.joint_names = [
            "torso_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_yaw_joint",
            "right_shoulder_roll_joint",
            "right_elbow_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_knee_joint",
            "right_ankle_joint",
            "right_hip_yaw_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_yaw_joint",
            "left_shoulder_roll_joint",
            "left_elbow_joint",
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_knee_joint",
            "left_ankle_joint",
            "left_hip_yaw_joint",
        ]

        self.joint_state = JointState()
        self.joint_state.name = self.joint_names

        num_joints = len(self.joint_names)

        # Set default joints at zero
        self.default_joints = np.zeros(num_joints, dtype=np.float64)

        # Set max and min joints wiggle range (±0.5 rad)
        self.max_joints = self.default_joints + 0.5
        self.min_joints = self.default_joints - 0.5

        self.time_start = time.time()

        timer_period = 0.05  # 20Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        joint_position = (
            np.sin(time.time() - self.time_start) * (self.max_joints - self.min_joints) * 0.5 + self.default_joints
        )
        self.joint_state.position = joint_position.tolist()

        # Publish the message to the topic
        self.publisher_.publish(self.joint_state)




def main(args=None):
    rclpy.init(args=args)

    ros2_publisher = TestROS2Bridge()

    rclpy.spin(ros2_publisher)

    # Destroy the node explicitly
    ros2_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
