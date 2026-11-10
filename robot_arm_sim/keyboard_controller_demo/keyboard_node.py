#!/usr/bin/env python3

import math
import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class KeyboardArmController(Node):

    def __init__(self):
        super().__init__('keyboard_arm_controller')

        # Publishes commands to the ROS 2 JointTrajectoryController
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # Must eventually match the joint names in your URDF
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        self.get_logger().info(
            '\n'
            '6-DOF Keyboard Arm Controller\n'
            '-----------------------------\n'
            '1 = Position A\n'
            '2 = Position B\n'
            '3 = Position C\n'
            'h = Home\n'
            'q = Quit\n'
        )

    def move_arm(self, positions_deg, duration=2.0):
        """
        Send all six joints to the requested positions.

        positions_deg:
            List of six joint angles in degrees.

        duration:
            Time allowed for the arm to reach the target.
        """

        if len(positions_deg) != 6:
            self.get_logger().error(
                'Expected exactly 6 joint positions.'
            )
            return

        # ROS uses radians for revolute joints
        positions_rad = [
            math.radians(angle)
            for angle in positions_deg
        ]

        trajectory = JointTrajectory()

        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        point.positions = positions_rad

        # Set how long the robot should take to reach the target
        seconds = int(duration)

        nanoseconds = int(
            (duration - seconds) * 1_000_000_000
        )

        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = nanoseconds

        trajectory.points.append(point)

        self.trajectory_pub.publish(trajectory)

        self.get_logger().info(
            f'Sent target (degrees): {positions_deg}'
        )


def get_key():
    """
    Read one keyboard key without requiring Enter.
    """

    fd = sys.stdin.fileno()

    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)

    finally:
        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old_settings
        )

    return key


def main(args=None):

    rclpy.init(args=args)

    node = KeyboardArmController()

    # Temporary test positions.
    # These should later be changed to match your real robot's joint limits.

    HOME = [
        0,
        0,
        0,
        0,
        0,
        0
    ]

    POSITION_A = [
        0,
        20,
        -20,
        0,
        0,
        0
    ]

    POSITION_B = [
        30,
        30,
        -40,
        20,
        0,
        0
    ]

    POSITION_C = [
        -30,
        15,
        -30,
        0,
        25,
        0
    ]

    try:

        while rclpy.ok():

            key = get_key()

            if key == '1':

                node.move_arm(
                    POSITION_A,
                    duration=2.0
                )

            elif key == '2':

                node.move_arm(
                    POSITION_B,
                    duration=2.0
                )

            elif key == '3':

                node.move_arm(
                    POSITION_C,
                    duration=2.0
                )

            elif key.lower() == 'h':

                node.move_arm(
                    HOME,
                    duration=2.0
                )

            elif key.lower() == 'q':

                node.get_logger().info(
                    'Exiting keyboard controller.'
                )
                break

            rclpy.spin_once(
                node,
                timeout_sec=0.01
            )

    except KeyboardInterrupt:

        node.get_logger().info(
            'Keyboard interrupt received.'
        )

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
