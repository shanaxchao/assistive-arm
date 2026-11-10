#!/usr/bin/env python3
"""Pick up a 5 cm Gazebo cube at the given world-frame XYZ in metres."""

import argparse
import fcntl
import math
from pathlib import Path
import re
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

BOX_NAME = 'pick_cube'
BOX_HALF_HEIGHT = 0.025
ARM_GROUP = 'ur_manipulator'
ARM_JOINTS = (
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
)
GRASP_LINK = 'pickup_center'
LIFT_HEIGHT = 0.14
FINGER_JOINTS = ('pickup_left_finger', 'pickup_right_finger')
OPEN_FORCE = -2.0
CLOSE_FORCE = 4.0


class Pickup(Node):
    def __init__(self, box_name=BOX_NAME):
        super().__init__('pick_box', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.box_name = box_name
        self.move_group = ActionClient(self, MoveGroup, '/move_action')
        self.execute_path = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.cartesian_path = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.compute_ik = self.create_client(GetPositionIK, '/compute_ik')
        self.gripper = self.create_publisher(Float64MultiArray, '/pickup_gripper/commands', 10)
        self.joints = {}
        self.create_subscription(JointState, '/joint_states', self.update_joints, qos_profile_sensor_data)

    def update_joints(self, message):
        self.joints = dict(zip(message.name, message.position))

    def wait_for(self, predicate, failure, timeout=15.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(failure)
            rclpy.spin_once(self, timeout_sec=min(remaining, 0.1))

    def wait_sim(self, seconds):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        self.wait_for(
            lambda: self.get_clock().now().nanoseconds >= end,
            'Gazebo stopped advancing. Check that the simulation is playing.',
        )

    def ready(self):
        self.wait_for(
            lambda: self.gripper.get_subscription_count()
            and all(joint in self.joints for joint in (*ARM_JOINTS, *FINGER_JOINTS))
            and self.get_clock().now().nanoseconds > 0,
            'Pickup hardware is unavailable. Start pickup.launch.py and check that Gazebo is playing.',
        )
        if not (self.move_group.wait_for_server(timeout_sec=15.0)
                and self.execute_path.wait_for_server(timeout_sec=5.0)
                and self.compute_ik.wait_for_service(timeout_sec=5.0)
                and self.cartesian_path.wait_for_service(timeout_sec=5.0)):
            raise RuntimeError('MoveIt is unavailable. Check the pickup launch terminal.')

    def read_cube(self):
        for attempt in range(3):
            result = subprocess.run(
                ['gz', 'model', '-m', self.box_name, '-p'],
                capture_output=True, text=True, timeout=10.0,
            )
            if f'No model named <{self.box_name}> was found' in result.stdout + result.stderr:
                return None
            result.check_returncode()
            # Harmonic prints XYZ and roll/pitch/yaw as two bracketed lines.
            pose = re.search(
                r'^ +\[([^\[\]\n]+)\]\s*\n +\[([^\[\]\n]+)\]\s*$',
                result.stdout, re.MULTILINE,
            )
            if pose is not None:
                position = tuple(map(float, pose[1].split()))
                _, _, yaw = map(float, pose[2].split())
                return position, yaw
            rclpy.spin_once(self, timeout_sec=0.2)
        raise RuntimeError(
            f'Gazebo did not return the pose of {self.box_name} after three requests.'
        )

    def open_gripper(self):
        self.gripper.publish(Float64MultiArray(data=[OPEN_FORCE, OPEN_FORCE]))
        self.wait_for(
            lambda: all(abs(self.joints[joint]) < 0.001 for joint in FINGER_JOINTS),
            'The gripper did not open.',
        )

    def run_action(self, client, goal, stage):
        request = client.send_goal_async(goal)
        self.wait_for(request.done, f'{stage}: MoveIt did not answer.')
        handle = request.result()
        if not handle.accepted:
            raise RuntimeError(f'{stage}: MoveIt rejected the goal.')
        result = handle.get_result_async()
        try:
            self.wait_for(result.done, f'{stage}: execution timed out.', timeout=90.0)
        except RuntimeError:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
            raise
        response = result.result()
        code = response.result.error_code
        if response.status != GoalStatus.STATUS_SUCCEEDED or code.val != code.SUCCESS:
            raise RuntimeError(f'{stage}: MoveIt failed with error {code.val}. {code.message}'.rstrip())

    def approach(self, target, orientation):
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = ARM_GROUP
        ik.ik_link_name = GRASP_LINK
        ik.robot_state.is_diff = True
        ik.avoid_collisions = True
        ik.timeout.sec = 3
        ik.pose_stamped.header.frame_id = 'base_link'
        ik.pose_stamped.pose = Pose(
            position=Point(x=target[0], y=target[1], z=target[2]), orientation=orientation,
        )
        calculation = self.compute_ik.call_async(request)
        self.wait_for(calculation.done, 'Approach: inverse kinematics timed out.')
        result = calculation.result()
        if result.error_code.val != result.error_code.SUCCESS:
            raise RuntimeError(f'Approach: no collision-free grasp pose found '
                               f'(MoveIt error {result.error_code.val}).')
        state = result.solution.joint_state
        positions = dict(zip(state.name, state.position))
        # IK can return an equivalent pose one full turn away. Keep the
        # nearest representation that is within this UR5e's joint limits.
        constraints = []
        for joint in ARM_JOINTS:
            current = self.joints[joint]
            nearest = current + math.remainder(positions[joint] - current, math.tau)
            limit = math.pi if joint == 'elbow_joint' else math.tau
            target_angle = nearest if abs(nearest) <= limit else positions[joint]
            constraints.append(JointConstraint(
                joint_name=joint, position=target_angle,
                tolerance_above=0.001, tolerance_below=0.001, weight=1.0,
            ))
        goal = MoveGroup.Goal()
        goal.request.group_name = ARM_GROUP
        goal.request.pipeline_id = 'ompl'
        goal.request.start_state.is_diff = True
        goal.request.allowed_planning_time = 8.0
        goal.request.num_planning_attempts = 5
        goal.request.max_velocity_scaling_factor = 0.25
        goal.request.max_acceleration_scaling_factor = 0.25
        goal.request.goal_constraints = [Constraints(joint_constraints=constraints)]
        self.run_action(self.move_group, goal, 'Approach')

    def move_straight(self, target, orientation, stage):
        request = GetCartesianPath.Request()
        request.header.frame_id = 'base_link'
        request.start_state.is_diff = True
        request.group_name = ARM_GROUP
        request.link_name = GRASP_LINK
        request.waypoints = [Pose(position=Point(x=target[0], y=target[1], z=target[2]),
                                  orientation=orientation)]
        request.max_step = 0.005
        request.revolute_jump_threshold = 0.25
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = 0.15
        request.max_acceleration_scaling_factor = 0.15
        plan = self.cartesian_path.call_async(request)
        self.wait_for(plan.done, f'{stage}: Cartesian planning timed out.')
        path = plan.result()
        if path.error_code.val != path.error_code.SUCCESS or path.fraction < 0.999:
            raise RuntimeError(f'{stage}: only {path.fraction:.0%} of the straight path was planned '
                               f'(MoveIt error {path.error_code.val}).')
        goal = ExecuteTrajectory.Goal(trajectory=path.solution)
        self.run_action(self.execute_path, goal, stage)

    def spawn_cube(self, target):
        if self.read_cube() is not None:
            raise RuntimeError(f'{self.box_name} already exists. Use its current coordinates without '
                               '--spawn or restart the pickup launch to start fresh.')
        model = Path(__file__).with_name('pickup_cube.sdf')
        x, y, z = target
        subprocess.run([
            'ros2', 'run', 'ros_gz_sim', 'create', '-world', 'empty', '-name', self.box_name,
            '-file', str(model), '-x', str(x), '-y', str(y), '-z', str(z + 0.005),
        ], check=True, timeout=15.0)
        self.wait_sim(0.5)

    def cube_at(self, target):
        cube = self.read_cube()
        if cube is None:
            raise RuntimeError(f'Gazebo has no model named {self.box_name}. Use --spawn to add the demo cube.')
        position, _ = cube
        if math.dist(position, target) > 0.008:
            coordinates = ', '.join(f'{value:.3f}' for value in position)
            raise RuntimeError(f'The cube is at ({coordinates}) m, more than 8 mm from the supplied point.')
        return cube

    def pick(self, target, spawn, on_grasp=None):
        self.ready()
        if spawn:
            self.spawn_cube(target)
        _, yaw = self.cube_at(target)
        # Rotate the downward tool about world Z to line up with the sides of the cube.
        orientation = Quaternion(x=math.cos(yaw / 2), y=math.sin(yaw / 2), w=0.0)
        x, y, z = target
        above = (x, y, z + LIFT_HEIGHT)

        print('Opening and approaching the cube.', flush=True)
        self.open_gripper()
        self.approach(above, orientation)
        print('Lowering the gripper.', flush=True)
        self.move_straight(target, orientation, 'Descend')
        before, _ = self.cube_at(target)

        print('Closing and lifting.', flush=True)
        self.gripper.publish(Float64MultiArray(data=[CLOSE_FORCE, CLOSE_FORCE]))
        self.wait_sim(0.5)
        if on_grasp is not None:
            on_grasp()
        self.move_straight(above, orientation, 'Lift')
        self.wait_sim(0.6)
        lifted = self.read_cube()
        if lifted is None or math.dist(lifted[0], above) > 0.025:
            raise RuntimeError('The cube did not stay between the fingers after the lift.')
        print(f'Picked up {self.box_name} and lifted it {lifted[0][2] - before[2]:.3f} m.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('z', type=float)
    parser.add_argument('--spawn', action='store_true', help='Place the demo cube on the floor first')
    args = parser.parse_args()
    target = (args.x, args.y, args.z)
    if not all(math.isfinite(value) for value in target) or args.z < BOX_HALF_HEIGHT - 0.001:
        parser.error('Enter finite XYZ coordinates with Z at least 0.024 m.')
    if args.spawn and not math.isclose(args.z, BOX_HALF_HEIGHT, abs_tol=0.0001):
        parser.error('--spawn places the cube on the floor. Use Z = 0.025 m.')

    with open('/tmp/ur5e_pickup.lock', 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('Error: another pickup is running.', file=sys.stderr)
            return 1

        rclpy.init()
        node = Pickup()
        try:
            node.pick(target, args.spawn)
        except (RuntimeError, subprocess.SubprocessError) as error:
            print(f'Error: {error}', file=sys.stderr)
            return 1
        except KeyboardInterrupt:
            return 130
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
