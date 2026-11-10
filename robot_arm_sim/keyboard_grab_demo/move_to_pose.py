#!/usr/bin/env python3
"""Plan a UR5e tool pose in the running Gazebo/MoveIt simulation."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener


def current_tool_transform(node):
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            return buffer.lookup_transform('base_link', 'tool0', Time())
        except TransformException:
            pass
    raise RuntimeError('No base_link -> tool0 transform; is the UR5e running?')


def make_goal(position, orientation, args):
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = 'base_link'
    position_constraint.link_name = 'tool0'
    position_constraint.weight = 1.0

    region = SolidPrimitive()
    region.type = SolidPrimitive.SPHERE
    region.dimensions = [args.position_tolerance]
    region_pose = Pose()
    region_pose.position.x, region_pose.position.y, region_pose.position.z = position
    region_pose.orientation.w = 1.0
    position_constraint.constraint_region.primitives.append(region)
    position_constraint.constraint_region.primitive_poses.append(region_pose)

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = 'base_link'
    orientation_constraint.link_name = 'tool0'
    orientation_constraint.orientation = orientation
    orientation_constraint.absolute_x_axis_tolerance = args.orientation_tolerance
    orientation_constraint.absolute_y_axis_tolerance = args.orientation_tolerance
    orientation_constraint.absolute_z_axis_tolerance = args.orientation_tolerance
    orientation_constraint.weight = 1.0

    goal = MoveGroup.Goal()
    goal.request.group_name = 'ur_manipulator'
    goal.request.pipeline_id = 'ompl'
    goal.request.num_planning_attempts = args.attempts
    goal.request.allowed_planning_time = args.planning_time
    goal.request.max_velocity_scaling_factor = args.velocity_scale
    goal.request.max_acceleration_scaling_factor = args.acceleration_scale
    goal.request.start_state.is_diff = True  # Start from the current joint state.
    constraints = Constraints(position_constraints=[position_constraint])
    if not args.position_only:
        constraints.orientation_constraints = [orientation_constraint]
    goal.request.goal_constraints = [constraints]
    goal.planning_options.plan_only = not args.execute
    return goal


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x', type=float, help='Tool X in metres relative to base_link')
    parser.add_argument('y', type=float, help='Tool Y in metres relative to base_link')
    parser.add_argument('z', type=float, help='Tool Z in metres relative to base_link')
    parser.add_argument('--relative', action='store_true',
                        help='Treat x y z as offsets from the current tool position')
    parser.add_argument('--quaternion', type=float, nargs=4, metavar=('QX', 'QY', 'QZ', 'QW'),
                        help='Target orientation; defaults to the current tool orientation')
    parser.add_argument('--position-only', action='store_true',
                        help='Allow any tool orientation at the target XYZ')
    parser.add_argument('--position-tolerance', type=float, default=0.01, metavar='M',
                        help='Allowed tool position error in metres (default: 0.01)')
    parser.add_argument('--orientation-tolerance', type=float, default=0.1, metavar='RAD',
                        help='Allowed orientation error in radians (default: 0.1)')
    parser.add_argument('--planning-time', type=float, default=5.0, metavar='SEC',
                        help='Maximum planning time in seconds (default: 5)')
    parser.add_argument('--attempts', type=int, default=10,
                        help='Number of planning attempts (default: 10)')
    parser.add_argument('--velocity-scale', type=float, default=0.1, metavar='FRACTION',
                        help='Fraction of maximum joint velocity (default: 0.1)')
    parser.add_argument('--acceleration-scale', type=float, default=0.1, metavar='FRACTION',
                        help='Fraction of maximum joint acceleration (default: 0.1)')
    parser.add_argument('--execute', action='store_true',
                        help='Execute the plan in Gazebo after planning')
    args = parser.parse_args()

    if not all(math.isfinite(value) for value in (args.x, args.y, args.z)):
        parser.error('Position values must be finite')
    if args.quaternion:
        norm = math.sqrt(sum(value * value for value in args.quaternion))
        if not math.isfinite(norm) or norm == 0:
            parser.error('Quaternion must have finite, nonzero length')
    if args.position_only and args.quaternion:
        parser.error('--position-only cannot be combined with --quaternion')
    for name in ('position_tolerance', 'orientation_tolerance', 'planning_time'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f'--{name.replace("_", "-")} must be positive and finite')
    if args.attempts < 1:
        parser.error('--attempts must be at least 1')
    for name in ('velocity_scale', 'acceleration_scale'):
        value = getattr(args, name)
        if not math.isfinite(value) or not 0 < value <= 1:
            parser.error(f'--{name.replace("_", "-")} must be between 0 and 1')

    rclpy.init()
    node = Node('ur5e_pose_goal')
    try:
        transform = current_tool_transform(node)
        current = transform.transform.translation
        position = (args.x, args.y, args.z)
        if args.relative:
            position = (current.x + args.x, current.y + args.y, current.z + args.z)
        orientation = transform.transform.rotation
        if args.quaternion:
            orientation.x, orientation.y, orientation.z, orientation.w = (
                value / norm for value in args.quaternion
            )

        print(f'Current tool0: ({current.x:.3f}, {current.y:.3f}, {current.z:.3f}) m')
        print(f'Target tool0:  ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) m')
        print('Mode:', 'plan and execute' if args.execute else 'plan only')
        print('Goal:', 'position only' if args.position_only else 'position and orientation')

        client = ActionClient(node, MoveGroup, '/move_action')
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('MoveIt /move_action is unavailable; launch ur_sim_moveit first')
        send_future = client.send_goal_async(make_goal(position, orientation, args))
        rclpy.spin_until_future_complete(node, send_future)
        handle = send_future.result()
        if not handle.accepted:
            raise RuntimeError('MoveIt rejected the goal')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result().result
        if result.error_code.val != result.error_code.SUCCESS:
            raise RuntimeError(f'Planning failed (MoveIt error {result.error_code.val})')
        print(f'Success: {len(result.planned_trajectory.joint_trajectory.points)} trajectory points')
    except RuntimeError as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
