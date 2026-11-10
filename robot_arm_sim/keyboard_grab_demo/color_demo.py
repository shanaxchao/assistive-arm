#!/usr/bin/env python3
"""Interactive pick-and-return demo for three colored Gazebo cubes."""

import argparse
import fcntl
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from pick_box import Pickup, LIFT_HEIGHT

# Used only when spawning. Every selection reads the live Gazebo pose.
CUBES = {
    'red': ((0.45, 0.20, 0.03), '0.9 0.05 0.05 1'),
    'green': ((0.35, -0.28, 0.03), '0.05 0.8 0.1 1'),
    'blue': ((0.55, -0.02, 0.03), '0.05 0.15 0.95 1'),
}
READY = (0.40, 0.0, 0.35)


class ColorDemo(Pickup):
    def __init__(self):
        super().__init__()
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

    def spawn_scene(self):
        self.ready()
        for color, (position, rgba) in CUBES.items():
            self.box_name = color + '_cube'
            if self.read_cube() is not None:
                continue
            model = ET.parse(Path(__file__).with_name('pickup_cube.sdf')).getroot()
            model.find('model').set('name', self.box_name)
            for tag in ('ambient', 'diffuse'):
                model.find(f'.//material/{tag}').text = rgba
            subprocess.run([
                'ros2', 'run', 'ros_gz_sim', 'create', '-world', 'empty',
                '-name', self.box_name, '-string', ET.tostring(model, encoding='unicode'),
                '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
            ], check=True, timeout=15)
        self.wait_sim(0.5)
        print('Red, green, and blue cubes are ready.', flush=True)

    def update_obstacles(self, selected=None):
        """Refresh the other cubes as obstacles before planning an approach."""
        if not self.scene_client.wait_for_service(timeout_sec=5):
            raise RuntimeError('MoveIt planning scene is unavailable.')
        scene = PlanningScene(is_diff=True)
        previous_name = self.box_name
        try:
            for color in CUBES:
                self.box_name = color + '_cube'
                cube = self.read_cube()
                obj = CollisionObject(id=self.box_name)
                obj.header.frame_id = 'base_link'
                obj.operation = CollisionObject.REMOVE
                if cube is not None and color != selected:
                    position, yaw = cube
                    obj.operation = CollisionObject.ADD
                    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.05]*3)]
                    obj.primitive_poses = [Pose(
                        position=Point(x=position[0], y=position[1], z=position[2]),
                        orientation=Quaternion(z=math.sin(yaw/2), w=math.cos(yaw/2)),
                    )]
                scene.world.collision_objects.append(obj)
            future = self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene))
            self.wait_for(future.done, 'Updating collision objects timed out.')
            if not future.result().success:
                raise RuntimeError('MoveIt rejected the scene update.')
        finally:
            self.box_name = previous_name

    def payload(self, attach):
        """Tell MoveIt which geometry travels with the fingers."""
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        obj = CollisionObject(id=self.box_name)
        obj.operation = CollisionObject.REMOVE
        if attach:
            cube = self.read_cube()
            if cube is None:
                raise RuntimeError('Cannot locate the grasped cube.')
            position, yaw = cube
            obj.operation = CollisionObject.ADD
            obj.header.frame_id = 'base_link'
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.05]*3)]
            obj.primitive_poses = [Pose(
                position=Point(x=position[0], y=position[1], z=position[2]),
                orientation=Quaternion(z=math.sin(yaw/2), w=math.cos(yaw/2)),
            )]
        scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
            link_name='pickup_center', object=obj,
            touch_links=['pickup_left_pad', 'pickup_right_pad', 'pickup_palm', 'ground_plane'],
        )]
        future = self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene))
        self.wait_for(future.done, 'Payload update timed out.')
        if not future.result().success:
            raise RuntimeError('MoveIt rejected the payload update.')

    def cycle(self, color):
        self.ready()
        self.box_name = color + '_cube'
        original = self.read_cube()
        if original is None:
            raise RuntimeError(f'{self.box_name} is missing. Restart the color demo launch.')
        target, yaw = original
        orientation = Quaternion(x=math.cos(yaw/2), y=math.sin(yaw/2), w=0.0)
        self.update_obstacles()
        self.update_obstacles(selected=color)
        self.pick(target, spawn=False, on_grasp=lambda: self.payload(True))
        print('Holding for two seconds.', flush=True)
        self.wait_sim(2.0)
        held = self.read_cube()
        if held is None:
            raise RuntimeError('The selected cube disappeared.')
        above = (target[0], target[1], target[2] + LIFT_HEIGHT)
        if math.dist(held[0], above) > 0.025:
            raise RuntimeError('The cube slipped during the hold.')
        # Compensate for the measured offset between cube and finger center.
        # The tool keeps the same orientation during this vertical return.
        return_target = tuple(target[i] + above[i] - held[0][i] for i in range(3))
        print(f'Returning {color} to its original spot.', flush=True)
        self.move_straight(return_target, orientation, 'Return')
        self.open_gripper()
        self.payload(False)
        self.wait_sim(0.5)
        retreat = (return_target[0], return_target[1], return_target[2] + LIFT_HEIGHT)
        self.move_straight(retreat, orientation, 'Retreat')
        self.wait_sim(0.5)
        returned = self.read_cube()
        if returned is None:
            raise RuntimeError('Could not verify the returned cube.')
        error = math.dist(returned[0], target)
        yaw_error = abs(math.remainder(returned[1] - yaw, math.pi/2))
        self.update_obstacles()
        self.approach(READY, Quaternion(x=1.0, w=0.0))
        print(f'{color}: return error {error*1000:.1f} mm, orientation error '
              f'{math.degrees(yaw_error):.1f} degrees.', flush=True)
        if error > 0.015 or yaw_error > math.radians(10):
            raise RuntimeError('Return exceeded the demo tolerance (15 mm / 10 degrees).')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spawn-only', action='store_true')
    parser.add_argument('--sequence', nargs='+', choices=list(CUBES),
                        help='Run a fixed sequence for repeatability testing')
    args = parser.parse_args()
    with open('/tmp/ur5e_pickup.lock', 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('Another pickup or scene setup is running. Try again when it finishes.')
            return 1
        rclpy.init()
        node = ColorDemo()
        try:
            if args.spawn_only:
                node.spawn_scene()
                return 0
            if args.sequence:
                for color in args.sequence:
                    node.cycle(color)
                return 0
            while True:
                color = input('Choose a cube [red / green / blue / quit]: ').strip().lower()
                if color == 'quit':
                    return 0
                if color not in CUBES:
                    print('Enter red, green, blue, or quit.')
                    continue
                node.cycle(color)
                print('Ready for another selection.')
        except (RuntimeError, subprocess.SubprocessError) as error:
            print(f'Demo stopped: {error}\nInspect the scene before restarting the script.', file=sys.stderr)
            return 1
        except (EOFError, KeyboardInterrupt):
            return 0
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
