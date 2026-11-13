# Keyboard grab demo

This demo runs ROS 2 Jazzy, MoveIt 2, Gazebo Harmonic, a UR5e arm, and a
two-finger gripper in Docker. Select red, green, or blue to pick up that cube
and return it to its current starting position.

## First-time setup

Install Git and Docker Desktop, and start Docker Desktop. Linux users can also
use Docker Engine with permission to run Docker commands. The scripts require
Bash. On Windows, use WSL 2 with Docker Desktop integration enabled.
Internet access is required for the initial image build. Allow sufficient disk
space and memory for a ROS/Gazebo desktop image. This demo has been tested on
Apple Silicon. Intel/AMD and Windows WSL runs have not been verified.

The Dockerfile installs the ROS, Gazebo, controller, Python, and browser desktop
dependencies. Host ROS installation, a Python virtual environment, and a colcon
build are unnecessary for this demo.

```bash
git clone https://github.com/shanaxchao/assistive-arm.git
cd assistive-arm/robot_arm_sim/keyboard_grab_demo
./start.sh
./shell.sh
```

Inside that shell, launch the three-cube scene.

```bash
ros2 launch /workspace/ros2-gazebo/color_demo.launch.py
```

Open [the simulation desktop](http://localhost:6080/vnc.html?autoconnect=true&resize=scale)
and wait for "Red, green, and blue cubes are ready."

In a second host terminal, enter the same cloned demo directory and run
`./shell.sh`. Inside that shell, run:

```bash
python3 /workspace/ros2-gazebo/color_demo.py
```

Enter `red`, `green`, `blue`, or `quit`. Stop the launch with Ctrl+C and
launch again to reset. Run `./stop.sh` from the host to stop the container.

Only this demo directory is mounted, at `/workspace/ros2-gazebo`.
This stable container path preserves the commands regardless of clone location.

When migrating from the old standalone folder, run `./stop.sh` followed by
`./start.sh` here to recreate the container with the new mount.
The first start builds the image automatically and may take several minutes.
Later starts reuse it. Port 6080 must be available.

## Other simulation commands

1. Start Docker Desktop.
2. Run `./start.sh` from this directory.
3. Open <http://localhost:6080/vnc.html?autoconnect=true&resize=scale>.
4. Run `./shell.sh` in a terminal for a ROS 2 shell.
5. In that shell, start the UR5e simulation:

   ```bash
   ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e
   ```

   This launch starts Gazebo and the robot controllers. The container starts
   only the graphical desktop, so do not start a second Gazebo process.
6. Stop the launch with Ctrl+C before trying the MoveIt version:

   ```bash
   ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
   ```

   MoveIt opens RViz, where you can plan and execute end-effector motions.
7. Run `./stop.sh` when finished.

In another ROS 2 shell, use `ros2 control list_controllers` and
`ros2 topic echo /joint_states --once` to inspect the running robot.

### Move the UR5e tool from Python

With `ur_sim_moveit.launch.py` running, open another `./shell.sh` session. The
script plans a pose for `tool0` in metres relative to `base_link`. It keeps the
tool's current orientation unless `--quaternion QX QY QZ QW` is supplied.

```bash
# Plan a target 2 cm in the base X direction from the current tool pose.
python3 /workspace/ros2-gazebo/move_to_pose.py 0.02 0 0 --relative

# Plan and move the simulated arm to that target.
python3 /workspace/ros2-gazebo/move_to_pose.py 0.02 0 0 --relative --execute

# For an absolute base-frame XYZ target, omit --relative.
python3 /workspace/ros2-gazebo/move_to_pose.py X Y Z --execute

# If XYZ matters more than tool orientation, remove the orientation constraint.
python3 /workspace/ros2-gazebo/move_to_pose.py X Y Z --position-only

# Give a hard but reachable goal more planning time and a wider target region.
python3 /workspace/ros2-gazebo/move_to_pose.py X Y Z \
  --planning-time 15 --position-tolerance 0.03 --orientation-tolerance 0.3
```

Without `--execute`, the script only plans and reports the result. It uses the
installed MoveIt action server, so no additional Python package is needed.
Run `python3 /workspace/ros2-gazebo/move_to_pose.py --help` for the other
settings, including planning attempts and velocity/acceleration scales. The
speed scales change trajectory timing; they do not make an unreachable goal
reachable.

### Pick up a cube by its coordinates

Stop the previous simulation launch with Ctrl+C. Inside a `./shell.sh` session,
start the pickup scene and leave it running.

```bash
ros2 launch /workspace/ros2-gazebo/pickup.launch.py
```

In a second terminal, run `./shell.sh` again and enter the cube's center
coordinates in metres. The world origin is at the arm's base.

```bash
python3 /workspace/ros2-gazebo/pick_box.py 0.45 0.20 0.025 --spawn
```

`--spawn` places the demo's 5 cm cube on the floor, where its center is at
`Z = 0.025`. To pick up an existing 5 cm cube named `pick_cube`, omit
`--spawn` and supply its current center. You can look up that center with
`gz model -m pick_cube -p`.

The arm approaches from above, closes the fingers, and lifts. The script checks
the cube's position in Gazebo after lifting. The gripper keeps holding the cube
after the script exits. To open the fingers and drop the cube, run:

```bash
ros2 topic pub --once -w 1 /pickup_gripper/commands std_msgs/msg/Float64MultiArray '{data: [-2.0, -2.0]}'
```

Restart the pickup launch for a fresh scene. Use a clear workspace with enough
room for the gripper to descend and lift vertically.

The first start builds the image; later starts reuse it.
To rebuild after editing the Dockerfile, run
`docker build -t senior-design-ros2-gazebo:jazzy-harmonic .` from this directory.

The browser view uses software rendering. It may be slower than a native GPU
window. The browser service listens on `127.0.0.1:6080` only.

### Interactive red green and blue demo

Stop the previous launch with Ctrl+C, then run inside the container.

```bash
ros2 launch /workspace/ros2-gazebo/color_demo.launch.py
```

Wait for "Red, green, and blue cubes are ready." In another `./shell.sh`
session, start the interactive script.

```bash
python3 /workspace/ros2-gazebo/color_demo.py
```

Enter `red`, `green`, or `blue`. The arm picks up that cube, holds it for
two seconds, returns it to the position saved before pickup, opens the fingers,
and returns to a ready pose. Enter `quit` to exit.

Only initial spawn positions are fixed. Each selection reads the cube's live
Gazebo position and yaw. Keep cubes upright on the floor, reachable, and separated
enough for the open gripper. The demo uses Gazebo model names to identify colors.
Return position and orientation errors are reported after settling, with a
15 mm and 10 degree acceptance tolerance. A failed cycle ends the script for
inspection. Restart the launch for a fresh scene.

The launch reuses `pickup.launch.py` and runs `color_demo.py --spawn-only`.
The `CUBES` dictionary defines spawn positions and colors. `ColorDemo.cycle`
coordinates selection, grasp, hold, return, release, verification, and retreat.
The inherited `Pickup` class in `pick_box.py` handles live pose queries, MoveIt
planning, straight-line motions, and gripper commands. Its default model name
remains `pick_cube` for the original single-cube CLI.

For a repeatable test, run
`python3 /workspace/ros2-gazebo/color_demo.py --sequence red green blue`.

Before each cycle, the script refreshes the other cubes in MoveIt's collision
scene. During the lift and return, it represents the grasped cube as attached
geometry so its size is included in planning. Gazebo's finger contact and
friction hold the physical cube. The selected cube is temporarily removed from
the world obstacle list during its approach and restored after placement.

The demo assumes upright 5 cm cubes. It compensates for the measured offset
between the held cube and the finger center when placing the cube back.

### Control the demo by voice

Keep the color demo launch running in the container. Speech recognition runs
on the host, where the microphone is available; each accepted command starts
one color cycle in the running container. Install PortAudio on the host if
needed, then install the Python packages:

```bash
brew install portaudio
python3 -m pip install openai-whisper PyAudio
```

From the repository root, start the listener:

```bash
python3 transcriber.py
```

Speak a command such as "pick up the red cube", "grab the green block", or
"lift the blue box". The parser rejects negated commands, unsupported or
ambiguous colors, and phrases without both a grab action and an object.
Whisper downloads its `tiny` model the first time it runs. Press Ctrl+C to
stop listening.
