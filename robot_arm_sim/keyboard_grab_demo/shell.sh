#!/usr/bin/env bash
set -euo pipefail
docker exec -it senior-design-ros2-gazebo bash -c 'source /opt/ros/jazzy/setup.bash && exec bash'
