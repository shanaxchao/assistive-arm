#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
image="senior-design-ros2-gazebo:jazzy-harmonic"
container="senior-design-ros2-gazebo"

if ! command -v docker >/dev/null 2>&1; then
  echo "Install Docker Desktop and start it before running this script." >&2
  exit 1
fi
if ! docker info >/dev/null 2>&1; then
  echo "Start Docker and wait for it to finish starting, then try again." >&2
  exit 1
fi

if docker ps --format '{{.Names}}' | grep -Fxq "$container"; then
  mounted_dir="$(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/workspace/ros2-gazebo"}}{{.Source}}{{end}}{{end}}' "$container")"
  if [[ "$mounted_dir" != "$script_dir" ]]; then
    echo "The running simulator uses another folder. Run ./stop.sh, then ./start.sh." >&2
    exit 1
  fi
fi

if ! docker image inspect "$image" >/dev/null 2>&1; then
  docker build -t "$image" "$script_dir"
fi

if ! docker ps --format '{{.Names}}' | grep -Fxq "$container"; then
  docker rm -f "$container" >/dev/null 2>&1 || true
  docker run -d --name "$container" \
    -p 127.0.0.1:6080:6080 \
    --mount "type=bind,source=$script_dir,target=/workspace/ros2-gazebo" -w /workspace/ros2-gazebo \
    "$image" >/dev/null
fi

printf 'Simulation desktop: http://localhost:6080/vnc.html?autoconnect=true&resize=scale\n'
printf 'ROS 2 shell: %s/shell.sh\n' "$script_dir"
