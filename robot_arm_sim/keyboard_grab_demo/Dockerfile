FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    xvfb x11vnc novnc websockify openbox dbus-x11 \
    libgl1-mesa-dri libglx-mesa0 mesa-utils \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2-control \
    ros-jazzy-ur-simulation-gz \
    && rm -rf /var/lib/apt/lists/*

# The UR MoveIt launch references this package even when using simulated hardware.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ur-robot-driver \
    && rm -rf /var/lib/apt/lists/*

ENV DISPLAY=:99 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1

COPY start-gazebo /usr/local/bin/start-gazebo
RUN chmod +x /usr/local/bin/start-gazebo

CMD ["/usr/local/bin/start-gazebo"]
