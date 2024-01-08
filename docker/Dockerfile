# Authoer: Jinwon Kim
# Email: mqjinwon@gmail.com

# Use the official OSRF ROS Noetic image as the base image
FROM osrf/ros:noetic-desktop-full

# Install additional dependencies
RUN apt-get -y update && \
    apt-get install -y --no-install-recommends libomp-dev libpcl-dev libeigen3-dev && \
    apt-get install -y python3-catkin-tools git

# Create a working directory (you can customize the path)
WORKDIR /root/workspace

# Load ROS environment at each run
COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# Set the entry point (customize as needed)
CMD ["/bin/bash"]