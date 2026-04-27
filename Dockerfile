# OmniPlan Dockerfile
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO} AS deps

# Create workspace
WORKDIR /root/ros2_ws
SHELL ["/bin/bash", "-c"]
RUN mkdir -p src

# Copy OmniPlan source code
COPY . /root/ros2_ws/src/omni_plan

# Colin and OPTIC dependency symlink
RUN apt update && apt install libz3-dev coinor-libcbc3 -y

# Import dependencies
RUN vcs import src < src/omni_plan/dependencies.repos

# Install ROS 2 and package dependencies
RUN rosdep update --include-eol-distros && rosdep install --from-paths src --ignore-src -r -y

# Build the ws (colcon)
FROM deps AS builder
ARG CMAKE_BUILD_TYPE=Release
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build

# Source the ROS 2 setup file
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Run a default command, e.g., starting a bash shell
CMD ["bash"]