# OmniPlan Dockerfile
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO} AS deps

# coinor-libcbc is needed by the bundled Colin and OPTIC planners. The package
# name differs by Ubuntu release: 24.04 (Jazzy/Kilted/Lyrical) ships
# coinor-libcbc3.1 while 20.04/22.04 (Foxy/Galactic/Humble/Iron) ship
# coinor-libcbc3. python3-vcstool provides the `vcs` command used below.
ARG ROS_DISTRO
RUN apt-get update \
    && apt-get install -y --no-install-recommends libz3-dev python3-vcstool \
    && case "${ROS_DISTRO}" in \
         foxy|galactic|humble|iron) \
           apt-get install -y --no-install-recommends coinor-libcbc3 ;; \
         *) \
           apt-get install -y --no-install-recommends coinor-libcbc3.1 ;; \
       esac \
    && rm -rf /var/lib/apt/lists/*

# The bundled Colin/OPTIC binaries link against libCbc.so.3, but some
# coinor-libcbc packages only install the fully versioned shared object.
# Resolve the host multiarch directory instead of hardcoding x86_64.
RUN MULTIARCH="$(uname -m)-linux-gnu" \
    && if [ ! -e "/usr/lib/${MULTIARCH}/libCbc.so.3" ]; then \
         CBC_LIB="$(find /usr/lib -name 'libCbc.so.3.*' -print -quit)"; \
         if [ -n "${CBC_LIB}" ]; then \
           ln -s "${CBC_LIB}" "/usr/lib/${MULTIARCH}/libCbc.so.3"; \
         else \
           echo "WARNING: libCbc.so.3.* not found; Colin/OPTIC may not run" >&2; \
         fi; \
       fi

# Create workspace and copy OmniPlan source code
WORKDIR /root/ros2_ws
SHELL ["/bin/bash", "-c"]
RUN mkdir -p src
COPY . /root/ros2_ws/src/omni_plan

# Import dependencies and install ROS 2 package dependencies. The APT package
# lists were removed in the previous layer, so refresh them here before rosdep
# installs anything, and clean them again afterwards.
RUN apt-get update \
    && vcs import src < src/omni_plan/dependencies.repos \
    && rosdep update --include-eol-distros \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace
FROM deps AS builder
ARG ROS_DISTRO
ARG CMAKE_BUILD_TYPE=Release
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --symlink-install \
       --cmake-args -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"

# Source the ROS 2 workspace for interactive shells
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Run a default command, e.g., starting a bash shell
CMD ["bash"]
