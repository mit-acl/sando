FROM ros:humble

LABEL Maintainer="Anonymous" \
      Description="Ubuntu 22.04, ROS2 Humble for SANDO"

ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

# System packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -q -y --no-install-recommends \
        git tmux vim wget tmuxp make gdb && \
    rm -rf /var/lib/apt/lists/*

# ROS 2 dependencies
RUN apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-gazebo-ros-pkgs \
        ros-${ROS_DISTRO}-pcl-conversions \
        ros-${ROS_DISTRO}-example-interfaces \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-rviz-common \
        ros-${ROS_DISTRO}-rqt-gui \
        ros-${ROS_DISTRO}-rqt-gui-py \
        ros-${ROS_DISTRO}-tf2-tools \
        ros-${ROS_DISTRO}-tf-transformations \
        ros-${ROS_DISTRO}-desktop \
        ros-dev-tools \
        ros-${ROS_DISTRO}-gazebo-ros2-control \
        ros-${ROS_DISTRO}-xacro \
        libpcl-dev build-essential nlohmann-json3-dev && \
    rm -rf /var/lib/apt/lists/*

# Gurobi optimizer
RUN wget -q https://packages.gurobi.com/11.0/gurobi11.0.3_linux64.tar.gz -P /tmp && \
    tar -xzf /tmp/gurobi11.0.3_linux64.tar.gz -C /opt && \
    rm /tmp/gurobi11.0.3_linux64.tar.gz && \
    cd /opt/gurobi1103/linux64/src/build && make && cp libgurobi_c++.a ../../lib/
ENV GUROBI_HOME="/opt/gurobi1103/linux64"
ENV PATH="${PATH}:${GUROBI_HOME}/bin"
ENV LD_LIBRARY_PATH="${GUROBI_HOME}/lib"
COPY gurobi.lic /opt/gurobi1103/

# Clone SANDO with all submodules
ENV SANDO_WS=/root/sando_ws
ENV LIVOX_WS=/root/livox_ws
RUN mkdir -p ${SANDO_WS}/src
WORKDIR ${SANDO_WS}/src
ARG DUMMY=unknown
RUN DUMMY=${DUMMY} git clone --recursive <ANONYMIZED_REPO_URL>

# Symlink deps for colcon package discovery (skip Livox-SDK2 and livox_ros_driver2 — built separately)
WORKDIR ${SANDO_WS}/src
RUN for d in sando/deps/*/; do \
      name=$(basename "$d"); \
      case "$name" in Livox-SDK2|livox_ros_driver2) continue;; esac; \
      [ ! -e "$name" ] && ln -s "$d" "$name"; \
    done

# Build Livox-SDK2 (cmake install to /usr/local)
WORKDIR ${SANDO_WS}/src/sando/deps/Livox-SDK2
RUN mkdir -p build && cd build && cmake .. && make -j && make install

# Build livox_ros_driver2 (in its own workspace — requires build.sh for package file setup)
RUN mkdir -p ${LIVOX_WS}/src
WORKDIR ${LIVOX_WS}/src
RUN cp -r ${SANDO_WS}/src/sando/deps/livox_ros_driver2 livox_ros_driver2 && \
    cd livox_ros_driver2 && \
    rm -f COLCON_IGNORE && \
    cp -f package_ROS2.xml package.xml && \
    cp -rf launch_ROS2/ launch/
SHELL ["/bin/bash", "-c"]
WORKDIR ${LIVOX_WS}
RUN source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
                              -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble

# Build all colcon packages (three passes: decomp_util, main, ros2_livox_simulation)
WORKDIR ${SANDO_WS}
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select decomp_util --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN source /opt/ros/humble/setup.bash && \
    source ${SANDO_WS}/install/setup.bash && \
    colcon build --packages-ignore ros2_livox_simulation \
                 --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN source /opt/ros/humble/setup.bash && \
    source ${SANDO_WS}/install/setup.bash && \
    source ${LIVOX_WS}/install/setup.bash && \
    colcon build --packages-select ros2_livox_simulation \
                 --cmake-args -DCMAKE_BUILD_TYPE=Release

# Gazebo audio fix
RUN echo 'pcm.!default { type plug; slave.pcm "null" }' >> /usr/share/alsa/alsa.conf

# Livox library path
ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

# Shell aliases
RUN echo "alias tks='tmux kill-server'" >> ~/.bashrc && \
    echo "alias roscd='cd ${SANDO_WS}'" >> ~/.bashrc && \
    echo "alias ss='source ${SANDO_WS}/install/setup.bash'" >> ~/.bashrc && \
    echo "alias cbd='cd ${SANDO_WS} && colcon build --packages-select sando && ss'" >> ~/.bashrc && \
    echo "alias cbps='cd ${SANDO_WS} && colcon build --packages-select'" >> ~/.bashrc

# Entry point
COPY sando.sh /
RUN chmod +x /sando.sh
ENTRYPOINT ["/sando.sh"]
