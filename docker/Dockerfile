FROM ros:humble

LABEL Maintainer="Kota Kondo <kkondo@mit.edu>" \
      Description="Ubuntu 22.04, ROS2 humble for SANDO"

# Set environment variables
##########################################
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

# Usuals
##########################################
RUN rm -rf /var/lib/apt/lists/*
RUN apt update
RUN apt upgrade -y
RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -q -y --no-install-recommends git tmux vim wget tmuxp make gdb

# Making directories and setting up the environment
##########################################
WORKDIR /home/kkondo
RUN mkdir code

# Install dependencies
##########################################
RUN apt-get install -y  ros-${ROS_DISTRO}-gazebo-* \
                        ros-${ROS_DISTRO}-pcl-conversions \
                        ros-${ROS_DISTRO}-example-interfaces \
                        ros-${ROS_DISTRO}-pcl-ros \
                        ros-${ROS_DISTRO}-rviz2 \
                        ros-${ROS_DISTRO}-rqt-gui \
                        ros-${ROS_DISTRO}-rqt-gui-py \
                        ros-${ROS_DISTRO}-tf2-tools \
                        ros-${ROS_DISTRO}-tf-transformations

RUN apt install -y  ros-${ROS_DISTRO}-desktop \
                        ros-dev-tools \
                        ros-${ROS_DISTRO}-turtlesim \
                        ros-${ROS_DISTRO}-rqt* \
                        ros-${ROS_DISTRO}-rviz2 \
                        ros-${ROS_DISTRO}-rviz-common \ 
                        libpcl-dev \
                        build-essential \
                        nlohmann-json3-dev

# Gurobi optimizer
##########################################
RUN wget -q https://packages.gurobi.com/11.0/gurobi11.0.3_linux64.tar.gz -P /tmp && \
    tar -xzf /tmp/gurobi11.0.3_linux64.tar.gz -C /opt && \
    rm /tmp/gurobi11.0.3_linux64.tar.gz
WORKDIR /opt/gurobi1103/linux64/src/build
RUN make && cp libgurobi_c++.a ../../lib/
ENV GUROBI_HOME="/opt/gurobi1103/linux64"
ENV PATH="${PATH}:${GUROBI_HOME}/bin"
ENV LD_LIBRARY_PATH="${GUROBI_HOME}/lib"
COPY gurobi.lic /opt/gurobi1103/

# Install SANDO and dependencies
##########################################
RUN mkdir -p /home/kkondo/code/sando_ws/src
WORKDIR /home/kkondo/code/sando_ws/src
# dummy arg to invalidate cache and build from here
ARG DUMMY=unknown
RUN DUMMY=${DUMMY} git clone https://github.com/mit-acl/sando.git
WORKDIR /home/kkondo/code/sando_ws/src/sando
RUN git switch dev && git checkout d0fafa56d11aa0cc21c4f6710adb15603f442776
WORKDIR /home/kkondo/code/sando_ws/src
RUN git clone https://github.com/kotakondo/dynus_interfaces.git && \
    cd dynus_interfaces && git checkout 24113061479bbdc2661fcc413a9885aebae56076 && \
    printf '\nament_package()\n' >> CMakeLists.txt
RUN git clone https://github.com/kotakondo/realsense_gazebo_plugin.git && \
    cd realsense_gazebo_plugin && git checkout 8c357e69da2160f54328db70915bd9b28eb975bf
RUN git clone https://github.com/kotakondo/livox_laser_simulation_ros2.git && \
    cd livox_laser_simulation_ros2 && git checkout 246040d802ec2d7374f38ffd07e5e7fc49e297fb
RUN git clone https://github.com/kotakondo/gazebo_ros_pkgs.git && \
    cd gazebo_ros_pkgs && git checkout ed1f7b886971d7bf898e6e0f59c622df929c0521
RUN git clone https://github.com/kotakondo/uav_simulator.git && \
    cd uav_simulator && git checkout b196208d3cbf0269b8e80220f7cc1cc992b6c7f8
RUN git clone https://gitlab.com/mit-acl/lab/acl-mapping.git
WORKDIR /home/kkondo/code/sando_ws/src/acl-mapping
RUN git switch ros2 && git checkout d36a5338c2eecfdbbbcf17870994e9310f12b412
RUN mkdir -p /home/kkondo/code/decomp_ws/src
WORKDIR /home/kkondo/code/decomp_ws/src
RUN git clone https://github.com/kotakondo/DecompROS2.git && \
    cd DecompROS2 && git checkout a2bc42ed46bf934323994706f17af8a3883860ea
RUN mkdir -p /home/kkondo/code/livox_ws/src
WORKDIR /home/kkondo/code/livox_ws/src
RUN git clone https://github.com/kotakondo/livox_ros_driver2.git && \
    cd livox_ros_driver2 && git checkout 274aad204b5dfce15c648e88dfd375fa5d7b3026
WORKDIR /home/kkondo/code
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && git checkout 6a940156dd7151c3ab6a52442d86bc83613bd11b

# Build the workspace
##########################################
### for decomp
WORKDIR /home/kkondo/code/decomp_ws
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select decomp_util colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN . /home/kkondo/code/decomp_ws/install/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

### for Livox-SDK2
WORKDIR /home/kkondo/code/Livox-SDK2
RUN mkdir build
WORKDIR /home/kkondo/code/Livox-SDK2/build
RUN cmake .. && make -j && sudo make install

### for livox_ros_driver2
WORKDIR /home/kkondo/code/livox_ws/src/livox_ros_driver2
RUN . /opt/ros/humble/setup.sh && ./build.sh humble

### Use bash for ROS setup scripts
SHELL ["/bin/bash", "-c"]

### Build all packages
WORKDIR /home/kkondo/code/sando_ws
RUN source /opt/ros/humble/setup.bash && \
    source /home/kkondo/code/decomp_ws/install/setup.bash && \
    export CMAKE_PREFIX_PATH=/home/kkondo/code/livox_ws/install/livox_ros_driver2:/home/kkondo/code/decomp_ws/install/decomp_util:${CMAKE_PREFIX_PATH} && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Gazebo
##########################################
### handle ALSA-related error
RUN echo    'pcm.!default { \
                  type plug \
                  slave.pcm "null" \
            }' >> /usr/share/alsa/alsa.conf

# For livox lidar
ENV LD_LIBRARY_PATH=/home/kkondo/code/livox_ws/install/livox_ros_driver2/lib:${LD_LIBRARY_PATH}:/usr/local/lib:/usr/local/include

# Alias
RUN echo 'alias tks="tmux kill-server"' >> ~/.bashrc
RUN echo 'alias sb="source ~/.bashrc"' >> ~/.bashrc
RUN echo 'alias eb="code ~/.bashrc"' >> ~/.bashrc
RUN echo 'alias gs="git status"' >> ~/.bashrc
RUN echo 'alias gp="git push"' >> ~/.bashrc
RUN echo 'alias roscd="cd ~/code/sando_ws"' >> ~/.bashrc
RUN echo 'alias cb="roscd && colcon build && sb"' >> ~/.bashrc
RUN echo 'alias ss="roscd && source install/setup.bash"' >> ~/.bashrc
RUN echo 'alias cbd="clear && roscd && colcon build && ss"' >> ~/.bashrc
RUN echo 'alias cbm="clear && roscd && colcon build --packages-select ros2_mapper && ss"' >> ~/.bashrc
RUN echo 'alias cbsl="roscd && colcon build --symlink-install && sb"' >> ~/.bashrc
RUN echo 'alias cbps="roscd && colcon build --packages-select"' >> ~/.bashrc
RUN echo 'alias tf_visualize="ros2 run rqt_tf_tree rqt_tf_tree"' >> ~/.bashrc
RUN echo 'alias tks="tmux kill-server"' >> ~/.bashrc

# Set up Entry Point
# ##########################################
COPY ./sando.sh /
RUN chmod +x /sando.sh
ENTRYPOINT ["/sando.sh"]
