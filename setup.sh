#!/bin/bash
# --------------------------------------------------------------------------
# SANDO Setup Script
# Installs ROS 2 Humble, Gurobi, and all dependencies, then builds the workspace.
#
# Usage:
#   git clone --recursive https://github.com/mit-acl/sando.git
#   cd sando && ./setup.sh
# --------------------------------------------------------------------------
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SANDO_WS="$(cd "$SCRIPT_DIR/../.." 2>/dev/null && pwd)" || SANDO_WS="$HOME/code/sando_ws"

echo "=== SANDO Setup ==="
echo "  Workspace: $SANDO_WS"
echo "  Sando dir: $SCRIPT_DIR"
echo ""

# Prompt for sudo once (cached for 15 min)
sudo -v

# ============================================================
# 1. System packages
# ============================================================
echo "=== Installing system packages ==="
sudo apt update && sudo apt upgrade -y
sudo apt-get install -q -y --no-install-recommends \
    git tmux vim wget tmuxp make gdb openssh-server net-tools \
    g++ xterm python3-pip build-essential \
    libomp-dev libpcl-dev libeigen3-dev nlohmann-json3-dev

# ============================================================
# 2. ROS 2 Humble
# ============================================================
if ! command -v ros2 &>/dev/null; then
    echo "=== Installing ROS 2 Humble ==="
    sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    sudo apt install -y software-properties-common
    echo -e "\n" | sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-humble-desktop ros-dev-tools
    grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || \
        echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
else
    echo "=== ROS 2 already installed, skipping ==="
fi

export ROS_DISTRO=humble
source /opt/ros/humble/setup.bash

# ROS 2 dependencies
echo "=== Installing ROS 2 dependencies ==="
sudo apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-common \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-example-interfaces \
    ros-${ROS_DISTRO}-rqt-gui \
    ros-${ROS_DISTRO}-rqt-gui-py

# ============================================================
# 3. Gurobi (if not already installed)
# ============================================================
if [ ! -d "/opt/gurobi1103" ]; then
    echo "=== Installing Gurobi 11.0.3 ==="
    wget -q https://packages.gurobi.com/11.0/gurobi11.0.3_linux64.tar.gz -P /tmp
    sudo tar -xzf /tmp/gurobi11.0.3_linux64.tar.gz -C /opt
    rm /tmp/gurobi11.0.3_linux64.tar.gz
    cd /opt/gurobi1103/linux64/src/build
    sudo make && sudo cp libgurobi_c++.a ../../lib/
    echo ""
    echo "  NOTE: Place your Gurobi license file at ~/gurobi.lic"
    echo "  Free academic licenses: https://www.gurobi.com/academia/academic-program-and-licenses/"
    echo ""
fi

export GUROBI_HOME="/opt/gurobi1103/linux64"
export PATH="${PATH}:${GUROBI_HOME}/bin"
export LD_LIBRARY_PATH="${GUROBI_HOME}/lib:${LD_LIBRARY_PATH}"

# ============================================================
# 4. Initialize submodules (if not already done)
# ============================================================
echo "=== Initializing submodules ==="
cd "$SCRIPT_DIR"
git submodule update --init --recursive

# ============================================================
# 5. Create symlinks for colcon discovery
# ============================================================
echo "=== Setting up workspace symlinks ==="
WS_SRC="$SANDO_WS/src"
for d in "$SCRIPT_DIR"/deps/*/; do
    name=$(basename "$d")
    # Skip non-colcon deps (they have COLCON_IGNORE)
    if [ -f "$d/COLCON_IGNORE" ]; then
        continue
    fi
    if [ ! -e "$WS_SRC/$name" ]; then
        ln -s "$d" "$WS_SRC/$name"
        echo "  Linked: $name"
    fi
done

# ============================================================
# 6. Build Livox-SDK2 (cmake, not colcon)
# ============================================================
LIVOX_SDK="$SCRIPT_DIR/deps/Livox-SDK2"
if [ -d "$LIVOX_SDK" ] && [ ! -f "/usr/local/lib/liblivox_lidar_sdk_static.a" ]; then
    echo "=== Building Livox-SDK2 ==="
    mkdir -p "$LIVOX_SDK/build"
    cd "$LIVOX_SDK/build"
    cmake .. && make -j$(nproc) && sudo make install
fi

# ============================================================
# 7. Build livox_ros_driver2 (needs its own workspace due to custom build.sh)
# ============================================================
LIVOX_DRIVER="$SCRIPT_DIR/deps/livox_ros_driver2"
LIVOX_WS="$SANDO_WS/livox_ws"
if [ -d "$LIVOX_DRIVER" ] && [ ! -d "$LIVOX_WS/install/livox_ros_driver2" ]; then
    echo "=== Building livox_ros_driver2 ==="
    mkdir -p "$LIVOX_WS/src"
    ln -sf "$LIVOX_DRIVER" "$LIVOX_WS/src/livox_ros_driver2"
    cd "$LIVOX_WS/src/livox_ros_driver2"
    source /opt/ros/humble/setup.bash
    ./build.sh humble
fi

# ============================================================
# 8. Build the workspace
# ============================================================
echo "=== Building SANDO workspace ==="
cd "$SANDO_WS"
source /opt/ros/humble/setup.bash
# Source livox driver if built
[ -f "$LIVOX_WS/install/setup.bash" ] && source "$LIVOX_WS/install/setup.bash"
colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
                 -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
                 -DCMAKE_C_COMPILER=/usr/bin/gcc \
    --allow-overriding gazebo_dev gazebo_msgs gazebo_ros gazebo_ros_pkgs gazebo_plugins

# ============================================================
# 9. Configure shell environment
# ============================================================
echo "=== Configuring shell ==="

# Add Gurobi to bashrc (idempotent)
grep -qxF 'export GUROBI_HOME="/opt/gurobi1103/linux64"' ~/.bashrc || cat >> ~/.bashrc << 'BASHEOF'

# Gurobi
export GUROBI_HOME="/opt/gurobi1103/linux64"
export PATH="${PATH}:${GUROBI_HOME}/bin"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"
export GRB_LICENSE_FILE="$HOME/gurobi.lic"
BASHEOF

# Add ROS domain ID
grep -qxF 'export ROS_DOMAIN_ID=20' ~/.bashrc || \
    echo 'export ROS_DOMAIN_ID=20' >> ~/.bashrc

echo ""
echo "=== SANDO setup complete! ==="
echo ""
echo "  To use:"
echo "    cd $SANDO_WS"
echo "    source install/setup.bash"
echo "    python3 src/sando/scripts/run_sim.py -m interactive -s install/setup.bash"
echo ""
