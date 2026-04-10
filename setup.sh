#!/bin/bash
# --------------------------------------------------------------------------
# SANDO Setup Script
# Installs ROS 2 Humble, Gurobi, and all dependencies, then builds the workspace.
#
# Usage:
#   git clone --branch v0.0.3 --recursive https://github.com/mit-acl/sando.git
#   cd sando && ./setup.sh [-j N]
#
#   -j N  Number of parallel jobs for building (default: all CPUs)
# --------------------------------------------------------------------------
set -e

# Parse arguments
NUM_JOBS=$(nproc)
while [[ $# -gt 0 ]]; do
    case $1 in
        -j|--jobs)
            NUM_JOBS="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: ./setup.sh [-j N]"
            exit 1
            ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SANDO_WS="$(cd "$SCRIPT_DIR/../.." 2>/dev/null && pwd)" || SANDO_WS="$HOME/code/sando_ws"
CODE_DIR="$(cd "$SANDO_WS/.." && pwd)"
DECOMP_WS="$CODE_DIR/decomp_ws"

echo "============================================="
echo "SANDO Complete Setup Script"
echo "============================================="
echo "  Workspace: $SANDO_WS"
echo "  Decomp:    $DECOMP_WS"
echo "  Sando dir: $SCRIPT_DIR"
echo "  Using $NUM_JOBS parallel jobs for building"
echo ""

# Prompt for sudo password once at the beginning and keep it cached
echo "This script requires sudo access for:"
echo "  - Installing system packages"
echo "  - Installing Livox-SDK2"
echo ""
sudo -v
# Keep sudo alive in background (updates timestamp every 60 seconds)
while true; do sudo -n true; sleep 60; kill -0 "$$" || exit; done 2>/dev/null &

# ============================================================
# 1. Fix Repository Issues
# ============================================================
echo ""
echo "============================================="
echo "STEP 1: Fixing Repository Issues"
echo "============================================="

# Only add ROS 2 repository if none exists — never overwrite existing setup
if ls /etc/apt/sources.list.d/ros2* > /dev/null 2>&1; then
    echo "ROS 2 repository already configured, keeping existing setup."
else
    echo "No ROS 2 repository found, adding..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

# ============================================================
# 2. System packages
# ============================================================
echo ""
echo "============================================="
echo "STEP 2: System Updates and Basic Software"
echo "============================================="
sudo apt update && sudo apt upgrade -y
sudo apt-get install -q -y --no-install-recommends \
    git tmux vim wget tmuxp make gdb openssh-server net-tools \
    g++ xterm python3-pip build-essential \
    libomp-dev libpcl-dev libeigen3-dev nlohmann-json3-dev

# ============================================================
# 3. ROS 2 Humble
# ============================================================
echo ""
echo "============================================="
echo "STEP 3: Installing ROS 2 Humble"
echo "============================================="

if [ ! -d "/opt/ros/humble" ]; then
    echo "Installing ROS 2 Humble..."
    sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    sudo apt install -y software-properties-common
    echo -e "\n" | sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl

    # ROS 2 repo already set up in Step 1 — just update
    sudo apt update
    sudo apt install -y ros-humble-desktop ros-dev-tools
    grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || \
        echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
else
    echo "ROS 2 Humble already installed, skipping..."
fi

export ROS_DISTRO=humble
source /opt/ros/humble/setup.bash

# ============================================================
# 4. ROS Dependencies
# ============================================================
echo ""
echo "============================================="
echo "STEP 4: Installing ROS Dependencies"
echo "============================================="
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
# 5. Gurobi (if not already installed)
# ============================================================
echo ""
echo "============================================="
echo "STEP 5: Installing Gurobi"
echo "============================================="
if [ ! -d "/opt/gurobi1103" ]; then
    echo "Installing Gurobi 11.0.3..."
    wget -q https://packages.gurobi.com/11.0/gurobi11.0.3_linux64.tar.gz -P /tmp
    sudo tar -xzf /tmp/gurobi11.0.3_linux64.tar.gz -C /opt
    rm /tmp/gurobi11.0.3_linux64.tar.gz
    cd /opt/gurobi1103/linux64/src/build
    sudo make && sudo cp libgurobi_c++.a ../../lib/
    echo ""
    echo "  NOTE: Place your Gurobi license file at ~/gurobi.lic"
    echo "  Free academic licenses: https://www.gurobi.com/academia/academic-program-and-licenses/"
    echo ""
else
    echo "Gurobi already installed, skipping..."
fi

export GUROBI_HOME="/opt/gurobi1103/linux64"
export PATH="${PATH}:${GUROBI_HOME}/bin"
export LD_LIBRARY_PATH="${GUROBI_HOME}/lib:${LD_LIBRARY_PATH}"

# ============================================================
# 6. Clone/Checkout SANDO and initialize submodules
# ============================================================
echo ""
echo "============================================="
echo "STEP 6: Setting Up SANDO Repository"
echo "============================================="

mkdir -p "$SANDO_WS/src"

if [ ! -d "$SANDO_WS/src/sando" ]; then
    echo "Cloning SANDO..."
    cd "$SANDO_WS/src"
    git clone https://github.com/mit-acl/sando.git
    cd sando
    git checkout v0.0.3
else
    echo "SANDO already exists, updating..."
    cd "$SANDO_WS/src/sando"
    git fetch
fi

echo "Initializing submodules..."
cd "$SANDO_WS/src/sando"
git submodule update --init --recursive

# ============================================================
# 7. Create symlinks for colcon discovery
# ============================================================
echo ""
echo "============================================="
echo "STEP 7: Setting Up Workspace Symlinks"
echo "============================================="
WS_SRC="$SANDO_WS/src"
for d in "$SCRIPT_DIR"/deps/*/; do
    name=$(basename "$d")
    # Skip deps built in separate workspaces
    case "$name" in Livox-SDK2|livox_ros_driver2|DecompROS2) continue;; esac
    if [ ! -e "$WS_SRC/$name" ]; then
        ln -s "$d" "$WS_SRC/$name"
        echo "  Linked: $name"
    fi
done

# ============================================================
# 8. Build DecompROS2 (separate workspace, matching mighty)
# ============================================================
echo ""
echo "============================================="
echo "STEP 8: Building DecompROS2"
echo "============================================="
mkdir -p "$DECOMP_WS/src"

DECOMP_SRC="$SCRIPT_DIR/deps/DecompROS2"
if [ -d "$DECOMP_SRC" ]; then
    if [ ! -d "$DECOMP_WS/src/DecompROS2" ]; then
        echo "Linking DecompROS2 to decomp_ws..."
        ln -sf "$DECOMP_SRC" "$DECOMP_WS/src/DecompROS2"
    fi

    cd "$DECOMP_WS"
    source /opt/ros/humble/setup.bash

    echo "Building decomp_util first (required by other packages)..."
    colcon build --packages-select decomp_util \
        --parallel-workers "$NUM_JOBS" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

    echo "Building remaining decomp packages..."
    source "$DECOMP_WS/install/setup.bash"
    colcon build --parallel-workers "$NUM_JOBS" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
else
    echo "DecompROS2 not found, skipping..."
fi

# ============================================================
# 9. Build Livox-SDK2 (cmake, not colcon)
# ============================================================
echo ""
echo "============================================="
echo "STEP 9: Building Livox-SDK2"
echo "============================================="
LIVOX_SDK="$SCRIPT_DIR/deps/Livox-SDK2"
if [ -d "$LIVOX_SDK" ] && [ ! -f "/usr/local/lib/liblivox_lidar_sdk_static.a" ]; then
    echo "Installing Livox-SDK2..."
    mkdir -p "$LIVOX_SDK/build"
    cd "$LIVOX_SDK/build"
    cmake .. && make -j"$NUM_JOBS" && sudo make install
else
    echo "Livox-SDK2 already installed, skipping..."
fi

# ============================================================
# 10. Build livox_ros_driver2 (needs its own workspace due to custom build.sh)
# ============================================================
echo ""
echo "============================================="
echo "STEP 10: Building livox_ros_driver2"
echo "============================================="
LIVOX_DRIVER="$SCRIPT_DIR/deps/livox_ros_driver2"
LIVOX_WS="$CODE_DIR/livox_ws"
if [ -d "$LIVOX_DRIVER" ] && [ ! -d "$LIVOX_WS/install/livox_ros_driver2" ]; then
    echo "Building livox_ros_driver2..."
    mkdir -p "$LIVOX_WS/src"
    ln -sf "$LIVOX_DRIVER" "$LIVOX_WS/src/livox_ros_driver2"
    cd "$LIVOX_WS/src/livox_ros_driver2"
    source /opt/ros/humble/setup.bash
    ./build.sh humble
else
    echo "livox_ros_driver2 already built, skipping..."
fi

# ============================================================
# 11. Build SANDO workspace
# ============================================================
echo ""
echo "============================================="
echo "STEP 11: Building SANDO Workspace"
echo "============================================="
cd "$SANDO_WS"
source /opt/ros/humble/setup.bash
source "$DECOMP_WS/install/setup.bash"
export CMAKE_PREFIX_PATH="$LIVOX_WS/install/livox_ros_driver2:$DECOMP_WS/install/decomp_util:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$LIVOX_WS/install/livox_ros_driver2/lib:$LD_LIBRARY_PATH"

colcon build \
    --parallel-workers "$NUM_JOBS" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
                 -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
                 -DCMAKE_C_COMPILER=/usr/bin/gcc \
    --allow-overriding gazebo_dev gazebo_msgs gazebo_ros gazebo_ros_pkgs gazebo_plugins

# ============================================================
# 12. Configure shell environment
# ============================================================
echo ""
echo "============================================="
echo "STEP 12: Setting Up Bash Configuration"
echo "============================================="

# Add Gurobi to bashrc (idempotent)
grep -qxF 'export GUROBI_HOME="/opt/gurobi1103/linux64"' ~/.bashrc || cat >> ~/.bashrc << 'BASHEOF'

# Gurobi
export GUROBI_HOME="/opt/gurobi1103/linux64"
export PATH="${PATH}:${GUROBI_HOME}/bin"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"
export GRB_LICENSE_FILE="$HOME/gurobi.lic"
BASHEOF

# Add Livox library path
grep -qxF "export LD_LIBRARY_PATH=$LIVOX_WS/install/livox_ros_driver2/lib:\$LD_LIBRARY_PATH" ~/.bashrc || \
    echo "export LD_LIBRARY_PATH=$LIVOX_WS/install/livox_ros_driver2/lib:\$LD_LIBRARY_PATH:/usr/local/lib" >> ~/.bashrc

# Source decomp and SANDO workspaces
grep -qxF "source $DECOMP_WS/install/setup.bash" ~/.bashrc || \
    echo "source $DECOMP_WS/install/setup.bash" >> ~/.bashrc
grep -qxF "source $SANDO_WS/install/setup.bash" ~/.bashrc || \
    echo "source $SANDO_WS/install/setup.bash" >> ~/.bashrc

# Add ROS domain ID
grep -qxF 'export ROS_DOMAIN_ID=20' ~/.bashrc || \
    echo 'export ROS_DOMAIN_ID=20' >> ~/.bashrc

# ============================================================
# Summary
# ============================================================
echo ""
echo "============================================="
echo "SANDO Setup Complete!"
echo "============================================="
echo ""
echo "Workspaces:"
echo "  - SANDO:  $SANDO_WS"
echo "  - Decomp: $DECOMP_WS"
echo "  - Livox:  $LIVOX_WS"
echo ""
echo "To get started:"
echo "  source ~/.bashrc"
echo "  cd $SANDO_WS"
echo ""
echo "  # Single-agent interactive simulation (click goals in RViz2 with \"2D Goal Pose\")"
echo "  python3 src/sando/scripts/run_sim.py -m interactive -s install/setup.bash"
echo ""
echo "See README.md for more simulation modes and options."
echo "============================================="
