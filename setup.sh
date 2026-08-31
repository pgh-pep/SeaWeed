#!/bin/bash
if [ -z $1 ]; then
   echo "No parent folder argument. Defaulting to ~/pep..."
   PEP_DIR="~/pep"
else
   PEP_DIR=$1
fi

if [ -d "$PEP_DIR/src_deps" ]; then
   echo "Removing old src_deps folder..."
   rm -rf "$PEP_DIR/src_deps"
fi

if [ -d "$PEP_DIR/seaweed_ws" ]; then
   echo "Found existing seaweed_ws folder. Skipping workspace setup..."
else
   echo "Setting up SeaWeed workspace..."
   mkdir -p "$PEP_DIR/seaweed_ws/src" && cd $PEP_DIR/seaweed_ws/src && git clone https://github.com/pgh-pep/SeaWeed.git
fi

mkdir "$PEP_DIR/src_deps"

echo "Setting up source dependency workspaces..."
mkdir -p "$PEP_DIR/src_deps/vrx_ws/src" && cd $PEP_DIR/src_deps/vrx_ws/src && git clone -b pep_jazzy https://github.com/pgh-pep/vrx.git
mkdir -p "$PEP_DIR/src_deps/dlio_ws/src" && cd $PEP_DIR/src_deps/dlio_ws/src && git clone -b feature/ros2 https://github.com/pgh-pep/direct_lidar_inertial_odometry.git
mkdir -p "$PEP_DIR/src_deps/velodyne_ws/src" && cd $PEP_DIR/src_deps/velodyne_ws/src && git clone -b ros2 https://github.com/ros-drivers/velodyne.git

echo "Installing rosdeps..."
cd $PEP_DIR/src_deps/vrx_ws/ && rosdep install --from-paths src --ignore-src -r -y
cd $PEP_DIR/src_deps/dlio_ws/ && rosdep install --from-paths src --ignore-src -r -y
cd $PEP_DIR/src_deps/velodyne_ws/ && rosdep install --from-paths src --ignore-src -r -y
cd $PEP_DIR/seaweed_ws/ && rosdep install --from-paths src --ignore-src -r -y

echo "Installing additional dependencies..."
sudo apt install libomp-dev libpcl-dev libeigen3-dev
cd $PEP_DIR/seaweed_ws/src/SeaWeed && pip install -r requirements.txt

echo "Building source packages..."
cd $PEP_DIR/src_deps/vrx_ws/ && colcon build --merge-install
cd $PEP_DIR/src_deps/dlio_ws/ && colcon build --symlink-install --packages-select direct_lidar_inertial_odometry
cd $PEP_DIR/src_deps/velodyne_ws/ && colcon build

echo "Building SeaWeed workspace..."
source $PEP_DIR/src_deps/vrx_ws/install/setup.bash
cd $PEP_DIR/seaweed_ws && colcon build --symlink-install