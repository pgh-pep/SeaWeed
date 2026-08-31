#!/bin/bash

if [ -z $1 ]; then
   echo "No parent folder argument. Defaulting to ~/pep."
   PEP_DIR="~/pep"
else
   PEP_DIR=$1
fi

if [ -d "$PEP_DIR/pep-deps" ]; then
   echo "Removing old pep-deps folder..."
   rm -rf "$PEP_DIR/pep-deps"
fi

mkdir "$PEP_DIR/pep-deps"

echo "Setting up workspaces..."

mkdir -p "$PEP_DIR/pep-deps/vrx_ws/src" && cd $PEP_DIR/pep-deps/vrx_ws/src && git clone -b pep_jazzy https://github.com/pgh-pep/vrx.git
mkdir -p "$PEP_DIR/pep-deps/dlio_ws/src" && cd $PEP_DIR/pep-deps/dlio_ws/src && git clone -b feature/ros2 https://github.com/pgh-pep/direct_lidar_inertial_odometry.git
mkdir -p "$PEP_DIR/pep-deps/velodyne_ws/src" && cd $PEP_DIR/pep-deps/velodyne_ws/src && git clone -b ros2 https://github.com/ros-drivers/velodyne.git

echo "Installing rosdeps..."
cd $PEP_DIR/pep-deps/vrx_ws/ && rosdep install --from-paths src --ignore-src -r -y
cd $PEP_DIR/pep-deps/dlio_ws/ && rosdep install --from-paths src --ignore-src -r -y
cd $PEP_DIR/pep-deps/velodyne_ws/ && rosdep install --from-paths src --ignore-src -r -y

echo "Installing additional dependencies..."
sudo apt install libomp-dev libpcl-dev libeigen3-dev

echo "Running build commands..."
cd $PEP_DIR/pep-deps/vrx_ws/ && colcon build --merge-install
cd $PEP_DIR/pep-deps/dlio_ws/ && colcon build --symlink-install --packages-select direct_lidar_inertial_odometry
cd $PEP_DIR/pep-deps/velodyne_ws/ && colcon build