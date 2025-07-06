# Seaweed Description

## Launch Files

Note: To use simulation/visualization tools, you need to source both `seaweed_ws` and `vrx_ws`.

To visualize URDFs generated via VRX w/ rviz:

```sh
ros2 launch seaweed_description display_rviz.launch.py model:=x_drive
ros2 launch seaweed_description display_rviz.launch.py model:=diff_thrust
```



Generated URDFs using VRX toolkit:
```sh
# in seaweed_ws
ros2 launch vrx_gazebo generate_wamv.launch.py component_yaml:=`pwd`/src/SeaWeed/seaweed_description/urdf/x_drive_wamv/x_drive_component_config.yaml thruster_yaml:=`pwd`/src/SeaWeed/seaweed_description/urdf/x_drive_wamv/x_drive_thruster_config.yaml wamv_target:=`pwd`/src/SeaWeed/seaweed_description/urdf/x_drive_wamv/wamv_target.urdf wamv_locked:=False

ros2 launch vrx_gazebo generate_wamv.launch.py component_yaml:=`pwd`/src/SeaWeed/seaweed_description/urdf/diff_thrust_wamv/diff_thrust_component_config.yaml thruster_yaml:=`pwd`/src/SeaWeed/seaweed_description/urdf/diff_thrust_wamv/diff_thrust_thruster_config.yaml wamv_target:=`pwd`/src/SeaWeed/seaweed_description/urdf/diff_thrust_wamv/wamv_target.urdf wamv_locked:=False

```