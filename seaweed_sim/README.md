

```sh
ros2 launch seaweed_sim gazebo.launch.py model:=x_drive world:=follow_path
# args: model:=x_drive, diff_thrust
# world:=sydney_regatta, sydney_regatta_empty, nbpark or follow_path
```



```sh
ros2 launch seaweed_sim diff_thrust_joy_sim.launch.py
```

diff thrust controller has callbacks for both joy and cmd velo for testing purposes
if using joy, RIGHT_TRIGGER is dead man's switch, left joystick to control motion.



```sh
# to generate follow_path.sdf from follow_path_obstacles.yaml
python3 seaweed_sim/seaweed_sim/obstacle_generator.py seaweed_sim/worlds/sydney_regatta_empty.sdf seaweed_sim/config/follow_path_obstacles.yaml -o seaweed_sim/worlds/follow_path.sdf
```
