

```sh
ros2 launch seaweed_sim gazebo.launch.py
# args: model:=x_drive or model:=diff_thrust
```



```sh
ros2 launch seaweed_sim diff_thrust_joy_sim.launch.py
```

diff thrust controller has callbacks for both joy and cmd velo for testing purposes
if using joy, RIGHT_TRIGGER is dead man's switch, left joystick to control motion.
