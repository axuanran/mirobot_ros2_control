# TODO

## environment

```bash
cd ~/data/robot/ros2ws &&  pyenv local 3.10.12 && source /opt/ros/humble/setup.bash && source ./install/setup.bash
```

```bash
colcon build --packages-select my_mirobot_package
```

## launch

```bash
ros2 launch teleop_twist_joy teleop-launch.py config_filepath:="/home/xuanran/data/robot/ros2ws/xbox.config.yaml"
ros2 run my_mirobot_package mirobot_teleop_with_wlkatapythonp --ros-args -p mirobot_tools:=2 -p serial_port:=/dev/ttyUSB1 -p instructions_file:=instructions.txt
ros2 topic echo /joy
ros2 topic echo /cmd_vel
ros2 topic echo /mirobot_status
ros2 topic echo /mirobot_action
```

! you must to edit the configuration for yourself
! the homing with the joystick maybe have some unknown action and maybe break the robot and i  know nothing with it (:<)

this project are used in my project for SH_CXDS
