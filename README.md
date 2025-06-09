# Wlkata Mirobot controller for ros2

## environment

Certainly,you should install ros2 environment first.

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
! Homing with the joystick maybe have some unexpected action and it maybe break the robot and i don't know why (:<)
if you encounter this problem, maybe you can try lifting hardware restrictions? I don't know how to fix it with a better way, use hand make my robot don't have a flexible  action as before )
but zero is safe(:))

thanks for https://github.com/wlkata/WLKATA-Python-SDK-wlkatapython/

(dev is already completed)
