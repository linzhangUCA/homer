# Usage

## Build Package
```console
cd <ros workspace>
colcon build
source install/local_setup.bash
```

## Launch
#### On Robot (RPi)
```console
ros2 launch homer_control bringup_homer.launch.py
```
#### On Host Machine
```console
ros2 launch slam_toolbox online_async_launch.py
ros2 launch nav2_bringup navigation_launch.py
```
