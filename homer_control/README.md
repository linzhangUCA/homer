# `homer_control` Package

## Build Package

```console
cd <ros workspace>  # e.g. cd ~/ros_ws/
colcon build
source install/local_setup.bash
```

## Applications

### 1. Gamepad teleop mapping

#### On Robot (RPi)

```console
ros2 launch homer_control bringup_homer.launch.py
```

#### On Host Machine

```console
ros2 launch homer_control joy_chart.launch.py
```

### 2. RViz2 Navigation

#### On Robot (RPi)

```console
ros2 launch homer_control bringup_homer.launch.py
```

#### On Host Machine

```console
ros2 launch homer_control navigate.launch.py
```
