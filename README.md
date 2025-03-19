# Home of HomeR (the robot)

## Quick Start

### Build Packages

```console
cd <ros workspace>  # e.g. cd ~/ros_ws/
colcon build
source install/local_setup.bash
```

### Start Driver

> On Robot (RPi)

```console
ros2 launch homer_control bringup_homer.launch.py
```

### Mapping

> (Recommended) On Host Machine

```console
ros2 launch homer_control joy_chart.launch.py
```

### Navigation

> (Recommended) On Host Machine

```console
ros2 launch homer_control navigate.launch.py
```
