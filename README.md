# Autonomous car simulator Localization module

### Intro
### Features

# Requirements

OS: Ubuntu 20.04\
Unreal engine: 4.26\
[carla-simulator: 0.9.13](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13.tar.gz)\
Ros (apt installation): Foxy\
[carla-ros-bridge/bionic: 0.9.10-1](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/) \
[PCL point cloud]() \
[ros2_numpy](https://github.com/Box-Robotics/ros2_numpy) \
Docker


# Building
  ```bash
  cd ~/colcon_ws
  source /opt/ros/foxy/setup.bash
  colcon build --packages-select mike_av_stack_localization
  ```

# How to Run
```
$ cd .../workspace_folder/
$ export ROS_DOMAIN_ID=1
$ source install/setup.bash
$ ros2 launch mike_av_stack_localization localization.launch
$ ros2 run mike_av_stack_localization localization --ros-args --log-level debug
```

# Demos
# Docs
# Todo