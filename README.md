# Robotiq Socket driver

A ROS2 controlling node for UR-mounted Robotiq grippers with Robotiq Wrist Camera URCap installed on the pendant.

Commands are published to port 63352 as ASCII strings.


## Dependency

This ROS2 node cooperates with the following tools:
1. Robotiq Wrist Camera
2. Robotiq Wrist Camera URCap. See [Access manuals, guides, software CAD and more - robotiq.com](https://robotiq.com/support).


## Environment

- Ubuntu 22.04
- Python 3.10
- ROS2 Humble


## Build

### 1. Create directories

For my example, all ROS2 package are in ```~/ros2_ws/src``` directory.

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Mimamsa/robotiq_sock_driver
```

### 2. Move packages

Move ```robotiq_sock_driver``` and ```robotiq_sock_driver_msgs``` packages to the parent directory.

```
cd ~/ros2_ws/src/robotiq_sock_driver
mv robotiq_sock_driver ..
mv robotiq_sock_driver_msgs ..
cd ..
rm -rf robotiq_sock_driver
```

### 3. Build

Build robotiq_sock_driver_msgs package:

```
cd ~/ros2_ws
colcon build --packages-select robotiq_sock_driver_msgs
```

Build robotiq_sock_driver package:
```
cd ~/ros2_ws
colcon build --packages-select robotiq_sock_driver
```


## Usage

Running a node with a pre-configured launch file:
```
ros2 launch robotiq_sock_driver gripper_driver.launch.py
```

Or alternatively running the node without the launch file:
```
ros2 run robotiq_sock_driver gripper_driver --ros-args \
-p ip_address:=192.168.10.4 -p port:=63352 \
-p inversed_pos:=true -p auto_calibrate:=true
```


### Topics

```
/gripper/cmd [robotiq_sock_driver_msgs/msg/GripperCmd]
/gripper/stat [robotiq_sock_driver_msgs/msg/GripperStat]
/gripper/joint_states [sensor_msgs/msg/JointState]

```


### Testing command

```
ros2 topic pub --once /gripper/cmd robotiq_sock_driver_msgs/msg/GripperCmd "{emergency_release: false, emergency_release_dir: 1, stop: false, position: 0.0, speed: 0.0, force: 0.0}"
```

```
ros2 echo /gripper/stat
```

```
ros2 echo /gripper/joint_states
```


## TODOs

1. Fix duplicated auto-calibration.
2. Change all parameters to arguments.


## Reference

1. [programming options ur16e 2f-85 - dof.robotiq.com](https://dof.robotiq.com/discussion/1962/programming-options-ur16e-2f-85)

2. [felixvd/cmodel_urcap.py - gist.github.com](https://gist.github.com/felixvd/d538cad3150e9cac28dae0a3132701cf)

3. [PickNikRobotics/robotiq_85_gripper - github.com](https://github.com/PickNikRobotics/robotiq_85_gripper/blob/0f8410468ffd7b45a3345f411bacd855920c612e/robotiq_85_driver/)


## Inforation

- Author: Yu-Hsien Chen (mike_chen@wistron.com)
- Latest update: 2025/6/11

