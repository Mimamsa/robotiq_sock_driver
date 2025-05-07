# Robotiq sock driver

Robotiq gripper control node for UR-mounted grippers with URCap installed on the pendant.

Commands are published to port 63352 as ASCII strings.


## Environment

Ubuntu 22.04, Python 3.10, RO2 Humble


## Build

```
colcon build --packages-select robotiq_sock_driver_msgs
```

```
colcon build --packages-select robotiq_sock_driver
```


## Usage

```
ros2 launch robotiq_sock_driver gripper_driver ip_address:=192.168.10.4 port:=63352
```


### Topics

```
/gripper/cmd [robotiq_sock_driiver_msgs/msg/GripperCmd]
/gripper/stat [robotiq_sock_driiver_msgs/msg/GripperStat]
/gripper/joint_states [sensor_msgs/msg/JointState]
```


### Testing command

```
ros2 topic pub --once \
/gripper/cmd robotiq_sock_driiver_msgs/msg/GripperCmd \
"{\
emergency_release: false, \
emergency_release_dir: true, \
stop: false, \
position: 0, \
speed: 0, \
force: 0, \
}"
```


## Reference

1. [programming options ur16e 2f-85 - dof.robotiq.com](https://dof.robotiq.com/discussion/1962/programming-options-ur16e-2f-85)

2. [felixvd/cmodel_urcap.py - gist.github.com](https://gist.github.com/felixvd/d538cad3150e9cac28dae0a3132701cf)

3. [PickNikRobotics/robotiq_85_gripper - github.com](https://github.com/PickNikRobotics/robotiq_85_gripper/blob/0f8410468ffd7b45a3345f411bacd855920c612e/robotiq_85_driver/)

