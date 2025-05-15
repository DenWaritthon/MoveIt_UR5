# UR5 Move Using MoveIt2
Move real UR5 Robot using ROS2 Humble.

## Pre-Requirement
- ROS2 Jazzy
- Universal_Robots_ROS2_Driver 
  ```bash
  sudo apt-get install ros-jazzy-ur
  ```
  - Github: https://github.com/DenWaritthon/MoveIt_UR5
  - Documents: https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/index.html

## System Architecture
![System Architecture](<pictures/System Architecture.png>)

**UR5 Path Planning and Execute Move UR5**

Using Universal_Robots_ROS2_Driver for connect UR5 Robot to ROS2 system and Using MoveIt package for create path of UR5 Robot and execute path to move real UR5 Robot using `MoveGroupInterface`.

## Network setup UR5 Robot
### Setup UR5
In Setup Robot -> Network -> Network detailed settings
```
IP address: 192.168.1.102
Subnet mask: 255.255.255.0
Default gateway: 192.168.1.1
```
### Setup PC
```
IP address: 192.168.1.101
Subnet mask: 255.255.255.0
Default gateway: 192.168.1.1
```
### Test Network Connection
```bash
ping 192.168.1.102
```
if successful output is
```
# Output
PING 192.168.1.102 (192.168.1.102) 56(84) bytes of data.
64 bytes from 192.168.1.102: icmp_seq=1 ttl=64 time=0.153 ms
64 bytes from 192.168.1.102: icmp_seq=2 ttl=64 time=0.178 ms
64 bytes from 192.168.1.102: icmp_seq=3 ttl=64 time=0.183 ms
```
### Setup UR5 External Control
In UR5 program insert External Control and setup in Installation -> External Control
```
Host IP: 192.168.1.101 # Using PC IP address
Custom port: 50002 # Default in Driver
Host name: EXternal Control
```
## Testing UR5 using Universal_Robots_ROS2_Driver
Run this command in terminal
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102
```
if successful output is
```
# Output
[INFO] [spawner-7]: process has finished cleanly [pid 10600]
```
![UR5 Rviz](<pictures/Testing UR5 using Universal_Robots_ROS2_Driver.png>)
**Robot position in Rviz is related to Real UR5 Robot.**

## Using Project

### Install Project
Install project from this GitHub 

```bash
git clone https://github.com/DenWaritthon/ur_move_ws.git
```
Build and Source workspace

```bash
cd ur_move_ws/
colcon build
source install/setup.bash
```
Add `source ~/ur_move_ws/install/setup.bash` to `.bashrc` file.
```bash
echo "source ~/ur_move_ws/install/setup.bash" >> ~/.bashrc
```
### How to Use this project

1. launch UR Robot Driver for connect UR5 Robot.
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 launch_rviz:=true robot_ip:=192.168.1.102
```
2. Run External Control Program In Teach Pendant if successful output in terminal is.
```bash
# Output
[ur_ros2_control_node-1] [INFO] [1741780162.100902189] [UR_Client_Library:]: Robot connected to reverse interface. Ready to receive control commands.
```
3. Launch MoveIt Config for control UR5 Robot.
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=false
```

## Demo

**Click to watch VDO**
