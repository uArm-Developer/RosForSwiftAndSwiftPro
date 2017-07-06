# SwiftproForROS
This is the swiftpro ROS package designed by Roger Cui(roger@ufactory.cc) and David Long (xiaokun.long@ufactory.cc). 
These packages support Moveit!, RViz and serial communication with swiftpro.

## 1. Download and install
Download ros packages for uarm swift pro
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/uArm-Developer/SwiftproForROS.git
```
Install ros serial package
```bash
$ sudo apt-get install ros-kinetic-serial
```
Copy head file into include folder of catkin_ws
```bash
$ cd ~/catkin_ws
$ cp -r src/swiftpro/include/swiftpro devel/include
```
Compile
```bash
$ catkin_make
```

## 2. Set up enviroment
Source all setup.bash files to set up your enviroment.
```bash
# System configure ROS environment variables automatically every time you open a ternimal
echo "source /opt/ros/[ROS_version]/setup.bash" >> ~/.bashrc
# For example, if you are using kinetic version of ROS
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

# Source setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 3. Simulation
Get USB permission to access uArm
```bash
$ sudo chmod 666 /dev/ttyACM0
```

### 3.1 Get data from swiftpro
Connect swiftpro, get data from serial and simulate swiftpro in RViz.
```bash
roslaunch swiftpro display.launch
```
right now, you can drag and teach your swiftpro and it will simulate in Rviz.

### 3.2 Send data to swiftpro
Connect swiftpro, send data though serial (need to close serial_rece_node first).
```bash
roslaunch swiftpro control.launch
```
Open another ternimal to get joint angles from Moveit!.
```bash
roslaunch swiftpro_moveit demo.launch
```
right now, you can do trajectory planning or grasping in moveIt!.

### 3.3 About nodes and topics
<img src="http://obmqyor62.bkt.clouddn.com/ROS_swiftpro2.jpg" width = "800" height = "160" />

### 3.4 About message
unit of motor angles is degree; unit of length(x, y, z) is millimeter
```bash
float64 motor_angle1
float64 motor_angle2
float64 motor_angle3
float64 motor_angle4
float64 cart_x
float64 cart_y
float64 cart_z
```
