# FRA501-JINGPAO


## Install & Setup ROS2 micro_ros 

install micro_ros package 

```bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

Create a new firmware workspace
```bash 
# Create firmware step
ros2 run micro_ros_setup create_firmware_ws.sh host
```

Building the firmware
```bash
source install/local_setup.bash
ros2 run micro_ros_setup build_firmware.sh
```

Creating the micro-ROS agent
```bash
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

Give permission Com_Port for communicate
```bash
sudo chmod 777 /dev/{Com_Port}  
```

Run micro_ros to communicate with microcontroller 
```bash
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/{Com_Port} -b {Baud_rate}
```

micro_ros Topic Tests, it will display topic that micro_ros publish
```bash
ros2 topic list
``` 

