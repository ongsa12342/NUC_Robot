# NUC_Robot
This repository contains the setup and launch instructions for the NUC_Robot project, which includes support for the Unitree LiDAR sensor.
![image](https://github.com/user-attachments/assets/d8f1b66f-170a-48bb-8e90-d4080bdfe2f4)

## Installation and Setup
### Step 1: Clone the Repository
Clone the repository to your workspace:
```
git clone https://github.com/ongsa12342/NUC_Robot.git
```
### Step 2: Install Dependencies
Update `rosdep` and install all dependencies:
```
rosdep update && rosdep install --from-paths src --ignore-src -y
```
### Step 3: Build the Package
Navigate to the cloned repository and build it using `colcon`:
```
cd NUC_Robot && colcon build
```
### Step 4: Source the Setup File
To set up the environment, source the following:
```
source ~/NUC_Robot/install/setup.bash
```
### Step 5: (Optional) Add to .bashrc
For convenience, you may want to add the setup file to your `.bashrc`:
```
echo "source ~/NUC_Robot/install/setup.bash" >> ~/.bashrc && source ~/.bashrc
```
## Running a NUC robot
Run the initial launch file to initialize the micro-ROS agent, Unitree LiDAR, joystick, and required nodes:
```
ros2 launch navigate_NUC initial.launch.py
```
Run the navigation launch file for Nav2:
```
ros2 launch navigate_NUC nav2.launch.py
```
Launch the GUI for user interaction:
```
ros2 run navigate_NUC nuc_user_interface.py
```
## GUI
[Placeholder for GUI]
about....


## Teleop
### joy teleop


# Extentions
## Running the Unitree LiDAR
To launch the Unitree LiDAR:
```
ros2 launch unitree_lidar_ros2 launch.py
```
### Visualize in RViz
To open RViz with a preset configuration for the Unitree LiDAR:
```
rviz2 -d src/unitree_lidar_ros2/rviz/view.rviz
```
## Running Lidar SLAM
```
ros2 launch lidarslam lidarslam.launch.py
```
Run a bag file for map collection
```
ros2 bag play data
```
## Nav2 Launch 
```
ros2 launch convert_odom simple.launch.py 
ros2 launch convert_odom nav2.launch.py 
```
# System Architecture

# Validation

### Acknowledgments
This project relies on the following libraries and tools:
 - robot_localization - For state estimation and sensor fusion.
 - unitree_lidar - Integration and support for Unitree LiDAR sensors.
 - micro-ROS - ROS 2 framework for microcontrollers.
 - Nav2 - Navigation stack for autonomous robots.
