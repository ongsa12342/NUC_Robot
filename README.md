# NUC_Robot
This repository provides the setup and launch instructions for the NUC_Robot project, an omni-directional drive robot equipped with a Unitree LiDAR sensor. The robot utilizes the NAV2 framework for autonomous navigation and includes a mode-switching feature that allows seamless transitions between teleoperation(Manual) and navigation to goal(Auto).



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
1. Run the initial launch file to initialize the micro-ROS agent, Unitree LiDAR, joystick, and required nodes:
```
ros2 launch navigate_NUC initial.launch.py
```
2. Run the navigation launch file for Nav2:
```
ros2 launch navigate_NUC nav2.launch.py
```
3. Launch the GUI for user interaction:
```
ros2 run navigate_NUC nuc_user_interface.py
```
## GUI

![image](https://github.com/user-attachments/assets/f19cb0b6-cc70-402d-8ddf-b174f58a1582)

### Default Screen Components:
1. **Set Home Button**  
   - Publishes `/initialpose` using the room name "home" for pose estimation in `nav2`.

2. **Save Current Room Button**  
   - Saves the current position to the YAML file (`src/navigate_NUC/params/room_coordinates.yaml`) by subscribing to `/amcl_pose`.

3. **Mode Selection**  
   - Allows switching between modes.  
     - **Default Mode:** Manual  
     - **Auto Mode:**  
       - Reads all rooms from the YAML file.  
       - Displays buttons for each room.  
       - Clicking a room button sends a goal to `nav2` for navigation.


## Navigation
### Map
We created a map using this robot on the 3rd floor of FIBO KMUTT.

# System Architecture
(Click the image for better quality)
![Service](https://github.com/user-attachments/assets/ccf2474a-6c85-491e-aa48-fde32683ebc5)

## Package(Ours contribution)

## **convert_odom**
This package processes data received from the `microros` node and provides the following functionalities:
- **Command Velocity Handling**:
  - Sends `cmd_vel` to control the robot.
- **Sensor Data Filtering**:
  - Filters raw sensor data before feeding it into the EKF.
- **Data Transformation**:
  - Maps the processed data into formats suitable for other packages.

---

## **navigate_NUC**
This package handles:
- **Scheduler with Finite State Machine (FSM)**:
  - Implements a state machine to control the robot's mode (manual or auto).
  - Provides two services:
    1. `is_nuc_auto`: Indicates whether the robot is in auto or manual mode.
    2. `nuc_goal_pose`: Receives and sets the robot's goal pose.
    - **Example Usage**:
      ```bash
      ros2 service call /is_nuc_auto std_srvs/srv/SetBool "{data: true}"
      ros2 service call /nuc_goal_pose geometry_msgs/srv/PoseStamped "{pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}"
      ```

- **Navigation Configuration**:
  - Includes all necessary configurations for the `nav2` package.

---

## **teleop**
This package manages input devices for controlling the robot:
- **Joystick Controller**:
  - Processes commands from a xbox360 joystick to send `cmd_vel`.
- **Keyboard Control**:
  - Maps keyboard inputs to `cmd_vel` for manual robot operation.



# Validation

The source code and usage instructions are provided in the `Validation` branch


https://github.com/user-attachments/assets/45dccd1d-701b-4f95-9d32-96d47707a340

From the trajectory results from a 1×1 m path, with an error margin of within 0.2 m.


# Extentions CLI
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
### Demo VDO

Lidar slam : https://youtu.be/QLqYQUTp1BY

NUC demo : https://www.youtube.com/watch?v=u4htunHGWko

### Future Plan

We will explore ways to improve the robot's localization accuracy by researching localization techniques, sensor fusion, and SLAM

### Acknowledgments
This project relies on the following libraries and tools:
 - robot_localization - For state estimation and sensor fusion.
 - unitree_lidar - Integration and support for Unitree LiDAR sensors.
 - micro-ROS - ROS 2 framework for microcontrollers.
 - Nav2 - Navigation stack for autonomous robots.
