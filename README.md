
# NUC_Robot

This repository contains the setup and launch instructions for the NUC_Robot project, which includes support for the Unitree LiDAR sensor.

## Installation and Setup

### Step 1: Clone the Repository
```
git clone https://github.com/ongsa12342/NUC_Robot.git
```

### Step 2: Build the Package
Navigate to the cloned repository and build it using `colcon`:
```
cd NUC_Robot && colcon build
```

### Step 3: Source the Setup File
To set up the environment, source the following:
```
source ~/NUC_Robot/install/setup.bash
```

### Step 4: (Optional) Add to .bashrc
For convenience, you may want to add the setup file to your `.bashrc`:
```
echo "source ~/NUC_Robot/install/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

## Running the Unitree LiDAR
To launch the Unitree LiDAR with ROS 2:
```
ros2 launch unitree_lidar_ros2 launch.py
```

### Visualize in RViz
To open RViz with a preset configuration for the Unitree LiDAR:
```
rviz2 -d src/unitree_lidar_ros2/rviz/view.rviz
```
