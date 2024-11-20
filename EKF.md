
# EKF launch

This package use for EKF sensor fusion (IMU data & robot velocity) and sent odom to create path in rviz2 

This launch file contain Lowpass filter(imu data) and convert datatype float_multi_array (robot velocity) to odometry datatype for fusion in EKF node.

### Step 1: Launch EKF
```
cd ~/NUC_Robot/
colcon build && source install/setup.bash 
```

### Step 2: Launch EKF
```
ros2 launch convert_odom simple.launch.py 
```

### Step 3: Visualize on Rviz 
```
rviz2 -d src/convert_odom/config/path.rviz
```
