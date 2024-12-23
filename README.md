
# NUC_Robot Validation

This repository contains the setup and instructions method for validate the NUC_Robot position with realsense camera

<video controls src="470704536_8675603952517412_4196081051933534043_n.mp4" title="Title"></video>

## Installation and Setup

### Step 1: Clone the Repository
```
git clone https://github.com/ongsa12342/NUC_Robot.git -b validation
```

### Step 2: Make sure your already have this library on you device
- python 3.10.11 (for realsense camera)
- pyrealsense2 (for realsense camera)
- opencv python
- matplot lib

### Step 3: Config to fit your environment
To set up the environment, config the following:

- camera

![alt text](image.png)

- camera matrix and dish_coeffs

![alt text](image-1.png)

- aruco type and size 

![alt text](image-2.png)


### Step 4: Running the validation code
```
python Realsense_aruco.py
```
### 





