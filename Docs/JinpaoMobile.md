
# ESP32-S3-Pico


# PinMap

pin configuration for controlling four motors using the ESP32-S3-Pico along with encoder inputs and an IMU.

## MotorX Configuration (PWM, DIR)

- **Motor1:**  
  - PWM: `GP39`  
  - DIR: `GP40`

- **Motor2:**  
  - PWM: `GP42`  
  - DIR: `GP41`

- **Motor3:**  
  - PWM: `GP1`  
  - DIR: `GP2`

- **Motor4:**  
  - PWM: `GP4`  
  - DIR: `GP5`

## EncoderX Configuration (A, B)

- **Encoder1:**  
  - A: `GP17`  
  - B: `GP18`

- **Encoder2:**  
  - A: `GP33`  
  - B: `GP34`

- **Encoder3:**  
  - A: `GP35`  
  - B: `GP36`

- **Encoder4:**  
  - A: `GP37`  
  - B: `GP38`

## IMU Configuration (I2C)

- **IMU:**  
  - SDA: `GP15`  
  - SCL: `GP16`

---

This configuration is used for controlling the motors and reading encoder values for feedback, along with IMU data, using the ESP32-S3-Pico microcontroller.
