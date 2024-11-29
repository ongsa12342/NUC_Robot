#include "esp32-hal.h"
#ifndef MOBILE_COMMAND_H
#define MOBILE_COMMAND_H

#include "Arduino.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "Mobile_Config.h"
#include "Wire.h"

typedef struct {
  float x;
  float y;
  float z;
  float w;
} Axis_Structure;

typedef struct {
  Axis_Structure quat;
  Axis_Structure gyro;
  Axis_Structure accel;
} IMU_DATA;

typedef struct {
  float vx;
  float vy;
  float wz;
} ODOM_DATA;

class Mobile_command {
private:
  static const int NUM_MOTORS = 4;  // Number of motors, encoders, and PIDs
  float dt = 1 / 1000.0;

  Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire1);

  ESP32_CYTRON_MD* Mx[NUM_MOTORS];
  QEI* encx[NUM_MOTORS];
  PID_CONTROLLER* pidx[NUM_MOTORS];
  DC_MOTOR_FFD* ffdx[NUM_MOTORS];
  KalmanFilter* kfx[NUM_MOTORS];
  Kinematics* kinematics;

  void ramp(float set_target, uint8_t index);

public:

  float qd_target[NUM_MOTORS];

  int16_t cmd_ux[NUM_MOTORS];
  double fb_q[NUM_MOTORS];
  float fb_qd[NUM_MOTORS];
  float fb_i[NUM_MOTORS];

  Mobile_command(ESP32_CYTRON_MD* _Mx[], QEI* _encx[], PID_CONTROLLER* _pidx[], DC_MOTOR_FFD* _ffdx[], KalmanFilter* _kfx[], Kinematics* _kinematics);

  void begin();

  void control(float _vx, float _vy, float _wz);

  IMU_DATA getIMU();

  ODOM_DATA getODOM();
};

#endif
