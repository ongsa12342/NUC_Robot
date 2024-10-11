#include "Mobile_Config.h"
#include "Wire.h"
#include "esp32-hal.h"
#include "Mobile_command.h"

Mobile_command::Mobile_command(ESP32_CYTRON_MD* _Mx[], QEI* _encx[], PID_CONTROLLER* _pidx[], DC_MOTOR_FFD* _ffdx[], KalmanFilter* _kfx[], Kinematics* _kinematics)
  : kinematics(_kinematics) {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    Mx[i] = _Mx[i];
    encx[i] = _encx[i];
    pidx[i] = _pidx[i];
    ffdx[i] = _ffdx[i];
    kfx[i] = _kfx[i];
    // Serial.println(encx[i]->);
  }
}

void Mobile_command::begin() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    encx[i]->begin();
  }
  
  delay(1000);

  for (int i = 0; i < NUM_MOTORS; i++) {
    Mx[i]->begin();
    Mx[i]->set_duty(0);

    cmd_ux[i] = 0;
    fb_q[i] = 0;
    fb_qd[i] = 0;
    fb_i[i] = 0;

    encx[i]->reset();
    kfx[i]->begin();
  }

  delay(1000);

  // Adafruit_BNO08x bno08x(-1);
  //  Wire.begin(15, 16);  // Use the custom SDA and SCL pins
  // //  Serial2.begin(115200);
  // if (!bno08x.begin_I2C(0x4A, &Wire)) {
  //   // Serial.println("Failed to find BNO08x chip");
  //   while (1) {
  //     delay(10);
  //   }
  // }
  // Serial.println("BNO08x Found!");


  // sh2_SensorValue_t sensorValue;
  // Wire.begin(ADC_SDA, ADC_SCL, 400000);

  // Wire1.begin(BNO_SDA, BNO_SCL);
  // while (!Serial) {
  //   delay(10);  // Wait until serial console opens
  // }
  // if (!bno.begin_I2C(0x4A, &Wire)) {
  //   Serial.println("Failed to find BNO08x chip");
  //   while (1) {
  //     delay(10);
  //   }
  // }

  // Wire1.setPins(BNO_SDA, BNO_SCL);
  // Wire1.setClock(400000);
  // Wire1.setTimeOut(10);
  // if (!bno.begin_I2C(0x4A, &Wire)) {
  //   Serial.println("Failed to find BNO08x chip");
  //   while (1) { delay(10); }
  // }
  // bno.begin_I2C(0x4A, &Wire);
    // while (!bno.begin_I2C()) delay(1);
  // bno.setExtCrystalUse(true);

  // adafruit_bno055_offsets_t calibrationData;
  // calibrationData.accel_offset_x = -40;
  // calibrationData.accel_offset_y = 71;
  // calibrationData.accel_offset_z = -33;
  // calibrationData.gyro_offset_x = 1;
  // calibrationData.gyro_offset_y = 3;
  // calibrationData.gyro_offset_z = 0;
  // calibrationData.mag_offset_z = -33;
  // calibrationData.mag_offset_x = 1;
  // calibrationData.mag_offset_y = 3;
  // calibrationData.accel_radius = 1000;
  // calibrationData.mag_radius = 870;
  // bno.setSensorOffsets(calibrationData);
  // delay(1000);
}

void Mobile_command::control(float _vx, float _vy, float _wz) {
  Kinematics::RadPS wheel_radps = kinematics->Inverse_Kinematics(_vx, _vy, _wz);

  ramp(wheel_radps.radps_fl, 0);
  ramp(wheel_radps.radps_fr, 1);
  ramp(wheel_radps.radps_bl, 2);
  ramp(wheel_radps.radps_br, 3);

  for (int i = 0; i < NUM_MOTORS; i++) {
    fb_q[i] += encx[i]->get_diff_count() * 2 * M_PI / encx[i]->pulse_per_rev;
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    float* kf_ptr = kfx[i]->Compute(fb_q[i],
                                    cmd_ux[i] * ffdx[i]->Vmax / ffdx[i]->Umax);
    fb_qd[i] = kf_ptr[1];
    fb_i[i] = (1000 * fb_i[i] / 1012.0) + (12.0 * kf_ptr[3] / 1012.0);

    if (qd_target[i] != 0) {
      cmd_ux[i] = PWM_Satuation(pidx[i]->Compute(qd_target[i] - fb_qd[i]) + ffdx[i]->Compute(qd_target[i], fb_i[i]),
                                ffdx[i]->Umax,
                                -1 * ffdx[i]->Umax);
    } else {
      cmd_ux[i] = 0;
    }
  }

  // for (int i = 0; i < NUM_MOTORS; i++) {
  //   Mx[i]->set_duty(cmd_ux[i]);
  // }

  for (int i = 0; i < NUM_MOTORS; i++) {
    if(i == 1 || i == 3)
    {
      Mx[i]->set_duty(-1*cmd_ux[i]);
    }
    else{
      Mx[i]->set_duty(cmd_ux[i]);
    }
  }
}

IMU_DATA Mobile_command::getIMU() {
  IMU_DATA imu_data;
  // sensors_event_t angVelocityData, linearAccelData;
  // sensors_event_t angVelocityData, linearAccelData;
  // if (bno.getSensorEvent(&sensorValue)) {
  // Serial2.println(sensorValue.sensorId);
  // if (!bno08x.getSensorEvent(&sensorValue)) {
  //   return imu_data;  // Return empty data if no event is available
  // }

  // // Parse the sensor data depending on the event type
  // switch (sensorValue.sensorId) {
  //   case SH2_GYROSCOPE_CALIBRATED:
  //     imu_data.gyro.x = sensorValue.un.gyroscope.x;
  //     imu_data.gyro.y = sensorValue.un.gyroscope.y;
  //     imu_data.gyro.z = sensorValue.un.gyroscope.z;
  //     break;
  //   case SH2_LINEAR_ACCELERATION:
  //     imu_data.accel.x = sensorValue.un.linearAcceleration.x;
  //     imu_data.accel.y = sensorValue.un.linearAcceleration.y;
  //     imu_data.accel.z = sensorValue.un.linearAcceleration.z;
  //     break;
  //   case SH2_ROTATION_VECTOR:
  //     imu_data.quat.x = sensorValue.un.rotationVector.i;
  //     imu_data.quat.y = sensorValue.un.rotationVector.j;
  //     imu_data.quat.z = sensorValue.un.rotationVector.k;
  //     imu_data.quat.w = sensorValue.un.rotationVector.real;
  //     break;
  // }

  // return imu_data;

  // if (bno08x.getSensorEvent(&sensorValue)) {
  // switch (sensorValue.sensorId) {
  //     case SH2_GYROSCOPE_CALIBRATED:
  //       // imu_data.gyro.x = sensorValue.un.gyroscope.x;
  //       imu_data.gyro.x = 102.23f;
  //       imu_data.gyro.y = sensorValue.un.gyroscope.y;
  //       imu_data.gyro.z = sensorValue.un.gyroscope.z;
  //       break;
  //     case SH2_LINEAR_ACCELERATION:
  //       imu_data.accel.x = sensorValue.un.linearAcceleration.x;
  //       imu_data.accel.y = sensorValue.un.linearAcceleration.y;
  //       imu_data.accel.z = sensorValue.un.linearAcceleration.z;
  //       break;
  //     case SH2_ROTATION_VECTOR:
  //       imu_data.quat.x = sensorValue.un.rotationVector.i;
  //       imu_data.quat.y = sensorValue.un.rotationVector.j;
  //       imu_data.quat.z = sensorValue.un.rotationVector.k;
  //       imu_data.quat.w = sensorValue.un.rotationVector.real;
  //       break;
  //     case 0:
  //       imu_data.gyro.x = 2.123f;
  //       break;
  //     default:
  //       imu_data.gyro.x = 232.123f;
  //       break;
  //   }
  // }

    // imu_data.gyro.x = 100.213f;
  // }
  // switch (sensorValue.sensorId) {
  //   case SH2_GYROSCOPE_CALIBRATED:
  //     imu_data.gyro.x = sensorValue.un.gyroscope.x;
  //     imu_data.gyro.y = sensorValue.un.gyroscope.y;
  //     imu_data.gyro.z = sensorValue.un.gyroscope.z;
  //     break;
  //   case SH2_LINEAR_ACCELERATION:
  //     imu_data.accel.x = sensorValue.un.linearAcceleration.x;
  //     imu_data.accel.y = sensorValue.un.linearAcceleration.y;
  //     imu_data.accel.z = sensorValue.un.linearAcceleration.z;
  //     break;
  //   case SH2_ROTATION_VECTOR:
  //     imu_data.quat.x = sensorValue.un.rotationVector.i;
  //     imu_data.quat.y = sensorValue.un.rotationVector.j;
  //     imu_data.quat.z = sensorValue.un.rotationVector.k;
  //     imu_data.quat.w = sensorValue.un.rotationVector.real;
  //     break;
  // }
  // bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  // bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  // imu::Quaternion quat = bno.getQuat();

  // imu_data.quat.x = quat.x();
  // imu_data.quat.y = quat.y();
  // imu_data.quat.z = quat.z();
  // imu_data.quat.w = quat.w();


  // imu_data.gyro.x = angVelocityData.gyro.x;
  // imu_data.gyro.y = angVelocityData.gyro.y;
  // imu_data.gyro.z = angVelocityData.gyro.z;

  // imu_data.accel.x = linearAccelData.acceleration.x;
  // imu_data.accel.y = linearAccelData.acceleration.y;
  // imu_data.accel.z = linearAccelData.acceleration.z;

  return imu_data;
}

ODOM_DATA Mobile_command::getODOM() {
  Kinematics::Velocity robot_odom = kinematics->Forward_Kinematics_Velocity(fb_qd[0], fb_qd[1], fb_qd[2], fb_qd[3]);
  ODOM_DATA odom = {
    .vx = robot_odom.vx,
    .vy = robot_odom.vy,
    .wz = robot_odom.wz
  };
  return odom;
}

void Mobile_command::ramp(float set_target, uint8_t index) {
  static uint32_t timestamp[NUM_MOTORS] = { 0 };
  static float target[NUM_MOTORS] = { 0 };
  if (set_target != target[index]) {
    timestamp[index] = millis() + (abs(set_target - fb_qd[index]) * 1000.0 / ffdx[index]->qddmax);
    target[index] = set_target;
  }
  if (millis() < timestamp[index]) {
    if (qd_target[index] > target[index]) qd_target[index] -= ffdx[index]->qddmax * dt;
    else if (qd_target[index] < target[index]) qd_target[index] += ffdx[index]->qddmax * dt;
  } else {
    qd_target[index] = target[index];
  }
}
