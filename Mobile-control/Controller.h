#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "stdint.h"

typedef struct {
  float Ke;
  float Kt;
  float L;
  float R;
  float J;
  float B;
  float V_max;
  float U_max;
  float qdd_max;
} MotorConstant_Structure;

int32_t PWM_Satuation(float _u, int32_t _upper_limit, int32_t _lower_limit);

class PID_CONTROLLER {
  private:

  float Kp = 0;
  float Ki = 0;
  float Kd = 0;

  float ek_1 = 0;
  float ek_2 = 0;

  float u = 0;
  float u_max = 0;

  public:

  PID_CONTROLLER(float _Kp, float _Ki, float _Kd, float _u_max);

  float Compute(float ek);

};

class DC_MOTOR_FFD {
  private:

  MotorConstant_Structure *Mx;

  public:

  float Vmax;
  float Umax;
  float qddmax;

  DC_MOTOR_FFD(MotorConstant_Structure *_Mx);

  float Compute(float qd, float i);

};

#endif