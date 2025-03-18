#ifndef __CYTRON_MOTOR_260RPM_250W_H__
#define __CYTRON_MOTOR_260RPM_250W_H__

#include "stdint.h"
#include "Controller.h"

#define CYTRON_MOTOR_260RPM_250W_VElOCITY_KP 10000.0
#define CYTRON_MOTOR_260RPM_250W_VElOCITY_KI 1.0
#define CYTRON_MOTOR_260RPM_250W_VElOCITY_KD 0.0

#define AMT103_PPR 2048.0 * 4.0

extern MotorConstant_Structure CYTRON_MOTOR_260RPM_250W_Constant;

extern float CYTRON_MOTOR_260RPM_250W_MatrixA[];

//input matrix
extern float CYTRON_MOTOR_260RPM_250W_MatrixB[];

// observation matrix
extern float CYTRON_MOTOR_260RPM_250W_MatrixC[];

//process model variance vaule
extern float CYTRON_MOTOR_260RPM_250W_MatrixQ[];

// measurement covariance matrix
extern float CYTRON_MOTOR_260RPM_250W_MatrixR[];

#endif