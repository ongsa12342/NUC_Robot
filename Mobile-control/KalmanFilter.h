#include "stdint.h"
/*
 * kalmanfilter.h
 *
 *  Created on: 15 เม.ย. 2565
 *      Author: weera
 */

#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

#include "Matrix.h"

class KalmanFilter {
private:

  matrix A, B, C, D, R, G, Q;
  matrix gainK, errorY, U, Y, I33;
  matrix P, P_old, P_new;
  matrix predictX, predictX_old, predictX_new;
  matrix resultX, resultY;

  float updated_U[1] = { 0 };
  float updated_Ymeas[1] = { 0 };

  float G_data[4] = { 0.0, 1.0, 0.0, 0.0 };

  float estimateState[4] = { 0.0, 0.0, 0.0, 0.0 };

  void doKalman_gain();
  void doCorrect_p();
  void doPredict_y();
  void doCorrect();
  void doPredict_x();
  void doPredict_p();
  void doResult();
  void run();

public:

  KalmanFilter(float *_A_data, float *_B_data, float *_C_data, float *_Q_data, float *_R_data);
  void begin();
  float *Compute(double _q, float _Vin);
};

#endif /* INC_KALMANFILTER_H_ */
