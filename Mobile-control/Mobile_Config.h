#ifndef __MOBILE_CONFIG_H__
#define __MOBILE_CONFIG_H__

#include "QEI.h"
#include "Kinematics.h"
#include "KalmanFilter.h"
#include "Controller.h"
#include "Esp32_Cytron_MDxx.h"

#include "Cytron_Motor_260rpm_250W.h"

/*-----Config ADC Start-----*/
#define ADC_SDA 13
#define ADC_SCL 14
/*-----Config ADC End-----*/

/*-----Config BNO Start-----*/
#define BNO_SDA 15
#define BNO_SCL 16
/*-----Config BNO End-----*/

/*-----Config Motor Start-----*/
#define MOTOR_1_PWM_PIN 39
#define MOTOR_2_PWM_PIN 42
#define MOTOR_3_PWM_PIN 1
#define MOTOR_4_PWM_PIN 4

#define MOTOR_1_DIR_PIN 40
#define MOTOR_2_DIR_PIN 41
#define MOTOR_3_DIR_PIN 2
#define MOTOR_4_DIR_PIN 5

#define ENC_1_A_PIN 18
#define ENC_1_B_PIN 17
#define ENC_2_A_PIN 33
#define ENC_2_B_PIN 34
#define ENC_3_A_PIN 36
#define ENC_3_B_PIN 35
#define ENC_4_A_PIN 37
#define ENC_4_B_PIN 38

#define ENC_1_PPR 2048.0 * 4.0
#define ENC_2_PPR 2048.0 * 4.0
#define ENC_3_PPR 2048.0 * 4.0
#define ENC_4_PPR 2048.0 * 4.0
/*-----Config Motor End-----*/

/*-----Config Robot Base Start-----*/
#define WHEEL_DIAMETER 0.1524  // [m]
#define LX 0.2105              // [m]
#define LY 0.31                // [m]
#define LZ 0.306               // [m]
/*-----Config Robot Base End-----*/

/*-----Config Kalman Start-----*/

/*-----Config Kalman End-----*/

/*-----Config Controller Start-----*/

/*-----Config Controller Start-----*/

extern ESP32_CYTRON_MD* Mx[];
extern QEI* encx[];
extern PID_CONTROLLER* pidx[];
extern DC_MOTOR_FFD* ffdx[];
extern KalmanFilter* kfx[];
extern Kinematics* kin;

#endif