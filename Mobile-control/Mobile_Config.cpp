#include "Kinematics.h"
#include "Mobile_Config.h"

QEI enc1(ENC_1_A_PIN, ENC_1_B_PIN, ENC_1_PPR);
QEI enc2(ENC_2_A_PIN, ENC_2_B_PIN, ENC_2_PPR);
QEI enc3(ENC_3_A_PIN, ENC_3_B_PIN, ENC_3_PPR);
QEI enc4(ENC_4_A_PIN, ENC_4_B_PIN, ENC_4_PPR);

ESP32_CYTRON_MD M1(MOTOR_1_PWM_PIN, MOTOR_1_DIR_PIN);
ESP32_CYTRON_MD M2(MOTOR_2_PWM_PIN, MOTOR_2_DIR_PIN);
ESP32_CYTRON_MD M3(MOTOR_3_PWM_PIN, MOTOR_3_DIR_PIN);
ESP32_CYTRON_MD M4(MOTOR_4_PWM_PIN, MOTOR_4_DIR_PIN);
/*-----Config Motor End-----*/

/*-----Config Robot Base Start-----*/
Kinematics kinematics(WHEEL_DIAMETER, LX, LY);
/*-----Config Robot Base End-----*/

/*-----Config Kalman Start-----*/
KalmanFilter kf1(CYTRON_MOTOR_260RPM_250W_MatrixA,
                 CYTRON_MOTOR_260RPM_250W_MatrixB,
                 CYTRON_MOTOR_260RPM_250W_MatrixC,
                 CYTRON_MOTOR_260RPM_250W_MatrixQ,
                 CYTRON_MOTOR_260RPM_250W_MatrixR);
KalmanFilter kf2(CYTRON_MOTOR_260RPM_250W_MatrixA,
                 CYTRON_MOTOR_260RPM_250W_MatrixB,
                 CYTRON_MOTOR_260RPM_250W_MatrixC,
                 CYTRON_MOTOR_260RPM_250W_MatrixQ,
                 CYTRON_MOTOR_260RPM_250W_MatrixR);
KalmanFilter kf3(CYTRON_MOTOR_260RPM_250W_MatrixA,
                 CYTRON_MOTOR_260RPM_250W_MatrixB,
                 CYTRON_MOTOR_260RPM_250W_MatrixC,
                 CYTRON_MOTOR_260RPM_250W_MatrixQ,
                 CYTRON_MOTOR_260RPM_250W_MatrixR);
KalmanFilter kf4(CYTRON_MOTOR_260RPM_250W_MatrixA,
                 CYTRON_MOTOR_260RPM_250W_MatrixB,
                 CYTRON_MOTOR_260RPM_250W_MatrixC,
                 CYTRON_MOTOR_260RPM_250W_MatrixQ,
                 CYTRON_MOTOR_260RPM_250W_MatrixR);
/*-----Config Kalman End-----*/

/*-----Config Controller Start-----*/
PID_CONTROLLER pid1(CYTRON_MOTOR_260RPM_250W_VElOCITY_KP,
                    CYTRON_MOTOR_260RPM_250W_VElOCITY_KI,
                    CYTRON_MOTOR_260RPM_250W_VElOCITY_KD,
                    CYTRON_MOTOR_260RPM_250W_Constant.U_max);
PID_CONTROLLER pid2(CYTRON_MOTOR_260RPM_250W_VElOCITY_KP,
                    CYTRON_MOTOR_260RPM_250W_VElOCITY_KI,
                    CYTRON_MOTOR_260RPM_250W_VElOCITY_KD,
                    CYTRON_MOTOR_260RPM_250W_Constant.U_max);
PID_CONTROLLER pid3(CYTRON_MOTOR_260RPM_250W_VElOCITY_KP,
                    CYTRON_MOTOR_260RPM_250W_VElOCITY_KI,
                    CYTRON_MOTOR_260RPM_250W_VElOCITY_KD,
                    CYTRON_MOTOR_260RPM_250W_Constant.U_max);
PID_CONTROLLER pid4(CYTRON_MOTOR_260RPM_250W_VElOCITY_KP,
                    CYTRON_MOTOR_260RPM_250W_VElOCITY_KI,             
                    CYTRON_MOTOR_260RPM_250W_VElOCITY_KD,
                    CYTRON_MOTOR_260RPM_250W_Constant.U_max);

DC_MOTOR_FFD ffd1(&CYTRON_MOTOR_260RPM_250W_Constant);
DC_MOTOR_FFD ffd2(&CYTRON_MOTOR_260RPM_250W_Constant);
DC_MOTOR_FFD ffd3(&CYTRON_MOTOR_260RPM_250W_Constant);
DC_MOTOR_FFD ffd4(&CYTRON_MOTOR_260RPM_250W_Constant);
/*-----Config Controller Start-----*/

ESP32_CYTRON_MD* Mx[4] = { &M1, &M2, &M3, &M4 };
QEI* encx[4] = { &enc1, &enc2, &enc3, &enc4 };
PID_CONTROLLER* pidx[4] = { &pid1, &pid2, &pid3, &pid4 };
DC_MOTOR_FFD* ffdx[4] = { &ffd1, &ffd2, &ffd3, &ffd4 };
KalmanFilter* kfx[4] = { &kf1, &kf2, &kf3, &kf4 };
Kinematics* kin = &kinematics;
