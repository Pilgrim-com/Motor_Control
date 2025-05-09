#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "arm_math.h"
#include "string.h"

typedef struct {

    // Constant
    //Matrix A
    float32_t A_f32[16];
    float32_t A_t_f32[16];
    arm_matrix_instance_f32 A;
    arm_matrix_instance_f32 A_t;

    //Matrix B
    float32_t B_f32[16];
    arm_matrix_instance_f32 B;

    //Matrix G
    float32_t G_f32[4];
    arm_matrix_instance_f32 G;

    //Matrix H
    float32_t H_f32[4];
    float32_t H_t_f32[4];
    arm_matrix_instance_f32 H;
    arm_matrix_instance_f32 H_t;

    float32_t I_f32[16];
    arm_matrix_instance_f32 I;

    // Parameter
    //Matrix Z
    float32_t Z_f32[1];
    arm_matrix_instance_f32 Z;

    //Matrix X
    float32_t X_f32[4];
    float32_t X_pred_f32[4];
    arm_matrix_instance_f32 X;
    arm_matrix_instance_f32 X_pred;

    //Matrix P
    float32_t P_f32[16];
    float32_t P_pred_f32[16];
    arm_matrix_instance_f32 P;
    arm_matrix_instance_f32 P_pred;

    //Matrix K (Kalman gain)
    float32_t K_f32[4];
    arm_matrix_instance_f32 K;

    // Parameter
    //Process noise
    float32_t Q_f32[16];
    arm_matrix_instance_f32 Q;
    //Measurement noise scalar
    float32_t R_f32[16];
    arm_matrix_instance_f32 R;

    //Control input
    float32_t U_f32[1];
    arm_matrix_instance_f32 U;

} KALMAN;

void KalmanInit(KALMAN *kalman, float32_t Matrix_A[16], float32_t Matrix_B[4], float32_t Q_scalar, float32_t R_scalar);
void KalmanUpdate(KALMAN *kalman, double measurement);
void KalmanPrediction(KALMAN *kalman, float control_input);

#endif /* INC_KALMAN_H_ */
