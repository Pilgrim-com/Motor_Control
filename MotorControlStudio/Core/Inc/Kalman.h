#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "arm_math.h"

typedef struct {

    // zeros and identity matrix
    const float32_t zero16_f32[16];
    const float32_t zero4_f32[4];

    // Matrix A
    arm_matrix_instance_f32 A;
    arm_matrix_instance_f32 AX;
    arm_matrix_instance_f32 A_t; // A transpose
    arm_matrix_instance_f32 AP;
    arm_matrix_instance_f32 APA_t;

    // Matrix B (control input matrix)
    arm_matrix_instance_f32 B;
    arm_matrix_instance_f32 BV;

    // Matrix G (not used but included for consistency)
    float32_t G_f32[4];
    arm_matrix_instance_f32 G;

    // Matrix H
    float32_t H_f32[4];
    arm_matrix_instance_f32 H;
    arm_matrix_instance_f32 HU;
    arm_matrix_instance_f32 HX_pred;
    arm_matrix_instance_f32 HP_pred;
    arm_matrix_instance_f32 HP_predH_t;
    arm_matrix_instance_f32 H_sum;
    arm_matrix_instance_f32 H_t; // H transpose

    // Matrix Y
    arm_matrix_instance_f32 Y;

    // Matrix X (state)
    float32_t X_init_f32[4];
    arm_matrix_instance_f32 X;
    arm_matrix_instance_f32 X_pred;
    arm_matrix_instance_f32 X_update;

    // Matrix P (covariance)
    float32_t P_init_f32[16];
    arm_matrix_instance_f32 P;
    arm_matrix_instance_f32 P_pred;

    // Kalman gain
    arm_matrix_instance_f32 K;
    arm_matrix_instance_f32 KY;
    arm_matrix_instance_f32 KH;

    arm_matrix_instance_f32 I_sum;

    // Control Input
    arm_matrix_instance_f32 Z;

    // Process noise scalar
    float32_t Q_scalar;  // Now a scalar
    // Measurement noise scalar
    float32_t R_scalar;  // Now a scalar

    // Control input parameter (e.g., velocity)
    float32_t V_t;  // assuming control input is a 4x1 vector
    float32_t input_val;

} KALMAN;

void KalmanInit(KALMAN *kalman, float32_t Matrix_A[16], float32_t Matrix_B[4], float32_t Q_scalar, float32_t R_scalar);
void KalmanUpdate(KALMAN *kalman, double measurement);
void KalmanPrediction(KALMAN *kalman, float control_input);

#endif /* INC_KALMAN_H_ */
