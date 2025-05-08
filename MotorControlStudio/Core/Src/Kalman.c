#include "Kalman.h"
#include "QEI.h"

// Initialize Kalman filter matrices and parameters
void KalmanInit(KALMAN *kalman, float32_t Matrix_A[16], float32_t Matrix_B[4], float32_t Q_scalar, float32_t R_scalar)
{
    // Initialize matrices A, B, G, H
    arm_mat_init_f32(&kalman->A, 4, 4, Matrix_A);
    arm_mat_init_f32(&kalman->B, 4, 1, Matrix_B);  // Control matrix
    float32_t G_init[4] = {0.0, 0.0, 1.0, 0.0};
    memcpy(kalman->G_f32, G_init, sizeof(G_init));
    float32_t H_init[4] = {1.0, 0.0, 0.0, 0.0};
    memcpy(kalman->H_f32, H_init, sizeof(H_init));
    arm_mat_init_f32(&kalman->G, 4, 1, kalman->G_f32);
    arm_mat_init_f32(&kalman->H, 1, 4, kalman->H_f32);

    // Transpose matrices A and H
    arm_mat_init_f32(&kalman->A_t, 4, 4, (float *) {0});
    arm_mat_trans_f32(&kalman->A, &kalman->A_t);

    arm_mat_init_f32(&kalman->H_t, 1, 4, (float *) {0});
    arm_mat_trans_f32(&kalman->H, &kalman->H_t);

    // Initialize state variables (position and velocity)
    kalman->X_init_f32[0] = 1;
    kalman->X_init_f32[1] = 1;

    // Initialize covariance matrices
    arm_mat_init_f32(&kalman->X, 4, 1, (float *) {0});
    arm_mat_init_f32(&kalman->X_pred, 4, 1, (float *) {0});
    arm_mat_init_f32(&kalman->P, 4, 4, (float *) {0});
    arm_mat_init_f32(&kalman->P_pred, 4, 4, (float *) {0});

    // Kalman parameters initialization
    arm_mat_init_f32(&kalman->K, 4, 1, (float *) {0});
    arm_mat_init_f32(&kalman->Z, 1, 1, (float *) {0});

    // Assign scalar values to Q and R
    kalman->Q_scalar = Q_scalar;
    kalman->R_scalar = R_scalar;

    // Control input initialization
    arm_mat_init_f32(&kalman->V_t, 4, 1, (float *) {0});  // Assuming control input is 4x1 vector
}

// Prediction Step
void KalmanPrediction(KALMAN *kalman, float32_t control_input)
{
    // Convert the scalar control input to a 4x1 matrix
	float32_t V_t_f32[4] = {control_input};

    arm_matrix_instance_f32 control_input_matrix;
    arm_mat_init_f32(&control_input_matrix, 4, 1, V_t_f32);  // Initialize control input matrix



    // B * V_t
    arm_mat_mult_f32(&kalman->B, &control_input_matrix, &kalman->BV);

    // Predicted state: X_pred = A * X + B * V_t
    arm_mat_mult_f32(&kalman->A, &kalman->X, &kalman->AX);
    arm_mat_add_f32(&kalman->AX, &kalman->BV, &kalman->X_pred);  // Add control input contribution

    // Create Q matrix (diagonal) from scalar Q_scalar
    float Q_f32[16] =
    {
        kalman->Q_scalar, 0, 0, 0,
        0, kalman->Q_scalar, 0, 0,
        0, 0, kalman->Q_scalar, 0,
        0, 0, 0, kalman->Q_scalar
    };

    arm_matrix_instance_f32 Q_matrix;
    arm_mat_init_f32(&Q_matrix, 4, 4, Q_f32);

    // Predict the covariance: P_pred = A * P * A' + Q
    arm_mat_mult_f32(&kalman->A, &kalman->P, &kalman->AP);  // A * P
    arm_mat_mult_f32(&kalman->AP, &kalman->A_t, &kalman->APA_t);  // A * P * A'
    arm_mat_add_f32(&kalman->APA_t, &Q_matrix, &kalman->P_pred);  // Add Q (process noise matrix)
}


// Update Step
void KalmanUpdate(KALMAN *kalman, double measurement)
{
    // Set the measurement Z (scalar)
    kalman->Z.pData[0] = measurement;  // Assume measurement is a scalar (position in radians)

    // Innovation (Residual): y = Z - H * X_pred
    arm_mat_mult_f32(&kalman->H, &kalman->X_pred, &kalman->HX_pred);  // y = Z - H * X_pred
    arm_mat_sub_f32(&kalman->Z, &kalman->HX_pred, &kalman->Y);

    // Create R matrix (diagonal) from scalar R_scalar
    arm_matrix_instance_f32 R_matrix;
    float R_f32[1] = {kalman->R_scalar};
    arm_mat_init_f32(&R_matrix, 1, 1, R_f32);
    R_matrix.pData[0] = kalman->R_scalar;  // R is a scalar (1x1 matrix)

    // Kalman Gain: K = P_pred * H' * (H * P_pred * H' + R)^-1
    // Calculate S = H * P_pred * H' + R
    arm_mat_mult_f32(&kalman->H, &kalman->P_pred, &kalman->HP_predH_t);  // H * P_pred
    arm_mat_trans_f32(&kalman->H, &kalman->H_t);  // H transpose
    arm_mat_mult_f32(&kalman->HP_predH_t, &kalman->H_t, &kalman->HP_predH_t);  // H * P_pred * H'
    arm_mat_add_f32(&kalman->HP_predH_t, &R_matrix, &kalman->H_sum);  // Add R (measurement noise matrix)

    // Calculate Kalman Gain: K = P_pred * H' * S^-1
    arm_mat_inverse_f32(&kalman->H_sum, &kalman->H_sum);  // Inverse of H sum
    arm_mat_mult_f32(&kalman->P_pred, &kalman->H_t, &kalman->HP_pred);  // P_pred * H'
    arm_mat_mult_f32(&kalman->HP_pred, &kalman->H_sum, &kalman->K);  // Kalman Gain

    // Update the state estimate: X = X_pred + K * y
    arm_mat_mult_f32(&kalman->K, &kalman->Y, &kalman->KY);  // K * y
    arm_mat_add_f32(&kalman->X_pred, &kalman->KY, &kalman->X);  // X = X_pred + K * y

    // Update the covariance estimate: P = (I - K * H) * P_pred
    arm_mat_mult_f32(&kalman->K, &kalman->H, &kalman->KH);  // K * H
    arm_matrix_instance_f32 I;
    float I_data[16] = {0};
    arm_mat_init_f32(&I, 4, 4, I_data);
    for (int i = 0; i < 4; i++) I_data[i * 4 + i] = 1.0f;
    arm_mat_sub_f32(&I, &kalman->KH, &kalman->I_sum);  // I - K * H
    arm_mat_mult_f32(&kalman->I_sum, &kalman->P_pred, &kalman->P);  // (I - K * H) * P_pred
}
