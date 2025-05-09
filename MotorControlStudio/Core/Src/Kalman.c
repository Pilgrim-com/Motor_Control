#include "Kalman.h"
#include "QEI.h"

// Initialize Kalman filter matrices and parameters
void KalmanInit(KALMAN *kalman, float32_t Matrix_A[16], float32_t Matrix_B[4], float32_t Q_scalar, float32_t R_scalar)
{
	// set constance

	//Matrix A
	for (int i = 0; i < 16; i++)
	{
		kalman->A_f32[i] = Matrix_A[i];
	}
	arm_mat_init_f32(&kalman->A, 4, 4, kalman->A_f32);
	//Matrix A transpose
	arm_mat_init_f32(&kalman->A_t, 4, 4, kalman->A_t_f32);
	arm_mat_trans_f32(&kalman->A, &kalman->A_t);

	//Matrix B
	for(int i = 0; i<4; i++)
	{
		kalman->B_f32[i] = Matrix_B[i];
	}
	arm_mat_init_f32(&kalman->B, 4, 1, kalman->B_f32);

	//Matrix G
	float32_t G_new[4] = {0, 0, 1, 0};
	memcpy(kalman->G_f32, G_new, sizeof(G_new));
	arm_mat_init_f32(&kalman->G, 4, 1, kalman->G_f32);

	//Matrix H
	float32_t H_new[4] = {1, 0, 0, 0};
	memcpy(kalman->H_f32, H_new, sizeof(H_new));
	arm_mat_init_f32(&kalman->H, 1, 4, kalman->H_f32);

	//Matrix I
	float32_t I_new[16] =
	{
			1.0, 0.0, 0.0, 0.0,
			0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0,
	};
	memcpy(kalman->I_f32, I_new, sizeof(I_new));
	arm_mat_init_f32(&kalman->I, 4, 4,kalman->I_f32);

	//Measurement
	float32_t Z_new[1] = {0};
	memcpy(kalman->Z_f32, Z_new, sizeof(Z_new));
	arm_mat_init_f32(&kalman->Z, 1, 1, kalman->Z_f32);

	//Matrix X
	float32_t X_init[4] = { 0 };
	memcpy(kalman->X_f32, X_init, sizeof(X_init));
	memcpy(kalman->X_pred_f32, X_init, sizeof(X_init));
	arm_mat_init_f32(&kalman->X, 4, 1, kalman->X_f32);
	arm_mat_init_f32(&kalman->X_pred, 4, 1, kalman->X_pred_f32);

	//Matrix P
	float32_t P_init[16] = {
	    100.0, 0.0, 0.0, 0.0,
	    0.0, 100.0, 0.0, 0.0,
	    0.0, 0.0, 100.0, 0.0,
	    0.0, 0.0, 0.0, 100.0
	};
	memcpy(kalman->P_f32, P_init, sizeof(P_init));
	memcpy(kalman->P_pred_f32, P_init, sizeof(P_init));
	arm_mat_init_f32(&kalman->P, 4, 4, kalman->P_f32);
	arm_mat_init_f32(&kalman->P_pred, 4, 4, kalman->P_pred_f32);

	//Matrix K (kalman gain)
	float32_t K_new[4] = {0};
	memcpy(kalman->K_f32, K_new, sizeof(K_new));
	arm_mat_init_f32(&kalman->K, 4, 1, kalman->K_f32);

	//Matrix Q
	float32_t Q_new[16] =
	{
			Q_scalar, 0.0, 0.0, 0.0,
			0.0, Q_scalar, 0.0, 0.0,
			0.0, 0.0, Q_scalar, 0.0,
			0.0, 0.0, 0.0, Q_scalar
	};
	memcpy(kalman->Q_f32, Q_new, sizeof(Q_new));
	arm_mat_init_f32(&kalman->Q, 4, 4, kalman->Q_f32);

	//MatrixR
	float32_t R_new[1] = {R_scalar};
	memcpy(kalman->R_f32, R_new, sizeof(R_new));
	arm_mat_init_f32(&kalman->R, 1, 1, kalman->R_f32);

	//Matrix of contol input
	float32_t U_new[1] = {0.0};
	memcpy(kalman->U_f32, U_new, sizeof(U_new));
	arm_mat_init_f32(&kalman->U, 1, 1, kalman->U_f32);

}

// Prediction Step
void KalmanPrediction(KALMAN *kalman, float32_t control_input)
{
    // Update control input U
    kalman->U_f32[0] = control_input;
    arm_mat_init_f32(&kalman->U, 1, 1, kalman->U_f32);

    // Temporary matrices for intermediate results
    arm_matrix_instance_f32 temp1, temp2;
    float32_t temp1_data[4], temp2_data[16];

    // Step 1: X_pred = A * X + B * U
    arm_mat_init_f32(&temp1, 4, 1, temp1_data);
    arm_mat_mult_f32(&kalman->A, &kalman->X, &temp1);  // temp1 = A * X
    arm_mat_mult_f32(&kalman->B, &kalman->U, &kalman->X_pred);  // X_pred = B * U
    arm_mat_add_f32(&temp1, &kalman->X_pred, &kalman->X_pred);  // X_pred = A*X + B*U

    // Step 2: P_pred = A * P * A^T + Q
    arm_mat_init_f32(&temp1, 4, 4, temp1_data);
    arm_mat_init_f32(&temp2, 4, 4, temp2_data);
    arm_mat_mult_f32(&kalman->A, &kalman->P, &temp1);  // temp1 = A * P
    arm_mat_mult_f32(&temp1, &kalman->A_t, &temp2);    // temp2 = A * P * A^T
    arm_mat_add_f32(&temp2, &kalman->Q, &kalman->P_pred);  // P_pred = A*P*A^T + Q
}

// Update Step
void KalmanUpdate(KALMAN *kalman, double measurement)
{
    // Update measurement Z
    kalman->Z_f32[0] = measurement;
    arm_mat_init_f32(&kalman->Z, 1, 1, kalman->Z_f32);

    // Temporary matrices
    arm_matrix_instance_f32 temp1, temp2, temp3, H_t;
    float32_t temp1_data[4], temp2_data[4], temp3_data[1], H_t_data[4];

    // Step 1: Compute Kalman Gain K = P_pred * H^T * (H * P_pred * H^T + R)^-1
    arm_mat_init_f32(&H_t, 4, 1, H_t_data);
    arm_mat_trans_f32(&kalman->H, &H_t);  // H_t = H^T

    // Compute S = H * P_pred * H^T + R
    arm_mat_init_f32(&temp1, 1, 4, temp1_data);
    arm_mat_init_f32(&temp2, 1, 1, temp2_data);
    arm_mat_mult_f32(&kalman->H, &kalman->P_pred, &temp1);  // temp1 = H * P_pred
    arm_mat_mult_f32(&temp1, &H_t, &temp2);                // temp2 = H * P_pred * H^T
    arm_mat_add_f32(&temp2, &kalman->R, &temp2);           // temp2 = S = H*P_pred*H^T + R

    // Compute K = P_pred * H^T * inv(S)
    float32_t S_inv_data[1];
    arm_matrix_instance_f32 S_inv;
    arm_mat_init_f32(&S_inv, 1, 1, S_inv_data);
    arm_mat_inverse_f32(&temp2, &S_inv);  // S_inv = inv(S)

    arm_mat_mult_f32(&kalman->P_pred, &H_t, &temp1);  // temp1 = P_pred * H^T
    arm_mat_mult_f32(&temp1, &S_inv, &kalman->K);     // K = P_pred * H^T * inv(S)

    // Step 2: Update state X = X_pred + K * (Z - H * X_pred)
    arm_mat_mult_f32(&kalman->H, &kalman->X_pred, &temp2);  // temp2 = H * X_pred
    arm_mat_sub_f32(&kalman->Z, &temp2, &temp2);           // temp2 = Z - H * X_pred
    arm_mat_mult_f32(&kalman->K, &temp2, &temp1);          // temp1 = K * (Z - H * X_pred)
    arm_mat_add_f32(&kalman->X_pred, &temp1, &kalman->X);   // X = X_pred + K*(Z - H*X_pred)

    // Step 3: Update covariance P = (I - K * H) * P_pred
    arm_mat_mult_f32(&kalman->K, &kalman->H, &temp1);  // temp1 = K * H
    arm_mat_sub_f32(&kalman->I, &temp1, &temp1);       // temp1 = I - K * H
    arm_mat_mult_f32(&temp1, &kalman->P_pred, &kalman->P);  // P = (I - K*H) * P_pred
}
