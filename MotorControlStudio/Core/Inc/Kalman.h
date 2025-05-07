/*
 * Kalman.h
 *
 *  Created on: May 7, 2025
 *      Author: spwkp
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "arm_math.h"

typedef struct {

	float32_t A_f32[16];
	arm_matrix_instance_f32 A;

	float32_t B_f32[4];
	arm_matrix_instance_f32 B;

	float32_t H_f32[4];
	arm_matrix_instance_f32 H;

	float32_t G_f32[4];
	arm_matrix_instance_f32 G;



} KALMAN;

void KalmanInit();

#endif /* INC_KALMAN_H_ */
