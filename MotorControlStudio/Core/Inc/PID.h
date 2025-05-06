/*
 * PID.h
 *
 *  Created on: May 6, 2025
 *      Author: tien
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"
#include "math.h"

typedef struct{

	float kp;
	float ki;
	float kd;

	float u;

	float u_max;
	float u_min;

	float prev_error_one;
	float prev_error_two;

}CONTROLLER;


void PIDInit(CONTROLLER* controller, float u_max, float u_min);
float PIDCompute(CONTROLLER* controller , float kp, float ki, float kd, float error);

#endif /* INC_PID_H_ */
