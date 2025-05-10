/*
 * QEI.h
 *
 *  Created on: May 6, 2025
 *      Author: tien
 */

#ifndef INC_QEI_H_
#define INC_QEI_H_

#include "main.h"
#include "math.h"

typedef struct
{
	// Timer
	TIM_HandleTypeDef* htim_qei;

	// Parameter
	int32_t ppr;
	float frequency;
	int32_t cp;

	// Position
	int32_t pulses;
	float revs;
	float rads;
	float meter;
	int32_t diff_count;

	// Velocity
	float radps;

	int32_t new_val;
	int32_t old_val;

}QEI;

// Constructor

void QEIInit(QEI *qei, TIM_HandleTypeDef *htim_qei, int32_t ppr, float frequency, int32_t counter_period);

void QEIPosVelUpdate(QEI *qei);

#endif /* INC_QEI_H_ */
