/*
 * QEI.c
 *
 *  Created on: May 6, 2025
 *      Author: tien
 */

#include "QEI.h"


// Constructor
void QEIInit(QEI *qei, TIM_HandleTypeDef *htim_qei, int32_t ppr, float frequency, int32_t counter_period)
{

	qei->htim_qei = htim_qei;
	qei->ppr = ppr;
	qei->frequency = frequency;
	qei->cp = counter_period;

	qei->new_val = 0;
	qei->old_val = 0;
	qei->pulses = 0;
	qei->revs = 0;
	qei->rads = 0;
	qei->radps = 0;

	HAL_TIM_Encoder_Start(htim_qei,TIM_CHANNEL_ALL);

}

// Function
void QEIPosVelUpdate(QEI *qei)
{

	qei -> new_val = __HAL_TIM_GET_COUNTER(qei -> htim_qei);

	// Wrap around
	qei->diff_count = (qei -> new_val) -  (qei -> old_val);

	if (qei->diff_count > (qei->cp)/2){ qei->diff_count -= qei->cp;}
	else if (qei->diff_count < -(qei->cp/2)) { qei->diff_count += qei->cp;}


	// Pulse Position
	qei -> pulses += qei->diff_count;


	// Revolution round
 	qei -> revs = ((float)qei -> pulses / qei -> ppr);

 	// Radian
 	qei -> rads = qei->revs * 2 * M_PI;


 	// Angular velocity calculation
 	qei -> radps = ((qei->diff_count * qei->frequency)*2*M_PI/qei->ppr);

 	// Update value
 	qei -> old_val = qei -> new_val;

}
