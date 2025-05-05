/*
 * motor.h
 *
 *  Created on: May 2, 2025
 *      Author: jirat
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "math.h"
#include "main.h"

typedef struct
{
	TIM_HandleTypeDef* htimx;
	GPIO_TypeDef* gpiox;
	uint16_t motor_pin;
	uint16_t tim_chx;
	uint32_t period_cyc;
	uint16_t prescaler;
	uint16_t overflow;
	uint32_t cpu_freq;
	uint32_t OC;

}MOTOR;

void MotorInit(MOTOR* pwm, TIM_HandleTypeDef* htimx, uint16_t tim_chx, GPIO_TypeDef* gpiox, uint16_t motor_pin);
void MotorSet(MOTOR* MOTOR, float freq, float val, GPIO_PinState dir);


#endif /* INC_MOTOR_H_ */
