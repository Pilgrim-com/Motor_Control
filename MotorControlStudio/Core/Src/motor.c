#include "motor.h"

void MotorInit(MOTOR* MOTOR, TIM_HandleTypeDef* htimx, uint16_t tim_chx, GPIO_TypeDef* gpiox, uint16_t motor_pin)
{
	MOTOR->htimx = htimx;
	MOTOR->cpu_freq = 170e6;
	MOTOR->tim_chx = tim_chx;
	MOTOR->OC = 0;
	MOTOR->gpiox = gpiox;
	MOTOR->motor_pin = motor_pin;

	HAL_TIM_Base_Start(htimx);
	HAL_TIM_PWM_Start(htimx, tim_chx);
}

void MotorSet(MOTOR* MOTOR, float freq, float val)
{
	if (freq == 0)
	{
		__HAL_TIM_SET_COMPARE(MOTOR->htimx, MOTOR->tim_chx, 0);  // Fixed
	}
	else
	{

		// Calculate Prescaler
		MOTOR->period_cyc = (uint32_t) (MOTOR->cpu_freq / freq);
		MOTOR->prescaler = (uint16_t) ((MOTOR->period_cyc + 65535 - 1)/65535.00) - 1;
		MOTOR->overflow = (uint16_t) ((MOTOR->cpu_freq/ (float)(MOTOR->prescaler+1) / freq) - 1);
		MOTOR->OC = (uint16_t) (MOTOR->overflow * fabs(val) / 65535.00);

		// MOTOR DIR
		if (val >= 0) {HAL_GPIO_WritePin(MOTOR->gpiox, MOTOR->motor_pin, GPIO_PIN_RESET);}
		else {HAL_GPIO_WritePin(MOTOR->gpiox, MOTOR->motor_pin, GPIO_PIN_SET);}

		// Apply value to TIM
		__HAL_TIM_SET_PRESCALER(MOTOR->htimx, MOTOR->prescaler);
		__HAL_TIM_SET_AUTORELOAD(MOTOR->htimx, MOTOR->overflow);
		__HAL_TIM_SET_COMPARE(MOTOR->htimx, MOTOR->tim_chx, MOTOR->OC);
	}
}
