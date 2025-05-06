/*
 * PID.c
 *
 *  Created on: May 6, 2025
 *      Author: tien
 */


#include "PID.h"

void PIDInit(CONTROLLER* controller, float u_max, float u_min)
{
	controller -> u_max = u_max;
	controller -> u_min = u_min;
}

float PIDCompute(CONTROLLER* controller , float kp, float ki, float kd, float error)
{

	controller -> kp = kp;
	controller -> kd = kd;
	controller -> ki = ki;

    // Anti-windup: only integrate if not saturated
    if (!((controller->u >= controller->u_max && error > 0) ||
          (controller->u <= controller->u_min && error < 0)))
    {
        float delta_u = (controller->kp + controller->ki + controller->kd) * error
                      - (controller->kp + 2 * controller->kd) * controller->prev_error_one
                      + (controller->kd * controller->prev_error_two);

        controller->u += delta_u;
    }

    if (controller->u > controller->u_max) {controller->u = controller->u_max;}
    else if (controller->u < controller->u_min) {controller->u = controller->u_min;}

    // Shift error history
    controller->prev_error_two = controller->prev_error_one;
    controller->prev_error_one = error;

    return controller->u;
}
