/*
 * pid_control.c
 *
 *  Created on: Jul 28, 2025
 *      Author: davv
 */

#include <pid_control.h>

void update_pid(PIDController* pid, PIDCoefficients* coeff, float desired, float measured, float dt) {
    pid->error = desired - measured;
    pid->integral += pid->error * dt;
    pid->derivative = (pid->error - pid->last_error) / dt;

    pid->output = coeff->Kp * pid->error +
                  coeff->Ki * pid->integral +
                  coeff->Kd * pid->derivative;

    pid->last_error = pid->error;
}

