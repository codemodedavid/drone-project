/*
 * pid_control.h
 *
 *  Created on: Jul 28, 2025
 *      Author: davv
 */

#ifndef INC_PID_CONTROL_H_
#define INC_PID_CONTROL_H_

typedef struct {
    float error;
    float integral;
    float derivative;
    float last_error;
    float output;
} PIDController;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PIDCoefficients;

void update_pid(PIDController* pid, PIDCoefficients* coeff, float desired, float measured, float dt);

#endif /* INC_PID_CONTROL_H_ */
