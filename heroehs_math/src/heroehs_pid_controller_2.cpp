/*
 * pid_controller.cpp
 *
 *  Created on: Jan 25, 2019
 *      Author: ijm
 */

#include <stdio.h>
#include "heroehs_math/heroehs_pid_controller_2.h"

using namespace heroehs_math;


PIDController::PIDController(double control_time)
{
	kp = 0;
	ki = 0;
	kd = 0;

	control_time_= control_time;
	previous_error_ = 0;
	current_error_ = 0;
	integrator_ = 0;


}
PIDController::~PIDController()
{

}

void PIDController::PID_set_gains(double Kp, double Ki, double Kd)
{
	kp=Kp;
	ki=Ki;
	kd=Kd;
}

void PIDController::PID_reset_integral()
{
	integrator_=0;

}

double PIDController::PID_process(double desired, double present)
{
	float output = 0;
	current_error_ = desired-present;
	integrator_ += current_error_;

	output = kp*current_error_;
	output += ki*integrator_*control_time_;
	output += kd*(current_error_-previous_error_)/control_time_;

	previous_error_ = current_error_;

	return output;


}


