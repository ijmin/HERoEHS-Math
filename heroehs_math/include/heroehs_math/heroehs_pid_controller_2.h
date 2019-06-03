/*
 * pid_controller.h
 *
 *  Created on: Jan 25, 2019
 *      Author: ijm
 */

#ifndef HEROEHS_MATH_INCLUDE_HEROEHS_MATH_HEROEHS_PID_CONTROLLER_2_H_
#define HEROEHS_MATH_INCLUDE_HEROEHS_MATH_HEROEHS_PID_CONTROLLER_2_H_

namespace heroehs_math
{

class PIDController
{
public:
	PIDController(double control_time=0.008);
	~PIDController();

	double PID_process(double desired, double present);

	void PID_set_gains(double Kp, double Ki, double Kd);
	void PID_reset_integral();

	double kp, ki, kd;

private:

	double control_time_;
	double current_error_;
	double previous_error_;
	double integrator_;




};




}







#endif /* HEROEHS_MATH_INCLUDE_HEROEHS_MATH_HEROEHS_PID_CONTROLLER_2_H_ */
