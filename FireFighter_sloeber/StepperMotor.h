/*
 * StepperMotor.h
 *
 *  Created on: Apr 18, 2018
 *      Author: Lingrui
 */

#ifndef STEPPERMOTOR_H_
#define STEPPERMOTOR_H_

class StepperMotor {
public:
	StepperMotor(int outputpin, int dirpin);
	virtual ~StepperMotor();
	void turnTo(double deg);
	double getHeading();
private:
	double heading;//front = 0 deg; right = 90 deg; left = -90 deg
	int PWM_pin;
	int DIR_pin;
};

#endif /* STEPPERMOTOR_H_ */
