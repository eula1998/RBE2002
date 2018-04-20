/*
 * Robot.h
 *  An interface intended for controlling the Pololu motors with a
 *    pololu motor controller with forward and reverse current pin
 *
 *  Created on: Feb 24, 2018
 *      Author: Lingrui
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "PololuMotor.h"
#include "PID.h"
#include <Encoder.h>
#include <Ultrasonic.h>
#include "StepperMotor.h"

typedef enum {
	TURNLEFT, TURNRIGHT, ALRIGHT
} RightAlign;
class Robot {
public:
	Robot();
	virtual ~Robot();

	void drive(int leftspeed, int rightspeed);

	bool turn(int degree, bool CCW, int maxspeed, double currHeading);

	double readLeft(); //in inches
	double readRight(); //in inches
	bool driveDist(int speed, int distance); //distance in inch //not tested yet
	RightAlign rightAlign(int maxspeed);
	void resetEnc();
	void updateCoor(double heading); //given a global heading, calculate the x and y displacement
	void setStepperAngle(int deg);
private:
	PololuMotor motorright;
	PololuMotor motorleft;
	PID imuPID;
	Encoder rightEnc;
	Encoder leftEnc;
	double encFactor = 0.0016198837120072371385823004945; //  2.75in * PI / 3200 tick/rev * 3 / 5
	Ultrasonic usFront;
	Ultrasonic usRight;
	bool isRight();

	bool isFront();

	double x;
	double y;
	double ideal_heading;
	StepperMotor stepper;
};

#endif /* POLOLUMOTOR_H_ */
