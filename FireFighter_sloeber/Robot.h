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

typedef enum {
	LEFT, FORWARD, CONTINUE
} FlameStatus;

class Robot {
public:
	Robot();
	virtual ~Robot();

	void drive(int leftspeed, int rightspeed);

	bool turn(int degree, bool CCW, int maxspeed, double currHeading);

	bool driveDist(int speed, int distance); //distance in inch //not tested yet
	RightAlign rightAlign(int maxspeed);
	void resetEnc();
	void updateCoor(double heading); //given a global heading, calculate the x and y displacement
	void setStepperAngle(int deg); //blocking

	void servoDeg(int input);

	double readUsFront();

	double readUsRight();

	void fan(bool on);

	double readLeftEnc(); //in encoder in inches
	double readRightEnc(); //in encoder in inches
private:
	PololuMotor motorright;
	PololuMotor motorleft;

	Encoder rightEnc;
	Encoder leftEnc;

	PID imuPID;

	double ideal_heading;

	Ultrasonic usFront;
	Ultrasonic usRight;

	bool isFront();

	bool isFrontLine();

	bool isRightLine();

	double x;
	double y;
//	double readLeftEnc(); //in encoder in inches
//	double readRightEnc(); //in encoder in inches
	double lastLeftEnc;
	double lastRightEnc;

	StepperMotor stepper;

};

#endif /* POLOLUMOTOR_H_ */
