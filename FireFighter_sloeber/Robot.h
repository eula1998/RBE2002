/*
 * Robot.h
 *  An interface intended for controlling the Pololu motors with a
 *    pololu motor controller with forward and reverse current pin
 *
 *  Created on: Mar 24, 2018
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
	TURNLEFT, TURNRIGHT, ALRIGHT, ONCLIFF
} RightAlign;

//for signaling between the state machine and right-align() function
typedef enum {
	LEFT, FORWARD, CONTINUE
} FlameStatus;

class Robot {
public:
	Robot();
	virtual ~Robot();


	void drive(int leftspeed, int rightspeed);

	void stop();

	bool turn(int degree, bool CCW, int maxspeed, double currHeading);

	bool odometryTurn(int degree, bool CCW, int maxspeed);

	bool driveDist(int speed, bool forward, int distance); //distance in inch //not tested yet
	bool driveForward(int speed);

	RightAlign rightAlign(int maxspeed);
	void resetEnc();
	void updateCoor(); //given a global heading, calculate the x and y displacement
	void setStepperAngle(int deg); //blocking

	void servoDeg(int input);

	bool isFrontUS();
	bool isRightUS();
	bool isFrontLine();

	bool isRightLine();

	double readUsFront();

	double readUsRight();

	void fan(bool on);

	double readLeftEnc(); //in encoder in inches
	double readRightEnc(); //in encoder in inches

	double getX();
	double getY();

	bool buttonPressed();

	double ideal_heading;
private:
	PololuMotor motorright;
	PololuMotor motorleft;

	Encoder rightEnc;
	Encoder leftEnc;


	PID imuPID;

	Ultrasonic usFront;
	Ultrasonic usRight;

	double x;
	double y;
	double lastLeftEnc;
	double lastRightEnc;

	StepperMotor stepper;

};

#endif /* POLOLUMOTOR_H_ */
