/*
 * Robot.cpp
 *  The implementation on controlling different parts of the robot;
 *  contain reference to all electronic parts except for the gripper, the rack, and the gyro
 *
 *  Created on: Feb 24, 2018
 *      Author: Lingrui
 */

#include "Robot.h"
#include "Arduino.h"
#include "LookUpTable.h"

static int rmotorpinF = 5; //forward pin // in1
static int rmotorpinB = 4; //backward pin //in2

static int lmotorpinF = 7; //forward pin //in1
static int lmotorpinB = 6; //backward pin //in2

static int rencoder1 = 2; //an interrupt pin //yellow
static int rencoder2 = 3; //an interrupt pin //white

static int lencoder1 = 18; //an interrupt pin //yellow
static int lencoder2 = 19; //an interrupt pin //white

static int usf_in = 25, usf_out = 26; //digital pins
static int usr_in = 23, usr_out = 24; //digital pins

//static int fan_pin = 4;

static int front3_pin = A3, front5_pin = A4; //analog pins
static int right7_pin = A1, right8_pin = A2; //analog pins

static int stepper_step_pin = 28;
static int stepper_dir_pin = 27;

static double encFactor = 0.0016198837120072371385823004945; //  2.75in * PI / 3200 tick/rev * 3 / 5

static double Sin(double deg){
	int degree = deg;
	double difference = deg - (double) degree;
	if (difference > 0.5) {
		degree++;
	}
	while (degree > 360) {
		degree -= 360;
	}

	while (degree < 0) {
		degree += 360;
	}

	if (degree >= 0 && degree <= 90) {
		return sin_t[degree];
	} else if (degree > 90 && degree <= 180) {
		return sin_t[180 - degree];
	} else if (degree > 180 && degree <= 270) {
		return -sin_t[180 - degree];
	} else {
		return -sin_t[degree];
	}
}

static double Cos(double deg){
	int degree = deg;
	double difference = deg - (double) degree;
	if (difference > 0.5) {
		degree++;
	}
	while (degree > 360) {
		degree -= 360;
	}

	while (degree < 0) {
		degree += 360;
	}

	if (degree >= 0 && degree <= 90) {
		return cos_t[degree];
	} else if (degree > 90 && degree <= 180) {
		return -cos_t[180 - degree];
	} else if (degree > 180 && degree <= 270) {
		return -cos_t[degree - 180];
	} else {
		return cos_t[360-degree];
	}
}


Robot::Robot() :
		motorright(rmotorpinF, rmotorpinB), motorleft(lmotorpinF, lmotorpinB),
		rightEnc(rencoder1, rencoder2), leftEnc(lencoder1, lencoder2),
		imuPID(17, 0.05, 15),
		ideal_heading(0),
		usFront(usf_in, usf_out), usRight(usr_in,usr_out),
		y(0), x(0),
		lastLeftEnc(0), lastRightEnc(0),
		stepper(stepper_step_pin, stepper_dir_pin){

}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}

/**
 * values [-255, 255]
 * positive means forward, and negative means backward
 */
void Robot::drive(int leftspeed, int rightspeed) {
	this->motorleft.drive(-leftspeed);
	this->motorright.drive(rightspeed);
}

bool Robot::turn(int degree, bool CCW, int maxspeed, double currentHeading) {
	int target_heading = ideal_heading + (degree * (CCW ? 1 : -1));
	int speed = imuPID.calc(target_heading, currentHeading, maxspeed);
//	Serial.print(", target, ");
//	Serial.print(target_heading);
//	Serial.print(", ideal, ");
//	Serial.print(ideal_heading);
//	Serial.print(", speed, ");
//	Serial.println(speed);

	if (abs(speed) > 20) {
		drive(-speed, speed);
	} else {
		drive(0, 0);
		ideal_heading = target_heading;
		resetEnc();
		return true;
	}

	return false;
}

bool Robot::driveDist(int speed, int distance) {
	int s = (distance - readLeftEnc()) / distance * speed + 30; //30 is the base speed
	if (distance > readLeftEnc()) {
		drive(s, s);
		return false;
	}

	drive(0, 0);
	return true;
}

RightAlign Robot::rightAlign(int maxspeed) {
	if (isFront()) {
		drive(0, 0);
		return TURNLEFT;
	} else if (usRight.distanceRead() > 70) {
		return TURNRIGHT;
	} else {
//		int s = (usFront.distanceRead() - 15) / 100 * maxspeed + 30;
		int s = maxspeed;
		//if deviated away from the wall, tilt right until within range again

		drive(s, s);
		return ALRIGHT;
	}
//	else if (usRight.distanceRead() < 5){
//
//	}

//	else if (isRight()){//need to change
////		int s = (usFront.distanceRead() - 15) / 100 * maxspeed + 30;
//		int s = maxspeed;
//		//if deviated away from the wall, tilt right until within range again
//
//		drive(s, s);
//		return ALRIGHT;
//	}else{
//		drive(0, 0);
//		return TURNRIGHT;
//	}
}

bool Robot::isFront() {
	return readUsFront() < 2.0 && isFrontLine(); //also need line follower readings
}

/**
 * need to change
 */
bool Robot::isFrontLine() {
	return analogRead(front3_pin)  > 200  && analogRead(front5_pin)  > 200; //also need line follower readings
}

bool Robot::isRightLine(){
	return analogRead(right7_pin) > 200 && analogRead(right8_pin) > 200;
}

void Robot::setStepperAngle(int deg) {
	stepper.turnTo((double) deg);
}

// return in inches
double Robot::readUsFront(){
	return ((double)usFront.distanceRead() - 2.50) / 2.54;
}

// return in inches
double Robot::readUsRight(){
	return ((double)usRight.distanceRead() - 2.50) / 2.54;
}

void Robot::fan(bool on){
//	int control = on? HIGH : LOW;
//	digitalWrite(fan_pin, control);
}

//==================================================
//===                ENCODER                     ===
//==================================================

/**
 * resets the encoder count
 */
void Robot::resetEnc() {
	rightEnc.write(0);
	leftEnc.write(0);
	lastLeftEnc = 0;
	lastRightEnc = 0;
	delay(100);
}

double Robot::readLeftEnc() {
	return - (double) leftEnc.read() * encFactor;
}

double Robot::readRightEnc() {
	return (double) rightEnc.read() * encFactor;
}

void Robot::updateCoor(double heading){
	double currLeftEnc = readLeftEnc();
	double currRightEnc = readRightEnc();
	double delta = ((currLeftEnc - lastLeftEnc) + (currRightEnc - lastRightEnc)) / 2;
	x += delta * Sin(heading);
	y += delta * Cos(heading);
	lastLeftEnc = readLeftEnc();
	lastRightEnc = readRightEnc();
}
