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

static const int rmotorpinF = 5; //forward pin // in1
static const int rmotorpinB = 4; //backward pin //in2

static const int lmotorpinF = 7; //forward pin //in1
static const int lmotorpinB = 6; //backward pin //in2

static const int rencoder1 = 2; //an interrupt pin //yellow
static const int rencoder2 = 3; //an interrupt pin //white

static const int lencoder1 = 18; //an interrupt pin //yellow
static const int lencoder2 = 19; //an interrupt pin //white

static const int usf_in = 25, usf_out = 26; //digital pins
static const int usr_in = 23, usr_out = 24; //digital pins

static const int fan_pin = 12;

static const int front3_pin = A3, front5_pin = A4; //analog pins
static const int right7_pin = A1, right8_pin = A2; //analog pins

static const int stepper_step_pin = 28;
static const int stepper_dir_pin = 27;

static const int button_pin = 22;;

static double encFactor = 0.0016198837120072371385823004945; //  2.75in * PI / 3200 tick/rev * 3 / 5


static const double turningCircumference = 23.561944901923449288469825374596;
static const double turningFactor = 0.04244131815783875620503567023267;

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


Robot::Robot():
		motorright(rmotorpinF, rmotorpinB), motorleft(lmotorpinF, lmotorpinB),
		rightEnc(rencoder1, rencoder2), leftEnc(lencoder1, lencoder2),
		imuPID(17, 0.05, 15),
		ideal_heading(0),
		usFront(usf_in, usf_out), usRight(usr_in,usr_out),
		x(0.0), y(0.0),
		lastLeftEnc(0), lastRightEnc(0),
		stepper(stepper_step_pin, stepper_dir_pin){
	pinMode(fan_pin, OUTPUT);
	digitalWrite(fan_pin, LOW);
	pinMode(button_pin, INPUT_PULLUP);
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
//	if (firstturn){
//		degree *= 2;
//	}
	int speed = imuPID.calc(target_heading, currentHeading, maxspeed);

//	int speed = imuPID.calc(degree * CCW? 1 : -1, currentHeading, maxspeed);

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
//		if (!firstturn){
//			firstturn = true;
//		}
		return true;
	}

	return false;
}

//CCW = positive
bool Robot::odometryTurn(int degree, bool CCW, int maxspeed){
	double odometryHeading = (abs(readLeftEnc()) + (abs(readRightEnc()))) * turningFactor * 180 * (CCW? 1 : -1); //(360 / 2)
	Serial.print("Odometry Heading: ");
	Serial.println(odometryHeading);
	int speed = imuPID.calc(degree * (CCW? 1 : -1) , odometryHeading, maxspeed);
	if (abs(speed) > 20) {
		drive(-speed, speed);
	} else {
		drive(0, 0);
		resetEnc();
		ideal_heading += odometryHeading;
		return true;
	}
	return false;
}

bool Robot::driveDist(int speed, bool forward, int distance) {
	if (forward){
		if (isFrontLine() || isFrontUS()){
			drive(0, 0);
			return true;
		}
	}

	int s = (distance - abs(readLeftEnc())) / distance * speed + 30; //30 is the base speed
	s *= forward? 1 : -1;
	//make sure it drives straight
//	Serial.print("Dist: ");
//	Serial.println(readLeftEnc());
	if (distance > abs(readLeftEnc())) {
		drive(s, s);
		return false;
	}

	drive(0, 0);
	return true;
}

bool Robot::driveForward(int speed){
	if (isFrontLine()){
		drive(0, 0);
		return true;
	}
	delay(20);
	if (readUsFront() < 3){
		drive(0, 0);
		return true;
	}
	if (readUsRight() < 10){
		drive(0, 0);
		return true;
	}
	drive(speed, speed);
	return false;
}

RightAlign Robot::rightAlign(int maxspeed) {
	if (isFrontLine()) {
		drive(0, 0);
		return ONCLIFF;
	} else if(isFrontUS()){
		drive(0, 0);
		return TURNLEFT;
	}else if (readUsRight() > 10) {//field is ~ 80 inch
		delay(750);
		drive(0, 0);
		Serial.println("NO MORE WALL");
		return TURNRIGHT;
	} else {
//		int s = (usFront.distanceRead() - 15) / 100 * maxspeed + 30;
		int s = maxspeed;
		//if deviated away from the wall, tilt right until within range again
		if (usRight.distanceRead() > 7){//distance is greater than 20 cm
			drive(s*1.05, s * 0.95);
		}else if(usRight.distanceRead() < 4){
			drive(s * 0.95, s * 1.05);
		}else{
			drive(s, s);
		}
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

//==================================================
//===            MISCELLANEOUS CHECKS            ===
//==================================================

bool Robot::isFrontUS() {
	int reading = readUsFront();
//	Serial.print("us front (in): ");
//	Serial.println(reading);
	return reading < 3.0 && reading != 0;
}

bool Robot::isRightUS(){
	int reading = readUsRight();
//	Serial.print("us right (in): ");
//	Serial.println(reading);
	return readUsRight() < 5.0 && reading != 0;
}

/**
 * white board = ~110
 * black = 1023
 */
bool Robot::isFrontLine() {
	int reading3 = analogRead(front3_pin);
//	Serial.print("front 3: ");
//	Serial.println(reading3);
	int reading5 = analogRead(front5_pin);
//	Serial.print("front 5: ");
//	Serial.println(reading5);
	return reading3  > 200  && reading5  > 200; //also need line follower readings
}

bool Robot::isRightLine(){
	return analogRead(right7_pin) > 200 && analogRead(right8_pin) > 200;
}

void Robot::setStepperAngle(int deg) {
	stepper.turnTo((double) deg);
}

// return in inches
double Robot::readUsFront(){
	Serial.print("us front (in): ");
	double reading = usFront.distanceRead();
	reading = (reading - 2.50) / 2.54;
	Serial.println(reading);
	return reading;
}

// return in inches
double Robot::readUsRight(){
	Serial.print("us right (in): ");
	double reading = usRight.distanceRead();
	reading = (reading - 2.50) / 2.54;
	Serial.println(reading);
	return reading;
}

void Robot::fan(bool on){
	int control = on? HIGH : LOW;
	digitalWrite(fan_pin, control);
	Serial.print("fan");
	Serial.println(control);
}

void Robot::stop(){
	drive(0, 0);
}

bool Robot::buttonPressed(){
	return !digitalRead(button_pin);//LOW = pressed
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

void Robot::updateCoor(){
	double currLeftEnc = readLeftEnc();
	double currRightEnc = readRightEnc();
	double delta = ((currLeftEnc - lastLeftEnc) + (currRightEnc - lastRightEnc)) / 2;
	x += delta * Sin(ideal_heading);
	y += delta * Cos(ideal_heading);
	lastLeftEnc = readLeftEnc();
	lastRightEnc = readRightEnc();
}


double Robot::getX(){
	return x;
}

double Robot::getY(){
	return y;
}
