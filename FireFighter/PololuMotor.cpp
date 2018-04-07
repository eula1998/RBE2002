/*
 * PololuMotor.cpp
 *
 *  Created on: Feb 24, 2018
 *      Author: Lingrui
 */

#include "PololuMotor.h"
#include "Arduino.h"

PololuMotor::PololuMotor(int forward, int backward)
:forwardPin(forward), backwardPin(backward)
{
}

PololuMotor::~PololuMotor() {
	// TODO Auto-generated destructor stub
}

/**
 * drives the motor with the given speed;
 * [-255, 255], with negative being backward
 * 0 is stop
 */
void PololuMotor::drive(int speed){
	if (speed > 255)	speed = 255;
	if (speed < -255) 	speed = -255;

	if (speed >= 0){
		analogWrite(forwardPin, speed);
		analogWrite(backwardPin, 0);
	}else{
		analogWrite(forwardPin, 0);
		analogWrite(backwardPin, -speed);
	}
}
