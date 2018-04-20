/*
 * StepperMotor.cpp
 *
 *  Created on: Apr 18, 2018
 *      Author: Lingrui
 */

#include "StepperMotor.h"
#include <Arduino.h>

static float stepangle = 1.8;
static int frequency = 25;
static double delays = 500 / frequency;

StepperMotor::StepperMotor(int outputpin, int dirpin) {
	// TODO Auto-generated constructor stub
	heading = 0;
	PWM_pin = outputpin; //25
	DIR_pin = dirpin; //23
	pinMode(PWM_pin, OUTPUT);
	pinMode(DIR_pin, OUTPUT);
}

StepperMotor::~StepperMotor() {
	// TODO Auto-generated destructor stub
}

/**
 * Turn the stepper motor to a set direction, with input in the range of [-90, 90]
 *[left 90 deg, right 90 deg], with forward facing be 0 deg
 */
void StepperMotor::turnTo(double deg) {
	int steps = abs(deg - heading) / stepangle;
	steps++;
	if (deg < heading) {
		digitalWrite(DIR_pin, LOW); //LOW means ccw
	} else {
		digitalWrite(DIR_pin, HIGH);
	}
	for (int x = 0; x < steps; x++) {
		digitalWrite(PWM_pin, HIGH);
		delay(delays);
		digitalWrite(PWM_pin, LOW);
		delay(delays);
	}

//	Serial.print("Initial heading, ");
//	Serial.print(heading);
	if (deg < heading) {
		heading -= (steps * stepangle);
	} else {
		heading += (steps * stepangle);
	}
//	Serial.print(", final heading, ");
//	Serial.print(heading);
//	Serial.print(", Steps, ");
//	Serial.print(steps);
//	Serial.print(", Degree, ");
//	Serial.println(deg);
}

double StepperMotor::getHeading(){
	return heading;
}
