/*
 * StepperMotor.cpp
 *
 *  Created on: Apr 18, 2018
 *      Author: Lingrui
 */

#include "StepperMotor.h"
#include <Arduino.h>

static float stepangle = 1.8;
static int frequency = 20;
static double delays = 1000 / frequency / 2;

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
	int steps = (deg - heading) / stepangle;
	if (steps < 0) {
		digitalWrite(DIR_pin, LOW); //HIGH means ccw
		steps = -steps;
	} else {
		digitalWrite(DIR_pin, HIGH);
	}
	for (int x = 0; x < steps; x++) {
		digitalWrite(PWM_pin, HIGH);
		delay(delays);
		digitalWrite(PWM_pin, LOW);
		delay(delays);
		Serial.println("turn");
	}
	heading = deg;
}
