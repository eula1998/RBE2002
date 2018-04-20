/*
 * PID.cpp
 *
 *  Created on: April 14, 2018
 *      Author: Lingrui
 */

#include "PID.h"
#include "Arduino.h"

PID::PID(double p, double i, double d) :
		kp(p), ki(i), kd(d) {
	ki_sum = 0;
	lasterror = 0;
	index = 0;
}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

static double sum(double a[], int s) {
//a = array, s = size
	double total = 0.0;
	for (int x = 0; x < s; x++) {
		total += a[x];
	}
	return total;
}

//Write this function to calculate a control signal from the set velocity 
//and the current velocity 
double PID::calc(double setVel, double curVel, int max) {
	// calculate error
	double error = setVel - curVel;

	// calculate derivative of error
	double derivative = error - lasterror;
	lasterror = error;

	// calculate integral error. Running average is best but hard to implement
	pasterror[index] = error;
	index = (index + 1) % ARRAY_SIZE;
	double integral = sum(pasterror, ARRAY_SIZE);

//    ki_sum += error; 
//    double integral = ki_sum;

// sum up the error value to send to the motor based off gain values.
	float control_signal = (kp * error + ki * integral + kd * derivative);

//    float control_signal = (output_velocity / 1200.0) * 255.0; //full speed at 12V

// limit control value to 0-254
	if (control_signal > max) {
		control_signal = max;
	} else if (control_signal < -max) {
		control_signal = -max;
	}

//    Serial.print("p value:");
//    Serial.println(kp*error);
//    Serial.print("i value:");
//    Serial.println(ki*integral);
//    Serial.print("d value:");
//    Serial.println(kd*derivative);

//    Serial.print("control (c):");
//    Serial.println(control_signal);

	//return the control signal
	return control_signal;
//    return 255;
}
