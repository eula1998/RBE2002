/*
 * PololuMotor.h
 *	An interface intended for controlling the Pololu motors with a
 *		pololu motor controller with forward and reverse current pin
 *
 *  Created on: Feb 24, 2018
 *      Author: Lingrui
 */

#ifndef POLOLUMOTOR_H_
#define POLOLUMOTOR_H_

class PololuMotor {
public:
	PololuMotor(int forward, int backward);
	virtual ~PololuMotor();
	/**
	 * [-255, 255] backward -> forward
	 */
	void drive(int speed);
private:
	int forwardPin;
	int backwardPin;
};

#endif /* POLOLUMOTOR_H_ */
