#include <QTRSensors.h>//line tracker

#include "L3G.h"
#include "Wire.h"
#include "Arduino.h"
#include "Robot.h"

#define FRAME (40) // gyro heading refresh rate, in millis

/**
 * encoder left: 2, 23
 * encoder right: 3, 24
 * front: in: 27 out: 26
 * right: in: 29 out: 28
 * stepper pwn: 25
 * button: 22
 */
Robot robot;

//==================================================
//===                   GYRO                     ===
//==================================================

L3G gyro;

//stores the current accumulated heading read by the current frame
double heading;
double zero;

// turn off gyro reading when not needed to reduce interference with other sensor that interrupts
bool usegyro;

/**
 * reset the accumulated heading to zero and get ready for the next read
 */
void zeroHeading() {
	heading = 0.0;
	usegyro = true;
	Serial.println("Resetting");
}

/**
 * should only be called once in the loop() function for every frame
 */
double getHeading() {
	gyro.read();

	//integrates the gyro reading using the time interval since last read
	//assuming the robot is turning at a constant angular velocity since the last reading
	//unit in millis degrees (mdegree/s * ms * 1s/1000ms)
	//positive = CCW
	if (((int) gyro.g.x > 0 ? (int) gyro.g.x : -gyro.g.x) > 500) {
		heading += ((double) gyro.g.x - zero) * FRAME * 0.00000875; //8.75 / 1000 / 1000;
	}
	return heading;
}

//==================================================
//===                   MAIN                     ===
//==================================================

void setup() {
	Serial.begin(115200); //stay there for the gyro's sake
	//GYRO
	Wire.begin();
	//keeps polling the gyro status
	//side effect: blocks the program till the power is turned on
	while (!gyro.init()) {
		Serial.println("Failed to autodetect gyro type!");
		delay(200);
	}
	gyro.enableDefault();
	delay(1000);

	//GYRO calibration
	double calibration = 0;
	for (int x = 0; x < 100; x++) {
		gyro.read();
		calibration += gyro.g.x;
		delay(25);
	}
	zero = calibration / 100.0;

//  resetEnc();
//  usegyro = true;
	zeroHeading();

}

//states for recording the frame rate
int ltime = 0;	//last time
int curtime = 0;

void loop() {
	// put your main code here, to run repeatedly:
	robot.turn(180, true, 200, heading);
//  Serial.print("left ");
//  Serial.println(robot.readLeft());
//  Serial.print("right");
//  Serial.println(robot.readRight());
//  robot.drive(90, 90);
//CANNOT READ ENCODER AND USE GYRO AT THE SAME TIME

//do what needs to be done in a frame
	//set frame rate for gyro and everything else
	while (millis() - ltime < FRAME) {
	}

	ltime = millis();
	//the gyro can not be used simultaneously with bluetooth
	if (usegyro) {
		Serial.print("Heading: ");
		Serial.println(getHeading());
	}

//  Serial.print(readLeft());
//  Serial.print(" ");
//  Serial.println(readRight());
}
