#include "L3G.h"
#include "Wire.h"
#include "Arduino.h"
#include "Encoder.h"
#include "Robot.h"

#define FRAME (25) // gyro heading refresh rate

Robot robot; 

//==================================================
//===                   GYRO                     === 
//==================================================          

L3G gyro;

//stores the current accumulated heading read by the current frame
double heading;
int zero;

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
	//positive = CW
	if (((int) gyro.g.y > 0 ? (int) gyro.g.y : -gyro.g.y) > 500) {
		heading += ((double) gyro.g.y - zero) * FRAME * 0.00000875; //8.75 / 1000 / 1000;
//    lastread = millis();
	}
	return heading;
}


//==================================================
//===                   MAIN                     ===
//==================================================

void setup() {
	Serial.begin(115200);
	//GYRO
//	Wire.begin();
//	//keeps polling the gyro status
//	//side effect: blocks the program till the power is turned on
//	while (!gyro.init()) {
//		Serial.println("Failed to autodetect gyro type!");
//		delay(200);
//	}
//	gyro.enableDefault();
//
//	//GYRO calibration
//	double calibration = 0;
//	for (int x = 0; x < 100; x++) {
//		gyro.read();
//		calibration += gyro.g.y;
//	}
//	zero = calibration / 100.0;

}

//states for recording the frame rate
int ltime = 0;
int curtime = 0;

void loop() {
	// put your main code here, to run repeatedly:
  robot.drive(255, 255);


//do what needs to be done in a frame
  //set frame rate for gyro and everything else
  while (millis() - ltime < FRAME) {
  }
  //the gyro can not be used simultaneously with bluetooth
  if (usegyro) {
    Serial.print("Heading: ");
    Serial.println(getHeading());
  }
  ltime = millis();
}
