#include <QTRSensors.h>//line tracker

#include "L3G.h"
#include "Wire.h"
#include <Arduino.h>
#include "Robot.h"
#include <Servo.h>
#include <LiquidCrystal.h>
#include <Math.h>

#define FRAME (40) // gyro heading refresh rate, in millis
//#define FRAME (1000)

Robot robot;
const int rs = 40, en = 41, d4 = 42, d5 = 43, d6 = 44, d7 = 45;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//==================================================
//===                   GYRO                     ===
//==================================================

L3G gyro;

//stores the current accumulated heading read by the current frame
//double heading;
double zero;
double global_heading;

// turn off gyro reading when not needed to reduce interference with other sensor that interrupts
bool usegyro;

/**
 * reset the accumulated heading to zero and get ready for the next read
 */
void zeroHeading() {
//	heading = 0;
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
		global_heading += ((double) gyro.g.x - zero) * FRAME * 0.00000875; //8.75 / 1000 / 1000;
//		heading += ((double) gyro.g.x - zero) * FRAME * 0.00000875; //8.75 / 1000 / 1000;
	}
	return global_heading;
//	return heading;
}

//==================================================
//===           SERVO + FLAME DETECTION          ===
//==================================================

Servo servo;
//up = 180, down = 0
static int servopin = 8;
int flamedata[13];
const int flame_pin = A0;
const double delta_fan_flame_dist = 1.875; //inch
const double fan_height = 8.25;//in ground to fan axle
const double us_fix_dist = 2.0;
const double us_stepcenter_dist = 2.875; //inch *********PLS CHANGE**********
//TO BE CALCULATED - kinda like registers
int delta_fan_flame_angle;
double flame_height;

FlameStatus checkFlame(){
	servo.write(90);
	robot.fan(false);
	robot.setStepperAngle(-90);
	int angle;
	int reading;
	int max = 1024;
	int maxangle = 0;
	for (int x = 0; x < 13; x++){
		flamedata[x] = 1024;
		angle = 15 * x - 90;
		robot.setStepperAngle(angle);
		for (int y = 60; y <= 120; y+= 15){
			servo.write(y);
			reading = analogRead(flame_pin);
			if (reading  < flamedata[x]){
				flamedata[x] = reading;
			}
			delay(100);
		}
//		flamedata[x] = analogRead(flame_pin);
		Serial.print("flame, ");
		Serial.print(flamedata[x]);
		Serial.print(", angle, ");
		Serial.print(angle);

		if (flamedata[x] < max){
			maxangle = angle;
			max = flamedata[x];
			Serial.print(", max");
		}
		Serial.println();
	}
	Serial.print("maxangle, ");
	Serial.print(maxangle);
	Serial.print(", max, ");
	Serial.println(max);
	if (maxangle <= 0){
		if (max < 500){
//			return true;//maybe also return the 45 deg thingy
			if (maxangle < -45){
				return LEFT;
			}
			return FORWARD;
		}
	}
	return CONTINUE;
}

void getFlameHeight(int maxangle){//
//trig math taking into account the offset of the flame sensor away from the fan

	double candle_dist = us_fix_dist + us_stepcenter_dist;//in inches
	flame_height = sin((double)(maxangle - 90)/180*2*M_PI) * candle_dist + fan_height;
	int hypothenue = hypot(flame_height, candle_dist);//in inches
	delta_fan_flame_angle = atan(delta_fan_flame_dist/hypothenue);
	if (maxangle < 90){
		delta_fan_flame_angle *= -1;
	}
	Serial.print("Flame height, ");
	Serial.println(flame_height);
}

void blowOutFlame(){
	int max = 1024;
	int maxangle = 0;
	int flamedata[13];
	int angle;
	bool repeat;
	do {
		repeat = false;
		for (int x = 0; x < 13; x++) {
			angle = 60 + 5 * x;
			servo.write(angle);
			flamedata[x] = analogRead(flame_pin);
			Serial.print("flame, ");
			Serial.print(flamedata[x]);
			Serial.print(", angle, ");
			Serial.print(angle);
			if (max > flamedata[x]) {
				max = flamedata[x];
				maxangle = angle;
				Serial.print(", max");
			}
			Serial.println();
			delay(100);
		}
		Serial.print("maxangle, ");
		Serial.print(maxangle);
		Serial.print(", max, ");
		Serial.println(max);
		getFlameHeight(maxangle);
		servo.write(maxangle + delta_fan_flame_angle);
		robot.fan(true);
		delay(5000);
		robot.fan(false);
		servo.write(maxangle);
		if (analogRead(flame_pin) < 500){
			repeat = true;
		}
	}while(repeat);
//	lcd.setCursor(0, 1);
//	lcd.print("Flame Out!!!!!");
}

//==================================================
//===   		        MAIN 			         ===
//==================================================
void setup() {
	Serial.begin(115200); //stay there for the gyro's sake

	lcd.begin(16, 2);
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

	servo.attach(servopin);
	servo.write(90);

  usegyro = false;
//	zeroHeading();
	global_heading = 0;
}

//states for recording the frame rate
int ltime = 0;	//last time
int curtime = 0;

int temp = 0;

void loop() {
//CANNOT READ ENCODER AND USE GYRO AT THE SAME TIME
//NEED TO HOLD THE STEPPER MOTOR IN PLACE BEFORE TURNING ON THE ROBOT

//	switch (temp) {
//	case 0:
//		if (robot.turn(90, true, 175, global_heading)) {
//			temp = 1;
//			delay(5000);
//			zeroHeading();
//		}
//		break;
//	case 1:
//		if (robot.turn(75, false, 175, global_heading)){
//			temp = 2;
//			delay(5000);
////			usegyro = false;
//		}
//		break;
//	case 2:
//		if (robot.turn(105, false, 175, global_heading)){
//			temp = 3;
////			usegyro = false;
//		}
//		break;
//	}
////	Serial.println(temp);

//	for (int x = 0; x <= 180; x+=15){
//		servo.write(x);
//		Serial.println(x);
//		delay(1000);
//	}
//	Serial.println(analogRead(flame_pin));

//	lcd.setCursor(0, 1);
//	lcd.print("Hi");
//
	robot.fan(true);
	delay(1000);
	robot.fan(false);
	Serial.println("fan Off");
//	while(1){
//		robot.fan(false);
//	}

//	servo.write(90);
//	blowOutFlame();
//	robot.setStepperAngle(90);
	delay(500000);//


//do what needs to be done in a frame
	//set frame rate for gyro and everything else
	while (millis() - ltime < FRAME) {
	}

	ltime = millis();
	//the gyro can not be used simultaneously with bluetooth
	if (usegyro) {
		Serial.print("Heading, ");
		Serial.print(getHeading());
	}

//  Serial.print(readLeft());
//  Serial.print(" ");
//  Serial.println(readRight());
}
