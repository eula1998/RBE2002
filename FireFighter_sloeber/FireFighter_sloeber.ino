#include <QTRSensors.h>//line tracker

#include "L3G.h"
#include "Wire.h"
#include <Arduino.h>
#include "Robot.h"
#include <Servo.h>
#include <LiquidCrystal.h>
#include <Math.h>
#include <LSM303.h>
#include "Bno055.h"

#define FRAME (40) // gyro heading refresh rate, in millis //was 40
//#define FRAME (1000) //test

Robot robot;

//==================================================
//===                   LCD                      ===
//==================================================
const int rs = 40, en = 41, d4 = 42, d5 = 43, d6 = 44, d7 = 45;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//==================================================
//===                   GYRO                     ===
//==================================================
//Bno055 bno(trb);//xyz
//
//const double rad_to_deg = 57.295779513082320876798154814105;
//
//int underoverflow = 0;
//double lastreading;
//double getHeading() {
//	double degree = bno.heading() * rad_to_deg;
//	if(lastreading <= 2 && degree > 355){
//		underoverflow--;
//	}else if(lastreading > 355 && degree < 5){
//		underoverflow++;
//	}
//	lastreading = degree;
//	degree += 360 * underoverflow;
//	return degree;
//}

L3G gyro;

//stores the current accumulated heading read by the current frame
//double heading;
double zero;
//volatile double global_heading;
volatile double heading;

// turn off gyro reading when not needed to reduce interference with other sensor that interrupts
bool usegyro;

/**
 * reset the accumulated heading to zero and get ready for the next read
 */
void zeroHeading() {
	heading = 0;
//	noInterrupts();
	usegyro = true;
	Serial.println("Resetting");
	delay(200);
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
//		global_heading += ((double) gyro.g.x - zero) * FRAME * 0.00000875; //8.75 / 1000 / 1000;
		heading += ((double) gyro.g.x - zero) * FRAME * 0.00000875; //8.75 / 1000 / 1000;
	}
//	return global_heading;
	return heading;
}

void stopGyro() {
	usegyro = false;
	delay(200);
	heading = 0;
//	interrupts();
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
const double fan_height = 8.25; //in ground to fan axle
const double us_fix_dist = 3.0; //can be 2 in
const double us_stepcenter_dist = 2.875; //inch
//TO BE CALCULATED - kinda like registers
int delta_fan_flame_angle;
double flame_height;

int maxangle = 0;
int checkflame() {
	servo.write(90);
//	robot.fan(false);
	robot.setStepperAngle(-90);
	lcd.setCursor(0, 0);
	lcd.print("FINDING FLAME");
	int angle;
	int reading;
	int max = 1024;
	for (int x = 0; x < 13; x++) {//spanning left to right
		flamedata[x] = 1024;
		angle = 15 * x - 90;//-90 to 90
		robot.setStepperAngle(angle);
		for (int y = 120; y >= 45; y -= 15) {
			servo.write(y);
			delay(120);
			reading = analogRead(flame_pin);
			if (reading < flamedata[x]) {
				flamedata[x] = reading;
			}
		}
//		flamedata[x] = analogRead(flame_pin);
		Serial.print("flame, ");
		Serial.print(flamedata[x]);
		Serial.print(", angle, ");
		Serial.print(angle);

		if (flamedata[x] < max) {
			maxangle = angle;
			max = flamedata[x];
			Serial.print(", max");
		}
		Serial.println();
	}
	robot.setStepperAngle(0);
	Serial.print("maxangle, ");
	Serial.print(maxangle);
	Serial.print(", max, ");
	Serial.println(max);
//	if (maxangle <= 0) {
	if (max < 700) {
		lcd.setCursor(0, 0);
		lcd.print("FOUND FLAME");
		lcd.setCursor(0, 1);
		lcd.print("DIRECTION:");
		lcd.print(maxangle);
//			return true;//maybe also return the 45 deg thingy
//		if (maxangle < -45) {
//			return LEFT;
//		}
//		return FORWARD;
		return maxangle;
	}
//	}
	lcd.setCursor(0, 0);
	lcd.print("FLAME NOT FOUND");
	maxangle = -1024;
	return maxangle;
}

void getFlameHeight(int maxangle) { //
//trig math taking into account the offset of the flame sensor away from the fan

	double candle_dist = us_fix_dist + us_stepcenter_dist; //in inches
	flame_height = sin((double) (maxangle - 90) / 180 * 2 * M_PI) * candle_dist
			+ fan_height;
	int hypothenue = hypot(flame_height, candle_dist); //in inches
	delta_fan_flame_angle = atan(delta_fan_flame_dist / hypothenue);
	if (maxangle < 90) {
		delta_fan_flame_angle *= -1;
	}
	Serial.print("Flame height, ");
	Serial.println(flame_height);
}

void blowOutFlame() {
	int max = 1024;
	int maxanglev = 0;
	int flamedata[16];
	int angle;
	bool repeat;
	do {
		repeat = false;
		for (int x = 0; x < 16; x++) {
			angle = 45 + 5 * x;
			servo.write(angle);
			flamedata[x] = analogRead(flame_pin);
			Serial.print("flame, ");
			Serial.print(flamedata[x]);
			Serial.print(", angle, ");
			Serial.print(angle);
			if (max > flamedata[x]) {
				max = flamedata[x];
				maxanglev = angle;
				Serial.print(", max");
			}
			Serial.println();
			delay(100);
		}
		Serial.print("maxanglev, ");
		Serial.print(maxanglev);
		Serial.print(", max, ");
		Serial.println(max);
		getFlameHeight(maxanglev);
		servo.write(maxanglev + delta_fan_flame_angle);
		robot.fan(true);
		delay(5000);
		robot.fan(false);
		servo.write(maxangle);
		if (analogRead(flame_pin) < 500){
			repeat = true;
		}
	} while (repeat);
	lcd.setCursor(0, 1);
//	lcd.print("Flame Out, z: ");
//	lcd.print(flame_height);
}

//==================================================
//===   		  	STATE MACHINE 			     ===
//==================================================

typedef enum {
	INITIALIZATION,
	DECISION,
	RIGHT_ALIGN,
	TURN_LEFT90,
	CHECK_FLAME,
	TURN_RIGHT90,
	FOUND_FLAME,
	CLIFF_FORWARD,
	FORWARD2IN,
	BACKWARD2IN,
	DRIVE_TILL_FLAME,
	DRIVE_TILL_US,
	TURN_TO_FLAME,
	FINISH,
	TURNING_AWAY,
	HEADING_HOME
} State;

State state;
bool cliff = false;
bool foundflame = false;

typedef enum {

} FlameState;

void initialization() {
	if (robot.buttonPressed()) {
		state = DECISION;
//		state = CHECK_FLAME;
	}
}

void decision() {
	if (robot.isRightUS()) {
		state = RIGHT_ALIGN;
		robot.resetEnc();
	} else {
		state = TURN_RIGHT90;
		robot.resetEnc();
//		zeroHeading();
//		gyroreset();
	}
}

void rightAlign() {
//	robot.updateCoor(global_heading);
	robot.updateCoor();
	switch (robot.rightAlign(150)) {
	case TURNLEFT:
		robot.resetEnc();
//		zeroHeading();
//		gyroreset();
		state = TURN_LEFT90;
		break;
	case TURNRIGHT:
		robot.resetEnc();
//		zeroHeading();
//		gyroreset();
		state = TURN_RIGHT90;
		break;
	case ONCLIFF:
		cliff = true;
		state = BACKWARD2IN;
		break;
	default: //timer?
		break;
	}
}

void backward2in() {//actually backward something else
	robot.updateCoor();
	if (robot.driveDist(150, false, 5)) {
		robot.resetEnc();
		state = TURN_LEFT90;
//		zeroHeading();
//		gyroreset();
	}
}

void turnRight90() {
	if (robot.odometryTurn(90, false, 90)) {
		state = FORWARD2IN;
		robot.resetEnc();
		stopGyro();
//		gyrooff();
	}
}

void forward2in() {//actually forward another distance
	robot.updateCoor();
	if (robot.driveDist(120, true, 10)) {
		if (robot.isFrontLine()) {
			cliff = true;
			state = BACKWARD2IN;
//			Serial.println("CLIFF");
//			zeroHeading();
//			gyroreset();
		} else {
			if (robot.isFrontUS()){
				state = TURN_LEFT90;
				Serial.println("FOUND WALL");
//				zeroHeading();
			}else{
				state = CHECK_FLAME;
			}
		}
		robot.resetEnc();
	}
}

void turnLeft90() {
	if (robot.odometryTurn(90, true, 90)) {
		if (cliff) {
			state = CLIFF_FORWARD;
		} else {
			state = CHECK_FLAME;
		}
//		gyrooff();
//		stopGyro();
		robot.resetEnc();
	}
}

State wayToFlame;
void checkFlame() {
	checkflame();
	if (maxangle != -1024) {
		state = FOUND_FLAME;
		wayToFlame = DRIVE_TILL_FLAME;
		foundflame = true;
		zeroHeading();
		//		gyroreset();
		robot.resetEnc();
	} else {
		state = DECISION;
	}
}

void cliffForward() {
//	robot.updateCoor(global_heading);
	robot.updateCoor();
	if (robot.driveForward(150)) {
		delay(100);
		if (robot.isFrontUS()) {
			state = TURN_LEFT90;
//			zeroHeading();
//			gyroreset();
		} else {
			state = DECISION;
		}
		cliff = false;
		robot.resetEnc();
	}
}

int lastFlameReading = 1023;
int r;
void foundFlame() {
	switch (wayToFlame) {
	case TURN_TO_FLAME:
		if (robot.odometryTurn(abs(maxangle), (maxangle < 0), 90)) {
			wayToFlame = DRIVE_TILL_US;
			robot.setStepperAngle(0);
			robot.resetEnc();
			stopGyro();
//			gyrooff();
		}
		break;
	case DRIVE_TILL_FLAME:
		robot.updateCoor();

		robot.setStepperAngle(-90);
		servo.write(90);
		r = analogRead(flame_pin);
		Serial.print("flame: ");
		Serial.println(r);
		if (lastFlameReading < r && r < 200) {
			wayToFlame = TURN_LEFT90;
//			zeroHeading();
//			gyroreset();
			robot.drive(90, 90);
			delay(900);
			robot.stop();
			robot.resetEnc();
		} else {
			robot.drive(100, 100);
			lastFlameReading = r;
		}
		break;
	case TURN_LEFT90:
		if (robot.odometryTurn(90, true, 90)) {
//			gyrooff();
//			stopGyro();
			robot.resetEnc();
			wayToFlame = DRIVE_TILL_US;
			robot.setStepperAngle(0);
		}
		break;
	case DRIVE_TILL_US:
		robot.updateCoor();
//		int reading = robot.readUsFront();
		robot.drive(100, 100);
		if (robot.isFrontUS()) {
			robot.stop();
			blowOutFlame();
			wayToFlame = FINISH;
		}
		break;
	case FINISH:
//		lcd.clear();
//		lcd.setCursor(0, 0);
//		lcd.print("Flame loc: ");
		lcd.setCursor(0, 0);
		lcd.print("x:");
		lcd.print(robot.getX());
		lcd.print("y:");
		lcd.print(robot.getY());
		lcd.setCursor(0, 1);
		lcd.print("z:");
		lcd.print(flame_height);
		wayToFlame = TURNING_AWAY;
//		zeroHeading();
//		gyroreset();
//		robot.resetEnc();
		break;
	case TURNING_AWAY:
		if (robot.odometryTurn(180, true, 90)) {
			wayToFlame = HEADING_HOME;
//			stopGyro();
		}
		break;
	case HEADING_HOME:
		robot.drive(100, 100);
		if (robot.isFrontUS()) {
			robot.stop();
//			blowOutFlame();
			wayToFlame = INITIALIZATION; //stop the state machine
		}
		break;
	default:
		break;
	}
}

//==================================================
//===   		        MAIN 			         ===
//==================================================
void setup() {
	Serial.begin(115200); //stay there for the gyro's sake

	lcd.begin(16, 2);
	//GYRO
//	Wire.begin();
//	//keeps polling the gyro status
//	//side effect: blocks the program till the power is turned on
//	while (!gyro.init()) {
//		Serial.println("Failed to autodetect gyro type!");
//		delay(200);
//	}
//	gyro.enableDefault();
//	delay(1000);

//	gyroZero();
//	Accel_Init();

	//GYRO calibration
//	double calibration = 0;
//	for (int x = 0; x < 100; x++) {
//		gyro.read();
//		calibration += gyro.g.x;
//		delay(25);
//	}
//	zero = calibration / 100.0;

//	Serial.println("bno start");
//	bno.setup();
//	lastreading = bno.heading() * rad_to_deg;
//	robot.ideal_heading = lastreading;
//	Serial.println("bno end");

	servo.attach(servopin);
	servo.write(90);

//	usegyro = false;
//	global_heading = 0;
//	heading = 0;

	state = INITIALIZATION;
//	state = FOUND_FLAME;
	robot.resetEnc();
	wayToFlame = DRIVE_TILL_FLAME;
}

//states for recording the frame rate
int ltime = 0;	//last time
int curtime = 0;

int temp = 0;

int dummystate = 0;
void loop() {
//CANNOT READ ENCODER AND USE GYRO AT THE SAME TIME
//NEED TO HOLD THE STEPPER MOTOR IN PLACE BEFORE TURNING ON THE ROBOT

	//=========== STATE MACHINE================
	lcd.setCursor(0, 0);
	switch (state) {
	case INITIALIZATION:
		initialization();
		lcd.print("INIT");
		Serial.println("INIT");
		break;
	case DECISION:
		lcd.print("DECIDE");
		Serial.println("DECIDE");
		decision();
		break;
	case RIGHT_ALIGN:
		lcd.print("RIGHT ALIGN");
		Serial.println("RIGHT ALIGN");
		rightAlign();
		break;
	case TURN_LEFT90:
		lcd.print("LEFT 90");
		Serial.println("LEFT 90");
		turnLeft90();
		break;
	case TURN_RIGHT90:
		lcd.print("RIGHT 90");
		Serial.println("RIGHT 90");
		turnRight90();
		break;
	case FORWARD2IN:
		lcd.print("FORWARD 2IN");
		Serial.println("FORWARD 2IN");
		forward2in();
		break;
	case BACKWARD2IN:
		lcd.print("BACKWARD 2IN");
		Serial.println("BACKWARD 2IN");
		backward2in();
		break;
	case CLIFF_FORWARD:
		lcd.print("CLIFF FORWARD");
		Serial.println("CLIFF FORWARD");
		cliffForward();
		break;
	case FOUND_FLAME:
		lcd.print("FOUND FLAME");
		Serial.println("FOUND FLAME");
		foundFlame();
		break;
	case CHECK_FLAME:
		lcd.print("CHECK FLAME");
		Serial.println("CHECK FLAME");
		checkFlame();
		break;
	}

//do what needs to be done in a frame
	//set frame rate for gyro and everything else
	while (millis() - ltime < FRAME) {
	}

	ltime = millis();

//	if (usegyro) {
////		complimentaryFilter();
//		Serial.print("Heading, ");
//		Serial.println(bno.heading());
//		Serial.println(getHeading());
//	} else {
	if(!foundflame){
		lcd.setCursor(0, 1);
		lcd.print("x:");
		lcd.print(robot.getX());
		lcd.print("y:");
		lcd.print(robot.getY());

		Serial.print("x, y, ");
		Serial.print(robot.getX());
		Serial.print(" ");
		Serial.print(robot.getY());
		Serial.print(" ideal: ");
		Serial.println(robot.ideal_heading);
	}
	//update coordinates
//	}
}
