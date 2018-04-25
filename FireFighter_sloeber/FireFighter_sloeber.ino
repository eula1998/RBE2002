#include <QTRSensors.h>//line tracker

#include "L3G.h"
#include "Wire.h"
#include <Arduino.h>
#include "Robot.h"
#include <Servo.h>
#include <LiquidCrystal.h>
#include <Math.h>
#include <LSM303.h>

#define FRAME (20) // gyro heading refresh rate, in millis
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

L3G gyro;
LSM303 accel;

bool usegyro = false;


float G_Dt=0.020;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpose timer
long timer1=0;
long timer2=0;

float G_gain=.00875; // gyros gain factor for 250deg/sec
float gyro_x; //gyro x val
float gyro_y; //gyro x val
float gyro_z; //gyro x val
float gyro_xold; //gyro cummulative x value
float gyro_yold; //gyro cummulative y value
float gyro_zold; //gyro cummulative z value
float gerrx; // Gyro x error
float gerry; // Gyro y error
float gerrz; // Gyro 7 error

float A_gain=.00875; // gyros gain factor for 250deg/sec
float accel_x; //gyro x val
float accel_y; //gyro x val
float accel_z; //gyro x val
float accel_xold; //gyro cummulative x value
float accel_yold; //gyro cummulative y value
float accel_zold; //gyro cummulative z value
float aerrx; // Accel x error
float aerry; // Accel y error
float aerrz; // Accel 7 error

void gyroreset(){
//	gyro_x = 0;
//	gyro_y = 0;
//	gyro_z = 0;
//	accel_x = 0;
//	accel_y = 0;
//	accel_z = 0;
	usegyro = true;
	delay(200);
}

void gyrooff(){
	usegyro = false;
	delay(200);
}


void gyroZero(){
// takes 200 samples of the gyro
  for(int i =0;i<200;i++){
  gyro.read();
  gerrx+=gyro.g.x;
  gerry+=gyro.g.y;
  gerrz+=gyro.g.z;
  delay(20);
  }
  gerrx = gerrx/200; // average reading to obtain an error/offset
  gerry = gerry/200;
  gerrz = gerrz/200;

  Serial.println(gerrx); // print error vals
  Serial.println(gerry);
  Serial.println(gerrz);
}

void readGyro(){
  gyro.read(); // read gyro
//  timer=millis(); //reset timer
  gyro_x=(float)(gyro.g.x-gerrx)*G_gain; // offset by error then multiply by gyro gain factor
  gyro_y=(float)(gyro.g.y-gerry)*G_gain;
  gyro_z=(float)(gyro.g.z-gerrz)*G_gain;

  gyro_x = gyro_x*G_Dt; // Multiply the angular rate by the time interval
  gyro_y = gyro_y*G_Dt;
  gyro_z = gyro_z*G_Dt;

  gyro_x +=gyro_xold; // add the displacment(rotation) to the cumulative displacment
  gyro_y += gyro_yold;
  gyro_z += gyro_zold;

  gyro_xold=gyro_x ; // Set the old gyro angle to the current gyro angle
  gyro_yold=gyro_y ;
  gyro_zold=gyro_z ;
}

void printGyro(){
  timer2=millis();

 // The gyro_axis variable keeps track of roll, pitch,yaw based on the complimentary filter
  Serial.print(" GX: ");
  Serial.print(gyro_x);
  Serial.print(" GY: ");
  Serial.print(gyro_y);
  Serial.print(" GZ: ");
  Serial.print(gyro_z);

  Serial.print("  Ax =  ");
  Serial.print(accel_x);
  Serial.print("  Ay =  ");
  Serial.print(accel_y);
  Serial.print("  Az =  ");
  Serial.println(accel_z);
}

void Accel_Init()
{
  accel.init();
  accel.enableDefault();
  Serial.print("Accel Device ID");
  Serial.println(accel.getDeviceType());
  switch (accel.getDeviceType())
  {
    case LSM303::device_D:
      accel.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
      break;
    case LSM303::device_DLHC:
      accel.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
      break;
    default: // DLM, DLH
      accel.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
  }
}


// Reads x,y and z accelerometer registers
void readAccel()
{
  accel.readAcc();

  accel_x = accel.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
  accel_y = accel.a.y >> 4;
  accel_z = accel.a.z >> 4;

  // accelerations in G
  accel_x = (accel_x/256);
  accel_y = (accel_y/256);
  accel_z = (accel_z/256);
  }

void complimentaryFilter(){
  readGyro();
  readAccel();
float x_Acc,y_Acc;
float magnitudeofAccel= (abs(accel_x)+abs(accel_y)+abs(accel_z));
if (magnitudeofAccel > 6 && magnitudeofAccel < 1.2)
{
  x_Acc = atan2(accel_y,accel_z)*180/ PI;
  gyro_x = gyro_x * 0.98 + x_Acc * 0.02;

  y_Acc = atan2(accel_x,accel_z)* 180/PI;
  gyro_y = gyro_y * 0.98 + y_Acc * 0.02;
}

}

//L3G gyro;
//
////stores the current accumulated heading read by the current frame
////double heading;
//double zero;
////volatile double global_heading;
//volatile double heading;
//
//// turn off gyro reading when not needed to reduce interference with other sensor that interrupts
//bool usegyro;
//
///**
// * reset the accumulated heading to zero and get ready for the next read
// */
//void zeroHeading() {
//	heading = 0;
////	noInterrupts();
//	usegyro = true;
//	Serial.println("Resetting");
//	delay(200);
//}
//
///**
// * should only be called once in the loop() function for every frame
// */
//double getHeading() {
//	gyro.read();
//
//	//integrates the gyro reading using the time interval since last read
//	//assuming the robot is turning at a constant angular velocity since the last reading
//	//unit in millis degrees (mdegree/s * ms * 1s/1000ms)
//	//positive = CCW
//	Serial.print("gyro:");
//	Serial.println(gyro.g.x);
//	if (((int) gyro.g.x > 0 ? (int) gyro.g.x : -gyro.g.x) > 500) {
////		global_heading += ((double) gyro.g.x - zero) * FRAME * 0.00000875; //8.75 / 1000 / 1000;
//		heading += ((double) gyro.g.x - zero) * FRAME * 0.00000875; //8.75 / 1000 / 1000;
//	}
////	return global_heading;
//	return heading;
//}
//
//void stopGyro() {
//	usegyro = false;
//	delay(200);
//	heading = 0;
////	interrupts();
//}

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
const double us_fix_dist = 3.0;//can be 2 in
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
	for (int x = 0; x < 13; x++) {
		flamedata[x] = 1024;
		angle = 15 * x - 90;
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
	if (max < 500) {
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
//		robot.fan(true);
//		delay(5000);
//		robot.fan(false);
//		servo.write(maxangle);
//		if (analogRead(flame_pin) < 500){
//			repeat = true;
//		}
	} while (repeat);
	lcd.setCursor(0, 1);
	lcd.print("Flame Out, z: ");
	lcd.print(flame_height);
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
	FINISH
} State;

State state;
bool cliff = false;
bool foundflame = false;

typedef enum {

} FlameState;

void initialization() {
	if (robot.buttonPressed()) {
//		state = DECISION;
		state = CHECK_FLAME;
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
		gyroreset();
	}
}

void rightAlign() {
//	robot.updateCoor(global_heading);
	robot.updateCoor(gyro_x);
	switch (robot.rightAlign(150)) {
	case TURNLEFT:
		robot.resetEnc();
//		zeroHeading();
		gyroreset();
		state = TURN_LEFT90;
		break;
	case TURNRIGHT:
		robot.resetEnc();
//		zeroHeading();
		gyroreset();
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

void backward2in() {
	if (robot.driveDist(-150, 2)) {
		robot.resetEnc();
		state = TURN_LEFT90;
//		zeroHeading();
		gyroreset();
	}
}

void turnRight90() {
	if (robot.turn(90, false, 120, gyro_x)) {
		state = FORWARD2IN;
		robot.resetEnc();
//		stopGyro();
		gyrooff();
	}
}

void forward2in() {
	if (robot.driveDist(150, 2)) {
		if (robot.isFrontLine()) {
			cliff = true;
			state = TURN_LEFT90;
			gyroreset();
		} else {
			state = CHECK_FLAME;
		}
		robot.resetEnc();
	}
}

void turnLeft90() {
	if (robot.turn(90, true, 120, gyro_x)) {
		if (cliff) {
			state = CLIFF_FORWARD;
		} else {
			state = CHECK_FLAME;
		}
		gyrooff();
		robot.resetEnc();
	}
}

State wayToFlame;
void checkFlame() {
	checkflame();
	if (maxangle != -1024){
		state = FOUND_FLAME;
		wayToFlame = TURN_TO_FLAME;
		foundflame = true;
		gyroreset();
		robot.resetEnc();
	}else{
		state = DECISION;
	}
//	switch (checkflame()) {
//	case FORWARD:
//	case LEFT:
//		state = FOUND_FLAME;
//		foundflame = true;
////		wayToFlame =
//		break;
//	default:
//		state = DECISION;
//	}
}

void cliffForward() {
//	robot.updateCoor(global_heading);
	robot.updateCoor(gyro_x);
	if (robot.driveForward(150)) {
		if (robot.isFrontUS()) {
			state = TURN_LEFT90;
			gyroreset();
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
	switch(wayToFlame){
	case TURN_TO_FLAME:
		if (robot.turn(maxangle, false, 90, gyro_x)){
			wayToFlame = DRIVE_TILL_US;
			robot.setStepperAngle(0);
			robot.resetEnc();
			gyrooff();
		}
		break;
	case DRIVE_TILL_FLAME:
		robot.updateCoor(gyro_x);

		robot.setStepperAngle(-90);
		servo.write(90);
		r = analogRead(flame_pin);
		Serial.print("flame: ");
		Serial.println(r);
		if ( lastFlameReading < r && r < 200){
			wayToFlame = TURN_LEFT90;
			gyroreset();
			robot.drive(90, 90);
			delay(900);
			robot.stop();
		}else{
			robot.drive(100, 100);
			lastFlameReading = r;
		}
		break;
	case TURN_LEFT90:
		if (robot.turn(90, true, 120, gyro_x)) {
			gyrooff();
			robot.resetEnc();
			wayToFlame = DRIVE_TILL_US;
			robot.setStepperAngle(0);
		}
		break;
	case DRIVE_TILL_US:
		robot.updateCoor(gyro_x);
//		int reading = robot.readUsFront();
		robot.drive(100, 100);
		if (robot.isFrontUS()){
			robot.stop();
			blowOutFlame();
			wayToFlame = FINISH;
		}
		break;
	case FINISH:
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("Flame loc: ");
		lcd.setCursor(0, 1);
		lcd.print("x:");
		lcd.print((int)robot.getX());
		lcd.print("y:");
		lcd.print((int)robot.getX());
		lcd.print("z:");
		lcd.print(fan_height);
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
	Wire.begin();
	//keeps polling the gyro status
	//side effect: blocks the program till the power is turned on
	while (!gyro.init()) {
		Serial.println("Failed to autodetect gyro type!");
		delay(200);
	}
	gyro.enableDefault();
	delay(1000);
	gyroZero();
	Accel_Init();

	//GYRO calibration
//	double calibration = 0;
//	for (int x = 0; x < 100; x++) {
//		gyro.read();
//		calibration += gyro.g.x;
//		delay(25);
//	}
//	zero = calibration / 100.0;

	servo.attach(servopin);
	servo.write(90);

	usegyro = false;
//	global_heading = 0;
//	heading = 0;

	state = INITIALIZATION;
//	state = FOUND_FLAME;
	wayToFlame = DRIVE_TILL_FLAME;
}

//states for recording the frame rate
int ltime = 0;	//last time
int curtime = 0;

int temp = 0;

void loop() {
//CANNOT READ ENCODER AND USE GYRO AT THE SAME TIME
//NEED TO HOLD THE STEPPER MOTOR IN PLACE BEFORE TURNING ON THE ROBOT
	//==================================================
	//===   		   DEMONSTRATION 		         ===
	//==================================================

//	//simply find flame
//	checkFlame();
//	delay(5000000);

//	//find flame height
//	blowOutFlame();
//	delay(5000000);


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

//	robot.fan(true);
//	delay(5000);
//	robot.fan(false);
//	robot.setStepperAngle(75);
//	robot.setStepperAngle(105);
//	Serial.println("fan Off");

	//=========== STATE MACHINE================
	lcd.setCursor(0, 0);
	lcd.clear();
	switch (state) {
	case INITIALIZATION:
		initialization();
		lcd.print("INIT");
		break;
	case DECISION:
		lcd.print("DECIDE");
		decision();
		break;
	case RIGHT_ALIGN:
		lcd.print("RIGHT ALIGN");
		rightAlign();
		break;
	case TURN_LEFT90:
		lcd.print("LEFT 90");
		turnLeft90();
		break;
	case TURN_RIGHT90:
		lcd.print("RIGHT 90");
//		turnRight90();
		break;
	case FORWARD2IN:
		lcd.print("FORWARD 2IN");
		forward2in();
		break;
	case BACKWARD2IN:
		lcd.print("BACKWARD 2IN");
		backward2in();
		break;
	case CLIFF_FORWARD:
		lcd.print("CLIFF");
		cliffForward();
		break;
	case FOUND_FLAME:
		lcd.print("FOUND FLAME");
		foundFlame();
		break;
	case CHECK_FLAME:
		lcd.print("CHECK FLAME");
		checkFlame();
		break;
	}

//do what needs to be done in a frame
	//set frame rate for gyro and everything else
	while (millis() - ltime < FRAME) {
	}

	ltime = millis();
	if (usegyro) {
		complimentaryFilter();
		Serial.print("Heading, ");
		Serial.println(gyro_x);
	} else {
		Serial.print("x, y, ");
		Serial.print(robot.getX());
		Serial.print(" ");
		Serial.println(robot.getY());
		//update coordinates
	}
}
