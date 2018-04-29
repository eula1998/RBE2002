//*************************************************************//
// TITLE
//*************************************************************//

//!t Bno055.h
//!b Class for interfacing with the Bno055 IMU.
//!a Dan Oates (WPI Class of 2020)

//!d The Bno055 9-axis IMU computes absolute orientation
//!d (pitch, yaw, roll) angular velocity, and gravity-
//!d adjusted acceleration (all SI units).
//!d The euler angle conventions are as follows:
//!d - Heading increases modulo 2pi when body rotates clockwise.
//!d - Positive pitch tilts the top of the body forwards.
//!d - Positive roll tilts the top of the body to the right.

#pragma once
#include "ISquaredC.h"

//*************************************************************//
// MACROS AND CONSTANTS
//*************************************************************//

#define I2CADDRESS         0x28
#define REG_ID             0x00
	#define ID_CHIP        0xA0
	#define ID_ACC         0xFB
	#define ID_MAG         0x32
	#define ID_GYR         0x0F
#define REG_PWRMODE        0x3E
	#define PWRMODE_NORMAL 0x00
#define REG_OPMODE         0x3D
	#define OPMODE_CONFIG  0x00
	#define OPMODE_IMU     0x08
	#define OPMODE_NDOF    0x0C
#define REG_AXIS_CONFIG    0x41
#define REG_AXIS_SIGN      0x42

#define REG_ACC_X 0x28
#define REG_ACC_Y 0x2A
#define REG_ACC_Z 0x2C

#define REG_EUL_H 0x1A
#define REG_EUL_R 0x1C
#define REG_EUL_P 0x1E

#define REG_GYRO_X 0x14
#define REG_GYRO_Y 0x16
#define REG_GYRO_Z 0x18

const byte AXIS_CONFIG[8] = {
	0x21, 0x24, 0x24, 0x21, 0x24, 0x21, 0x21, 0x24 };
const byte AXIS_SIGN[8] = {
	0x04, 0x00, 0x06, 0x02, 0x03, 0x01, 0x07, 0x05 };

typedef enum {
	tlf = 0, trf = 1, tlb = 2, trb = 3,
	dlf = 4, drf = 5, dlb = 6, drb = 7
} ax_t;

//*************************************************************//
// CLASS DECLARATION
//*************************************************************//

class Bno055 {
	public:
		Bno055(ax_t);
		bool setup();

		void accel(float&, float&, float&);
		float aX();
		float aY();
		float aZ();

		void gyros(float&, float&, float&);
		float gX();
		float gY();
		float gZ();

		void euler(float&, float&, float&);
		float heading();
		float pitch();
		float roll();

	private:
		static float accScaleFactor;
		static float gyrScaleFactor;
		static float eulScaleFactor;

		ISquaredC i2c;
		int axisConfig;
};
