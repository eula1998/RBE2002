//*************************************************************//
// TITLE
//*************************************************************//

//!t Bno055.cpp
//!a Dan Oates (WPI Class of 2020)
//!et
#include "Bno055.h"

float Bno055::accScaleFactor = 0.01000000000;
float Bno055::gyrScaleFactor = 0.00111111111;
float Bno055::eulScaleFactor = 0.00109170305;

//*************************************************************//
// CONSTRUCTOR DEFINITIONS
//*************************************************************//

//!b Constructs Bno055 object.
//!d Axis config is enumerated by the position of the dot on the
//!d relative to the forward-facing orientation of the body.
//!d Examples:
//!d - tlf (top left front)
//!d - drb (down right back)
Bno055::Bno055(ax_t axisConfig) {
	this->axisConfig = axisConfig;
}

//*************************************************************//
// PUBLIC METHOD DEFINITIONS
//*************************************************************//

//!b Initializes chip and returns connection status.
bool Bno055::setup() {
	i2c.setup(I2CADDRESS, false);
	i2c.writeByte(REG_OPMODE, OPMODE_CONFIG);

	Serial.println("testing start");
	i2c.requestBytes(REG_ID, 4);
	bool test1 = (i2c.readUint(1) == ID_CHIP);
	Serial.println("test1");
	bool test2 = (i2c.readUint(1) == ID_ACC);
	Serial.println("test2");
	bool test3 = (i2c.readUint(1) == ID_MAG);
	Serial.println("test3");
	bool test4 = (i2c.readUint(1) == ID_GYR);
	Serial.println("testing done");

	if(test1 && test2 && test3 && test4) {
		i2c.writeByte(REG_PWRMODE, PWRMODE_NORMAL);
		i2c.writeByte(REG_AXIS_CONFIG, AXIS_CONFIG[axisConfig]);
		i2c.writeByte(REG_AXIS_SIGN, AXIS_SIGN[axisConfig]);
		i2c.writeByte(REG_OPMODE, OPMODE_NDOF);
		return true;
	}
	else return false;
}

//!b Stores acceleration values (m/s/s) in given references
void Bno055::accel(float& aX, float& aY, float& aZ) {
	i2c.requestBytes(REG_ACC_X, 6);
	aX = (int16_t)i2c.readInt(2) * accScaleFactor;
	aY = (int16_t)i2c.readInt(2) * accScaleFactor;
	aZ = (int16_t)i2c.readInt(2) * accScaleFactor;
}

//!b Returns x-acceleration (m/s/s)
float Bno055::aX() {
	i2c.requestBytes(REG_ACC_X, 2);
	return (int16_t)i2c.readInt(2) * accScaleFactor;
}

//!b Returns y-acceleration (m/s/s)
float Bno055::aY() {
	i2c.requestBytes(REG_ACC_Y, 2);
	return (int16_t)i2c.readInt(2) * accScaleFactor;
}

//!b Returns z-acceleration (m/s/s)
float Bno055::aZ() {
	i2c.requestBytes(REG_ACC_Z, 2);
	return (int16_t)i2c.readInt(2) * accScaleFactor;
}

//!b Stores gyro values (rad/s) in given references
void Bno055::gyros(float& gX, float& gY, float& gZ) {
	i2c.requestBytes(REG_GYRO_X, 6);
	gX = (int16_t)i2c.readInt(2) * gyrScaleFactor;
	gY = (int16_t)i2c.readInt(2) * gyrScaleFactor;
	gZ = (int16_t)i2c.readInt(2) * gyrScaleFactor;
}

//!b Returns x angular velocity (rad/s)
float Bno055::gX() {
	i2c.requestBytes(REG_GYRO_X, 2);
	return (int16_t)i2c.readInt(2) * gyrScaleFactor;
}

//!b Returns y angular velocity (rad/s)
float Bno055::gY() {
	i2c.requestBytes(REG_GYRO_Y, 2);
	return (int16_t)i2c.readInt(2) * gyrScaleFactor;
}

//!b Returns z angular velocity (rad/s)
float Bno055::gZ() {
	i2c.requestBytes(REG_GYRO_Z, 2);
	return (int16_t)i2c.readInt(2) * gyrScaleFactor;
}

//!b Computes heading, pitch, and roll.
//!d Heading, pitch, and roll are stored in h, p, and r,
//!d respectively.
void Bno055::euler(float& h, float& p, float& r) {
	i2c.requestBytes(REG_EUL_H, 6);
	h = i2c.readUint(2) * eulScaleFactor;
	r = -(int16_t)i2c.readInt(2) * eulScaleFactor;
	p = +(int16_t)i2c.readInt(2) * eulScaleFactor;
}

//!b Returns heading (rad)
float Bno055::heading() {
	i2c.requestBytes(REG_EUL_H, 2);
	return i2c.readUint(2) * eulScaleFactor;
}

//!b Returns pitch (rad)
float Bno055::pitch() {
	i2c.requestBytes(REG_EUL_P, 2);
	return +(int16_t)i2c.readInt(2) * eulScaleFactor;
}

//!b Returns roll (rad)
float Bno055::roll() {
	i2c.requestBytes(REG_EUL_R, 2);
	return -(int16_t)i2c.readInt(2) * eulScaleFactor;
}
