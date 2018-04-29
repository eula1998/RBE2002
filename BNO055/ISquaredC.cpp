//*************************************************************//
// CLASS DECLARATION
//*************************************************************//

//!t ISquaredC.cpp
//!a Dan Oates (WPI Class of 2020)

#include "ISquaredC.h"

//*************************************************************//
// CONSTRUCTOR DEFINITIONS
//*************************************************************//

//!b Constructs I2C object.
ISquaredC::ISquaredC() {}

//*************************************************************//
// PUBLIC METHOD DEFINITIONS
//*************************************************************//

//!b Initializes I2C communication.
//!d For devices whos multi-register values begin with the most
//!d significant byte in increasing address order, choose
//!d true for msbFirst. Otherwise, choose false.
//!i I2C address of device
//!i Device msb preference (see above)
void ISquaredC::setup(int i2cAddr, bool msbFirst) {
	this->i2cAddr = i2cAddr;
	this->msbFirst = msbFirst;
	Wire.begin();
}

//!b Writes given byte to I2C register at given address.
void ISquaredC::writeByte(int addr, byte val) const {
	Wire.beginTransmission(i2cAddr);
	Wire.write(addr);
	Wire.write(val);
	Wire.endTransmission(true);
}

//!b Reads byte from I2C register at given address.
byte ISquaredC::readByte(int addr) const {
	Wire.beginTransmission(i2cAddr);
	Wire.write(addr);
	Wire.endTransmission(false);
	Wire.requestFrom(i2cAddr, 1, true);
	return Wire.read();
}

//!b Requests multiple bytes in ascending address order.
//!d After bytes are requested, call "readInt" and "readUint"
//!d until all bytes requested have been read.
//!i Starting register address on device
//!i Total number of bytes to request
void ISquaredC::requestBytes(int addr, int totalBytes) const {
	Wire.beginTransmission(i2cAddr);
	Wire.write(addr);
	Wire.endTransmission(false);
	Wire.requestFrom(i2cAddr, totalBytes, true);
}

//!b Reads signed int of up to 4 bytes from device.
//!d Call this after calling "requestBytes"
//!i Number of bytes to read (1-4)
int32_t ISquaredC::readInt(int numBytes) {
	readBytes(numBytes);
	return buffer.i;
}

//!b Reads unsigned int of up to 4 bytes from device.
//!d Call this after calling "requestBytes"
//!i Number of bytes to read (1-4)
uint32_t ISquaredC::readUint(int numBytes) {
	readBytes(numBytes);
	return buffer.u;
}

//*************************************************************//
// PRIVATE METHOD DEFINITIONS
//*************************************************************//

//!b Reads n bytes into internal buffer
//!d This method is used by "readInt" and "readUint".
//!i Number of bytes to read in (1-4)
void ISquaredC::readBytes(int n) {
	if(msbFirst)
		for(int i=n-1; i>=0; i--)
			buffer.b[i] = Wire.read();
	else
		for(int i=0; i<=n-1; i++)
			buffer.b[i] = Wire.read();
	for(int i=3; i>n-1; i--)
		buffer.b[i] = 0x00;
}
