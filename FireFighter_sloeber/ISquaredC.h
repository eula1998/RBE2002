//*************************************************************//
// TITLE
//*************************************************************//

//!t ISquaredC.h
//!b Class for interfacing with I2C devices.
//!a Dan Oates (WPI Class of 2020)

//!d This class functions as an abstraction wrapper for I2C
//!d communication through the Wire library. Classes for I2C
//!d devices can instantiate an ISquaredC object with the device
//!d address and endian preference (big or little) in order to
//!d more easily and readably access and control registers,
//!d including long values that span multiple bytes.

#pragma once
#include "Arduino.h"
#include "Wire.h"

//*************************************************************//
// CLASS DECLARATION
//*************************************************************//

class ISquaredC {
	public:
		ISquaredC();
		void setup(int, bool);

		void writeByte(int, byte) const;
		byte readByte(int) const;

		void requestBytes(int, int) const;
		int32_t readInt(int);
		uint32_t readUint(int);

	private:
		int i2cAddr = 0x00;
		bool msbFirst = true;

		union {
			byte b[4];
			int32_t i;
			uint32_t u;
		} buffer;

		void readBytes(int);
};
