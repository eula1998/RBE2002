#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2018-04-25 11:56:35

#include "Arduino.h"
#include <QTRSensors.h>
#include "L3G.h"
#include "Wire.h"
#include <Arduino.h>
#include "Robot.h"
#include <Servo.h>
#include <LiquidCrystal.h>
#include <Math.h>
#include <LSM303.h>

void gyroreset();
void gyrooff();
void gyroZero();
void readGyro();
void printGyro();
void Accel_Init() ;
void readAccel() ;
void complimentaryFilter();
int checkflame() ;
void getFlameHeight(int maxangle) ;
void blowOutFlame() ;
void initialization() ;
void decision() ;
void rightAlign() ;
void backward2in() ;
void turnRight90() ;
void forward2in() ;
void turnLeft90() ;
void checkFlame() ;
void cliffForward() ;
void foundFlame() ;
void setup() ;
void loop() ;

#include "FireFighter_sloeber.ino"


#endif
