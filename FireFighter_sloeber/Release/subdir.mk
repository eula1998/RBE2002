################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
INO_SRCS += \
..\FireFighter_sloeber.ino 

CPP_SRCS += \
..\PID.cpp \
..\PololuMotor.cpp \
..\Robot.cpp \
..\StepperMotor.cpp \
..\sloeber.ino.cpp 

LINK_OBJ += \
.\PID.cpp.o \
.\PololuMotor.cpp.o \
.\Robot.cpp.o \
.\StepperMotor.cpp.o \
.\sloeber.ino.cpp.o 

INO_DEPS += \
.\FireFighter_sloeber.ino.d 

CPP_DEPS += \
.\PID.cpp.d \
.\PololuMotor.cpp.d \
.\Robot.cpp.d \
.\StepperMotor.cpp.d \
.\sloeber.ino.cpp.d 


# Each subdirectory must supply rules for building sources it contributes
FireFighter_sloeber.o: ..\FireFighter_sloeber.ino
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:\Users\Lingrui\Desktop\sloeber\/arduinoPlugin/packages/arduino/tools/avr-gcc/4.9.2-atmel3.5.4-arduino2/bin/avr-g++" -c -g -Os -Wall -Wextra -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10802 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\cores\arduino" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\variants\mega" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Encoder\1.4.1" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Encoder\1.4.1\utility" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\L3G\3.0.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\QTRSensors\3.0.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Ultrasonic\2.1.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Ultrasonic\2.1.0\src" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\libraries\Wire" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\libraries\Wire\src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

PID.cpp.o: ..\PID.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:\Users\Lingrui\Desktop\sloeber\/arduinoPlugin/packages/arduino/tools/avr-gcc/4.9.2-atmel3.5.4-arduino2/bin/avr-g++" -c -g -Os -Wall -Wextra -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10802 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\cores\arduino" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\variants\mega" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Encoder\1.4.1" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Encoder\1.4.1\utility" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\L3G\3.0.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\QTRSensors\3.0.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Ultrasonic\2.1.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Ultrasonic\2.1.0\src" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\libraries\Wire" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\libraries\Wire\src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

PololuMotor.cpp.o: ..\PololuMotor.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:\Users\Lingrui\Desktop\sloeber\/arduinoPlugin/packages/arduino/tools/avr-gcc/4.9.2-atmel3.5.4-arduino2/bin/avr-g++" -c -g -Os -Wall -Wextra -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10802 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\cores\arduino" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\variants\mega" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Encoder\1.4.1" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Encoder\1.4.1\utility" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\L3G\3.0.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\QTRSensors\3.0.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Ultrasonic\2.1.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Ultrasonic\2.1.0\src" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\libraries\Wire" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\libraries\Wire\src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

Robot.cpp.o: ..\Robot.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:\Users\Lingrui\Desktop\sloeber\/arduinoPlugin/packages/arduino/tools/avr-gcc/4.9.2-atmel3.5.4-arduino2/bin/avr-g++" -c -g -Os -Wall -Wextra -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10802 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\cores\arduino" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\variants\mega" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Encoder\1.4.1" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Encoder\1.4.1\utility" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\L3G\3.0.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\QTRSensors\3.0.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Ultrasonic\2.1.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Ultrasonic\2.1.0\src" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\libraries\Wire" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\libraries\Wire\src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

StepperMotor.cpp.o: ..\StepperMotor.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:\Users\Lingrui\Desktop\sloeber\/arduinoPlugin/packages/arduino/tools/avr-gcc/4.9.2-atmel3.5.4-arduino2/bin/avr-g++" -c -g -Os -Wall -Wextra -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10802 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\cores\arduino" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\variants\mega" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Encoder\1.4.1" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Encoder\1.4.1\utility" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\L3G\3.0.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\QTRSensors\3.0.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Ultrasonic\2.1.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Ultrasonic\2.1.0\src" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\libraries\Wire" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\libraries\Wire\src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

sloeber.ino.cpp.o: ..\sloeber.ino.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:\Users\Lingrui\Desktop\sloeber\/arduinoPlugin/packages/arduino/tools/avr-gcc/4.9.2-atmel3.5.4-arduino2/bin/avr-g++" -c -g -Os -Wall -Wextra -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10802 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\cores\arduino" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\variants\mega" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Encoder\1.4.1" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Encoder\1.4.1\utility" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\L3G\3.0.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\QTRSensors\3.0.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Ultrasonic\2.1.0" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\libraries\Ultrasonic\2.1.0\src" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\libraries\Wire" -I"C:\Users\Lingrui\Desktop\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.21\libraries\Wire\src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '


