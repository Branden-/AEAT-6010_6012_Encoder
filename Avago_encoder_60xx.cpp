/*
  Arduino Avago encoder library - version 0.0.0
  by Branden Andersen <brandenandersen@gmail.com>
 
  This library is licensed under a MIT license
 */
 
 /* its like this library will be obsoletedd pretty quickly 
 */
 
#if ARDUINO >= 100
	#include "Arduino.h"
#else 
	#include "WProgram.h"
#endif

#include <Avago_encoder_60xx.h>

//use for the power function
#include <math.h>

/**Constructor(...)**********************************************************
 * contructor must be called during set up
 * to set up the pins
 *
 *
 *****************************************************************************/
 
 AvagoEncoder60xx::AvagoEncoder60xx(int* ChipSelectPin, int* ClockSignalPin, 
 int* DigitalOutputPin)
 {
	 CSn = ChipSelectPin;
	 CLK = ClockSignalPin;
	 DO  = DigitalOutputPin;
	 pinMode(*CSn, OUTPUT);
	 pinMode(*CLK, OUTPUT);
	 pinMode(*DO, INPUT);
	 prevPos = curPos = readSensor();
	 encoderBitSize = 10;
	 maxEncoderCount = pow(2,encoderBitSize)-1;
	 prevTime = millis();
	 
 }
 
 AvagoEncoder60xx::AvagoEncoder60xx(int* ChipSelectPin, int* ClockSignalPin, 
 int* DigitalOutputPin, byte encoderBitSize)
 {
	 CSn = ChipSelectPin;
	 CLK = ClockSignalPin;
	 DO  = DigitalOutputPin;
	 pinMode(*CSn, OUTPUT);
	 pinMode(*CLK, OUTPUT);
	 pinMode(*DO, INPUT);
	 this->encoderBitSize = encoderBitSize;
	 maxEncoderCount = pow(2, encoderBitSize)-1;
	 prevTime = millis();
 }
 
 /**readSensor()*********************************************
  * output the current position of the encoder, which is an 
  * absolute position encoder
  *
  ***********************************************************/
  
 unsigned int AvagoEncoder60xx::readSensor()
 {
		unsigned int dataOut = 0;
		  //turn the chip select on
  digitalWrite(*CSn, LOW);
  delayMicroseconds(1); //Waiting for t_CLKFE_, see AEAT-6010 datasheets


  for ( int i = 0; i < encoderBitSize; i++) 
  {
    //creating our clock signal manually
    //set the clock low
    digitalWrite(*CLK, LOW);
    delayMicroseconds(1); //T_CLK/2_
    digitalWrite(*CLK, HIGH);
    delayMicroseconds(1);
    
       // shift all the entering data tot he left
       // and past the pin state to it. MSB first.
    
    dataOut = (dataOut << 1) | digitalRead(*DO);
  }

  // close up the reading
  digitalWrite(*CSn, HIGH); //deselect the encoder from reading
  //Serial.println(dataOut); //lets see
  return dataOut;
}

/**relativePosition()**************************************************
 * return the relative instead of the absolute position that you get in
 * readSensor(). Useful for speed control.
 * assumptions
 * this function is called at regular time intervals
 *
 **********************************************************************/
 
unsigned int AvagoEncoder60xx::relativePosition()
{
	unsigned int relPos;
	curPos = readSensor();
	//a naive approach
/* 	if(curPos > prevPos){
		return relPos = curPos - prevPos;
	}
	else
	{
		
		return prevPos-curPos;
	} */
	
	// a slightly better approach based on knowing the sampling frequency is high enough(F_s > 2*freq)
	// 
	
	// Serial.print("Current Pos," );
	// Serial.println(curPos);
	
	// Serial.print("Prev Pos, ");
	// Serial.println(prevPos);
	if(curPos < prevPos)
	{
		relPos = (curPos + maxEncoderCount) - prevPos;
		prevPos = curPos;
		// Serial.print("Cur < Prev" );
		// Serial.println(relPos);
		
	}
	else
	{
		relPos = curPos - prevPos;
		prevPos = curPos;
		// Serial.print("Cur > Prev" );
		// Serial.println(relPos);
		
	}
	
	//check for erroneous vals and return a grabage val we will check for
	if(relPos > 30){
		return 40;
	}
	return relPos;
}

double AvagoEncoder60xx::getCountsPerSec()
{
	unsigned long now = millis();
	unsigned long deltaT = now - prevTime;
	unsigned int relPos = relativePosition();
	
	// indicate garbage ouput
	if(relPos == 40){
		return -1;
		
	}
	
	unsigned long encoderCountSpeed = ((long double)relPos)/deltaT*1000;
	prevTime = now;
	// Serial.print("Time passed ");
	// Serial.println(deltaT);
	return static_cast<double>(encoderCountSpeed);
}

/**angularSpeed()*********************************************************
 * returns the angular speed in radians of the encoder
 *
 *************************************************************************/


double AvagoEncoder60xx::angularSpeed() 
{
	return ENCODERSPEED_TO_ANGULERSPEED*getCountsPerSec();
}

/**RPM()*********************************************************
 * returns the RPM's 
 *
 *************************************************************************/

double AvagoEncoder60xx::RPM()
{
	
	return angularSpeed()*RPM_CONVERSION_CONSTANT;
}

/**getEncoderSpeedFromRPM()*********************************************************
 * returns the encoder speed in counts per sec from RPM
 *
 *************************************************************************/


double AvagoEncoder60xx::getEncoderSpeedFromRPM(unsigned int rpm)
{
	return rpm*RPM_TO_ENCODERSPEED;
}

int AvagoEncoder60xx::toRPM_from_countsPerSec(double cntsPerSec)
{
	return (int)(cntsPerSec*ENCODERSPEED_TO_ANGULERSPEED*RPM_CONVERSION_CONSTANT);
}