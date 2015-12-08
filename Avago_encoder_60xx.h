#ifndef AVAGO_ENCODER_60XX_H
#define AVAGO_ENCODER_60XX_H
#define LIBRARY_VERSION 0.0.0



class AvagoEncoder60xx
{
	public:
	//Macros
	#define RPM_CONVERSION_CONSTANT			9.5492965855137
	#define ENCODERSPEED_TO_ANGULERSPEED	0.00613592315
	#define RPM_TO_ENCODERSPEED				17.06666666667
	
	//our variables
	byte encoderBitSize; 						//we will default to the 10 bit encoder
	
	unsigned int maxEncoderCount;
	
	//set up times
	unsigned long prevTime;
	
	//set up our pins
	
	int* CSn;										//chip select pin
	int* CLK;										// pin for the clock signal
	int* DO;										//pin for the digital output
	unsigned int prevPos;									//holds the previous position
	unsigned int curPos; 									// used to hold the updated position
	
	
	//functions: 
	AvagoEncoder60xx(int*, int*, int*);
	AvagoEncoder60xx(int*, int*, int*, byte);
	
	unsigned int readSensor();
	
	unsigned int relativePosition();
	
	
	//using doyubles instead of floats to keep compatibility with the PID controller lib
	double getCountsPerSec();
	double angularSpeed();
	double RPM();
	
	double getEncoderSpeedFromRPM(unsigned int rpm);
	
	int toRPM_from_countsPerSec(double cntsPerSec);
	
	private:
	
	
	
	
	
};

#endif