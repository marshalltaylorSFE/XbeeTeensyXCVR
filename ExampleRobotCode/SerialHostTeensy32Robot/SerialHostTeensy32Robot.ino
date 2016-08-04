//**********************************************************************//
//  BEERWARE LICENSE
//
//  This code is free for any use provided that if you meet the author
//  in person, you buy them a beer.
//
//  This license block is BeerWare itself.
//
//  Written by:  Marshall Taylor
//  Created:  March 14, 2016
//
//**********************************************************************//
#include <math.h>
#include "HOS_char.h"
#include "ArduinoPSoCMotorDriverAPI.h"

//**Timers and stuff**************************//
#include "timerModule32.h"
#include "timeKeeper.h"

#define SCMD_STATUS          0x00
#define SCMD_ID              0x01
#define SCMD_ADDRESS         0x02
#define SCMD_CONFIG_BITS     0x03
#define SCMD_I2C_FAULTS      0x04
#define SCMD_I2C_RD_ERR      0x05
#define SCMD_I2C_WR_ERR      0x06
#define SCMD_SPI_FAULTS      0x07
#define SCMD_UART_FAULTS     0x08
#define SCMD_UPORT_TIME	     0x09

#define SCMD_FSAFE_TIME      0x18
#define SCMD_FSAFE_FAULTS    0x19

#define SCMD_MA_DRIVE        0x20
#define SCMD_MB_DRIVE        0x21

uint8_t nonLinerLUT[256] = {0,1,3,4,6,7,9,10,12,14,15,17,18,20,21,23,24,26,28,29,31,32,34,35,37,38,40,41,43,44,46,47,48,50,51,53,54,56,57,58,60,61,63,64,65,67,68,69,71,72,73,74,76,77,78,79,81,82,83,84,85,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,108,109,110,111,112,112,113,114,115,115,116,117,117,118,118,119,119,120,121,121,122,122,122,123,123,124,124,124,125,125,125,126,126,126,126,127,127,127,127,127,127,127,127,127,127,127,128,128,128,128,128,128,128,128,128,128,129,129,129,129,130,130,130,131,131,131,132,132,133,133,134,134,135,135,136,136,137,137,138,139,139,140,141,141,142,143,143,144,145,146,147,148,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,168,169,170,171,172,173,175,176,177,178,180,181,182,184,185,186,187,189,190,192,193,194,196,197,198,200,201,203,204,206,207,209,210,212,213,215,216,218,219,221,222,224,225,227,228,230,231,233,234,236,238,239,241,242,244,245,247,249,250,252,253,255,};

TimeKeeper failSafeTimer;

//Hardware locations
uint8_t wsPin = 6;
uint8_t debugPin = 13;

uint32_t maxTimer = 60000000;
uint32_t maxInterval = 2000000;

uint16_t i2cFaults = 0;
uint16_t i2cFaultsServiced = 0; //last serviced fault

IntervalTimer myTimer; //ISR for Teensy


//Other timers
TimerClass32 rxCheckTimer( 150 );
TimerClass32 remoteInputTimer( 1000 );
TimerClass32 debugTimer(200000);

//Serial packet defines
#define PACKET_LENGTH 24
#define START_SYMBOL '~'

char lastchar;
char packet[PACKET_LENGTH];
char txPacket[PACKET_LENGTH];
char lastPacket[PACKET_LENGTH];
char packetPending = 0;

uint8_t failSafe = 0;
uint16_t failSafeCounter = 0;

uint8_t packet_ptr;

//--tick variable for interrupt driven timer1
//elapsedMicros usTickInput = 0;
uint32_t usTicks = 0;
uint8_t usTicksLocked = 1; //start locked out

//Storage from received data
uint16_t lastX1;
uint16_t lastY1;
uint16_t lastX2;
uint16_t lastY2;
uint16_t lastB1;
uint16_t lastB2;
float lastR1;
float lastT1;
float lastR2;
float lastT2;
uint8_t lastPacketNumber = 0;

//debug variables to print
float lastLD;
float lastRD;
uint8_t lastDriveState;

uint8_t SCMD_i2cFaults = 0;
uint8_t SCMD_i2cRdErr = 0;
uint8_t SCMD_i2cWrErr = 0;
uint8_t SCMD_devID = 0;
uint8_t SCMD_fSafeTime = 0;
uint8_t SCMD_fSafeFaults = 0;


#define DEBUG_TIME_SLOTS 5
volatile uint32_t debugLastTime[DEBUG_TIME_SLOTS];
volatile uint32_t debugStartTime[DEBUG_TIME_SLOTS];
volatile uint32_t debugStopTime[DEBUG_TIME_SLOTS];

PSoCMD myMotorDriver;

void setup()
{
	//Build the empty packet
	txPacket[0] = '~';
	txPacket[1] = ' ';
	txPacket[2] = ' ';
	txPacket[3] = ' ';
	txPacket[4] = ' ';
	txPacket[5] = ' ';
	txPacket[6] = ' ';
	txPacket[7] = ' ';
	txPacket[8] = ' ';
	txPacket[9] = ' ';
	txPacket[10] = ' ';
	txPacket[11] = ' ';
	txPacket[12] = ' ';
	txPacket[13] = ' ';
	txPacket[14] = ' ';
	txPacket[15] = ' ';
	txPacket[16] = ' ';
	txPacket[17] = ' ';
	txPacket[18] = ' ';
	txPacket[19] = ' ';
	txPacket[20] = ' ';
	txPacket[21] = ' ';
	txPacket[22] = 0x0D;
	txPacket[23] = 0x0A;

	Serial.begin(115200); // Initialize Serial Monitor USB
	Serial2.begin(115200); // Initialize hardware serial port, pins 0/1
	Serial1.begin(115200);
	pinMode(debugPin, OUTPUT);
	digitalWrite(debugPin, 0);
	
	delay(1000);
	
	// Send a welcome message to the serial monitor:
	Serial.println("Host Started");

	// initialize IntervalTimer
	myTimer.begin(serviceUS, 1);  // serviceMS to run every 0.000001 seconds

	//Start the motor driver
	myMotorDriver.settings.commInterface = I2C_MODE;
	myMotorDriver.settings.I2CAddress = 0x58;
	//myMotorDriver.settings.chipSelectPin = 10;
	//myMotorDriver.settings.invertA = 1;
	myMotorDriver.settings.invertB = 1;
	Serial.println(myMotorDriver.begin(), HEX);

}

void loop()
{
	//**Update the timers*************************//  
	rxCheckTimer.update(usTicks);
	remoteInputTimer.update(usTicks);
	debugTimer.update(usTicks);
	

	//**Read changes from the controller**********//  
	if(rxCheckTimer.flagStatus() == PENDING)
	{
		debugLastTime[0] = debugStartTime[0];
		debugStartTime[0] = usTicks;
		while(Serial1.available())
		{
			lastchar = Serial1.read();
			//look for packet start (START_SYMBOL)
			if( lastchar == START_SYMBOL )
			{
				//Flag that the packet needs to be serviced
				packetPending = 1;
				//Fill packet with null, reset the pointer
				for( int i = 0; i < PACKET_LENGTH; i++ )
				{
					packet[i] = 0;
				}
				//write the start char
				packet[0] = START_SYMBOL;
				//reset the pointer
				packet_ptr = 1;
			}
			else if( ( packet_ptr < PACKET_LENGTH ) && (packet_ptr > 0) )//check for room in the packet, and that the start char has been seen
			{
				//put the char in the packet
				packet[packet_ptr] = lastchar;
				//advance the pointer
				packet_ptr++;
				//turn on LED
			}
			else
			{
				//Just overwrite to the last position
				packet[PACKET_LENGTH - 1] = lastchar;
			}
		}
		
		uint8_t changed = 0;
		debugStopTime[0] = usTicks;
	}
	
	//**Read changes from the controller**********//  
	if(remoteInputTimer.flagStatus() == PENDING)
	{
		debugLastTime[3] = debugStartTime[3];
		debugStartTime[3] = usTicks;
		failSafeTimer.mIncrement(1);
		if(failSafeTimer.mGet() > 100) //Detected delay of communication
		{
			//failed
			if(failSafe == 0)
			{
				//New failure
				failSafe = 1;
				failSafeCounter++;
			}
			myMotorDriver.setDrive(0,0,0);
			myMotorDriver.setDrive(1,0,0);
			lastDriveState = 7;
		}
		while(i2cFaults > i2cFaultsServiced)
		{
			Serial.print("*");
			i2cFaultsServiced = i2cFaults;
			uint8_t * I2C_CTRL1_reg;
			I2C_CTRL1_reg = (uint8_t *)0x40066002;
			*I2C_CTRL1_reg = 0x00;
			delay(10);
			myMotorDriver.reset();
			//delay(200);//Wait for fault to clear
			//myMotorDriver.setDrive(0,0,0);
			//myMotorDriver.setDrive(1,0,0);
			//lastDriveState = 7;
			
		}

		//if the packet is full and the last char is LF or CR, *do something here*
		if((packetPending == 1) && ((packet_ptr == PACKET_LENGTH) && ((packet[PACKET_LENGTH - 1] == 0x0A) || (packet[PACKET_LENGTH - 1] == 0x0D))) )
		{
			//Got a packet, clear failsafe
			failSafeTimer.mClear();
			failSafe = 0;
			
			digitalWrite(13, digitalRead(13)^1);
			//check for new data
			packetPending = 0;
			for(int k = 0; k < PACKET_LENGTH; k++)
			{
				//if(packet[k] != lastPacket[k])
				{
					lastPacket[k] = packet[k];
					//Serial.print(packet[k]);
					//Serial.println("change marked");
				} 
			}
			lastPacketNumber = char2hex(packet[1]);
			lastX1 = char2hex(packet[4]) | (char2hex(packet[3]) << 4);
			lastY1 = char2hex(packet[6]) | (char2hex(packet[5]) << 4);
			lastX2 = char2hex(packet[8]) | (char2hex(packet[7]) << 4);
			lastY2 = char2hex(packet[10]) | (char2hex(packet[9]) << 4);
			lastB1 = char2hex(packet[11]) & 0x01;
			lastB2 = char2hex(packet[12]) & 0x01;
			//Calculate polar coordinates 			
			//  Left stick
			int16_t lastX1i = (int16_t)nonLinerLUT[lastX1] - 0x7D;  //Needs to be well centered by hardcoded value here
			int16_t lastY1i = (int16_t)lastY1 - 0x82;  //Needs to be well centered by hardcoded value here

			
			cart2polar((float)lastX1i / 0x80,(float)lastY1i / 0x80,lastR1,lastT1);

			//  Right stick
			int16_t lastX2i = (int16_t)lastX2 - 0x83;  //Needs to be well centered by hardcoded value here
			int16_t lastY2i = (int16_t)lastY2 - 0x7A;  //Needs to be well centered by hardcoded value here
			cart2polar((float)lastX2i / 0x80,(float)lastY2i / 0x80,lastR2,lastT2);
			
			//Take actions
			if(lastB2)
			{
				lastR1 = lastR1 / 2;
			}
			if(lastB1 && lastB2)
			{
				myMotorDriver.writeRegister(0x18, 50);
			}
			//NOTE: Zones overlap, first executed has priority
			if(lastR1 > 0.05)
			{
				float variable1;
				lastDriveState = 0;
				if(((lastT1 < 0.087)&&(lastT1 >= 0))||((lastT1 < 6.29)&&(lastT1 > 6.196)))
				{
					//In right spin range
					myMotorDriver.setDrive(0,1,lastR1 * 255);
					myMotorDriver.setDrive(1,0,lastR1 * 255);
					lastDriveState = 1;
				}
				else if((lastT1 < 3.229)&&(lastT1 > 3.054))
				{
					//In left spin range
					myMotorDriver.setDrive(0,0,lastR1 * 255);
					myMotorDriver.setDrive(1,1,lastR1 * 255);
					lastDriveState = 2;
				}
				else if((lastT1 < 1.571)&&(lastT1 > 0))
				{
					//In right forward range
					//Get right scale
					variable1 = ((lastT1 - 0.087)/1.484);
					if(variable1 > 1) variable1 = 1;
					myMotorDriver.setDrive(0,1,lastR1 * 255);
					myMotorDriver.setDrive(1,1,lastR1 * 255 * variable1);
					lastDriveState = 3;
					
				}
				else if((lastT1 < 3.142)&&(lastT1 > 1.5))
				{
					//In left forward range
					//Get left scale
					variable1 = 1 - ((lastT1 - 1.571)/1.484);
					if(variable1 > 1) variable1 = 1;
					myMotorDriver.setDrive(0,1,lastR1 * 255 * variable1);
					myMotorDriver.setDrive(1,1,lastR1 * 255);
					lastDriveState = 4;
				}
				else if((lastT1 < 6.2)&&(lastT1 > 4.712))
				{
					//In right backward range
					//Get left scale
					variable1 = 1 - ((lastT1 - 4.712)/1.484);
					if(variable1 > 1) variable1 = 1;
					myMotorDriver.setDrive(0,0,lastR1 * 255 * variable1);
					myMotorDriver.setDrive(1,0,lastR1 * 255);
					lastDriveState = 5;
				}
				else if((lastT1 < 4.8)&&(lastT1 > 3.142))
				{
					//In left backward range
					//Get right scale
					variable1 = ((lastT1 - 3.229)/1.484);
					if(variable1 > 1) variable1 = 1;
					myMotorDriver.setDrive(0,0,lastR1 * 255);
					myMotorDriver.setDrive(1,0,lastR1 * 255 * variable1);
					lastDriveState = 6;
				}
			}
			else
			{   //Stop both
				myMotorDriver.setDrive(0,0,0);
				myMotorDriver.setDrive(1,0,0);
				lastDriveState = 7;
			}
			
			//Send return packet
			uint32_t tempInterval = 0;
			uint32_t tempDuration = 0;
			if(lastPacketNumber < 3)
			{
				tempInterval = debugStartTime[lastPacketNumber] - debugLastTime[lastPacketNumber];
				tempDuration = debugStopTime[lastPacketNumber] - debugStartTime[lastPacketNumber];
				if(tempInterval > 0xFFFF) tempInterval = 0xFFFF;
				if(tempDuration > 0xFFFF) tempDuration = 0xFFFF;
			}
			if(lastPacketNumber == 10)
			{
				tempInterval = failSafeCounter;
				if(tempInterval > 0xFFFF) tempInterval = 0xFFFF;
				tempDuration = i2cFaults;
				if(tempDuration > 0xFFFF) tempDuration = 0xFFFF;
			}
			if(lastPacketNumber == 11)
			{
				tempInterval = SCMD_i2cRdErr;
				if(tempInterval > 0xFFFF) tempInterval = 0xFFFF;
				tempDuration = SCMD_i2cWrErr;
				if(tempDuration > 0xFFFF) tempDuration = 0xFFFF;
			}
			if(lastPacketNumber == 12)
			{
				tempInterval = SCMD_devID;
				if(tempInterval > 0xFFFF) tempInterval = 0xFFFF;
				tempDuration = SCMD_i2cFaults;
				if(tempDuration > 0xFFFF) tempDuration = 0xFFFF;
			}
			if(lastPacketNumber == 13)
			{
				tempInterval = SCMD_fSafeTime;
				if(tempInterval > 0xFFFF) tempInterval = 0xFFFF;
				tempDuration = SCMD_fSafeFaults;
				if(tempDuration > 0xFFFF) tempDuration = 0xFFFF;
			}

			if( 1 )
			{
				txPacket[0] = '~';
				txPacket[1] = hex2char(lastPacketNumber & 0x000F);
				txPacket[2] = hex2char(0);
				txPacket[3] = ' ';
				txPacket[4] = ' ';
				txPacket[5] = ' ';
				txPacket[6] = ' ';
				txPacket[7] = hex2char((tempInterval & 0xF000) >> 12);
				txPacket[8] = hex2char((tempInterval & 0x0F00) >> 8);
				txPacket[9] = hex2char((tempInterval & 0x00F0) >> 4);
				txPacket[10] = hex2char(tempInterval & 0x000F);
				txPacket[11] = hex2char((tempDuration & 0xF000) >> 12);
				txPacket[12] = hex2char((tempDuration & 0x0F00) >> 8);
				txPacket[13] = hex2char((tempDuration & 0x00F0) >> 4);
				txPacket[14] = hex2char(tempDuration & 0x000F);
				txPacket[15] = ' ';
				txPacket[16] = ' ';
				txPacket[17] = ' ';
				txPacket[18] = '.';
				txPacket[19] = '.';
				txPacket[20] = '.';
				txPacket[21] = '.';
				txPacket[22] = 0x0D;
				txPacket[23] = 0x0A;
				for(int i = 0; i < PACKET_LENGTH; i++)
				{
					Serial1.write(txPacket[i]);
				}
				//Serial1.write(0x0A);
			}

		}

		debugStopTime[3] = usTicks;
		debugLastTime[1] = debugLastTime[3];
		debugStartTime[1] = debugStartTime[3];
		debugStopTime[1] = debugStopTime[3];
	}

	//**Debug timer*******************************//  
	if(debugTimer.flagStatus() == PENDING)
	{
		debugLastTime[2] = debugStartTime[2];
		debugStartTime[2] = usTicks;
		digitalWrite( debugPin, digitalRead(debugPin) ^ 1 );
		
		SCMD_i2cFaults = myMotorDriver.readRegister(SCMD_I2C_FAULTS);
		Serial.print("SCMD_i2cFaults = ");
		Serial.println(SCMD_i2cFaults);
		SCMD_i2cRdErr = myMotorDriver.readRegister(SCMD_I2C_RD_ERR);
		SCMD_i2cWrErr = myMotorDriver.readRegister(SCMD_I2C_WR_ERR);
		SCMD_devID = myMotorDriver.readRegister(SCMD_ID);
		Serial.print("SCMD_devID = ");
		Serial.println(SCMD_devID);
		SCMD_fSafeTime = myMotorDriver.readRegister(SCMD_FSAFE_TIME);
		SCMD_fSafeFaults = myMotorDriver.readRegister(SCMD_FSAFE_FAULTS);
		
		//Serial.print("Reading failSafeCounter: ");
		//Serial.print(failSafeCounter);
		//Serial.println("");
		//Serial.print("Reading i2cFaults: ");
		//Serial.print(i2cFaults);
		//Serial.println("");
		
		Serial.print("Reading lastX1: 0x");
		Serial.print(lastX1, HEX);
		Serial.println("");
		Serial.print("Reading lastY1: 0x");
		Serial.print(lastY1, HEX);
		Serial.println("");
		//Serial.print("Reading lastX2: 0x");
		//Serial.print(lastX2, HEX);
		//Serial.println("");
		//Serial.print("Reading lastY2: 0x");
		//Serial.print(lastY2, HEX);
		//Serial.println("");
		//Serial.print("Reading lastB1: 0x");
		//Serial.print(lastB1, HEX);
		//Serial.println("");
		//Serial.print("Reading lastB2: 0x");
		//Serial.print(lastB2, HEX);
		//Serial.println("");

		
		
		Serial.print("lastR1: ");
		Serial.print(lastR1);
		Serial.println("");
		Serial.print("lastT1: ");
		Serial.print(lastT1);
		Serial.println("");		

		//Serial.print("lastDeg1: ");
		//Serial.print( (lastT1 / 6.25) * 360 );
		//Serial.println("");		

		Serial.print("lastR2: ");
		Serial.print(lastR2);
		Serial.println("");
		Serial.print("lastT2: ");
		Serial.print(lastT2);
		Serial.println("");		

		//Serial.print("lastDeg2: ");
		//Serial.print( (lastT2 / 6.25) * 360 );
		//Serial.println("");		

		
		Serial.print("LeftDrive: ");
		Serial.print(lastLD);
		Serial.println("");
		Serial.print("RightDrive: ");
		Serial.print(lastRD);
		Serial.println("");		
		Serial.print("last Drive State: ");
		Serial.print(lastRD);
		Serial.println("");	
		
		Serial.print("lastDriveState: ");
		Serial.print(lastDriveState);
		Serial.println("");	

		Serial.print("0 Itvl: ");
		Serial.println(debugStartTime[0] - debugLastTime[0]);
		Serial.print("0 Dura: ");
		Serial.println(debugStopTime[0] - debugStartTime[0]);
		Serial.print("1 Itvl: ");
		Serial.println(debugStartTime[1] - debugLastTime[1]);
		Serial.print("1 Dura: ");
		Serial.println(debugStopTime[1] - debugStartTime[1]);
		Serial.print("2 Itvl: ");
		//Early
		debugStopTime[2] = usTicks;

		Serial.println(debugStartTime[2] - debugLastTime[2]);
		Serial.print("2 Dura: ");
		Serial.println(debugStopTime[2] - debugStartTime[2]);
		
		Serial.println("");
		uint8_t * address;
		for( uint32_t tempAddress = 0x40066000; tempAddress <= 0x4006600B; tempAddress++)
		{
			address = (uint8_t *)tempAddress;
			Serial.print("Address 0x");
			Serial.print((uint32_t)address, HEX);
			Serial.print(" contains 0x");
			Serial.println(*address, HEX);
		}

		Serial.println("");

	}

	
	////Do the ISR with the teensy built-in timer --
	////  Update the usTicks from usTickInput, alwasy keepin
	//if(usTickInput != usTicks)
	//{
	//	if(usTickInput >= ( maxTimer + maxInterval ))
	//	{
	//	usTickInput = usTickInput - maxTimer;
	//	
	//	}
	//	usTicks = usTickInput;
	//	usTicksLocked = 0;  //unlock
	//}
}

void serviceUS(void)
{
  uint32_t returnVar = 0;
  if(usTicks >= ( maxTimer + maxInterval ))
  {
    returnVar = usTicks - maxTimer;

  }
  else
  {
    returnVar = usTicks + 1;
  }
  usTicks = returnVar;
  usTicksLocked = 0;  //unlock
}



void cart2polar(float inputx, float inputy, float &outputr, float &outputt)
{
	//Protect ranges
	if(inputx > 1) inputx = 1;
	if(inputx < -1) inputx = -1;
	if(inputy > 1) inputy = 1;
	if(inputy < -1) inputy = -1;
	
	//Find radius
	outputr = sqrt(pow(inputx, 2) + pow(inputy, 2));
	if(outputr > 1) outputr = 1;
	
	//Find angle
	if(inputx == 0) inputx = 0.01;
	outputt = atan(inputy / inputx);

	//If either one is negative add pi
	//if y is between 0 and 1, and x is > 0 add another pi
	if((inputx < 0)||(inputy < 0)) outputt += 3.14159;
	if((inputy < 0)&&(inputy >= -1)&&(inputx>0)) outputt += 3.14159;

}