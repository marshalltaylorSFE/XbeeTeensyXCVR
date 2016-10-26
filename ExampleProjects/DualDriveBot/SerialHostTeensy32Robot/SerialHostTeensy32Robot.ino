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

#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values

//**Timers and stuff**************************//
#include "timerModule32.h"
#include "timeKeeper.h"

uint8_t nonLinerLUT[256] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,112,113,114,115,116,117,118,119,120,121,122,123,124,125,127,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,144,145,146,147,148,149,150,151,152,153,154,155,156,158,159,160,161,162,163,164,165,166,167,168,169,170,172,173,174,175,176,177,178,179,180,181,182,183,184,186,187,188,189,190,191,192,193,194,195,196,197,198,199,201,202,203,204,205,206,207,208,209,210,211,212,213,215,216,217,218,219,220,221,222,223,224,225,226,227,229,230,231,232,233,234,235,236,237,238,239,240,241,243,244,245,246,247,248,249,250,251,252,253,254,255};

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

uint8_t diagReadState = 0;

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

//***** Create the Motor Driver object*****//
SCMD myMotorDriver;

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

	//***** Configure the Motor Driver's Settings *****//
  
	//  .commInter face can be I2C_MODE or SPI_MODE
	myMotorDriver.settings.commInterface = I2C_MODE;
	//myMotorDriver.settings.commInterface = SPI_MODE;
	
	//  set address if I2C configuration selected with the config jumpers
	myMotorDriver.settings.I2CAddress = 0x5A; //config pattern "0101" on board for address 0x5A
	//  set chip select if SPI selected with the config jumpers
	myMotorDriver.settings.chipSelectPin = 10;
	
	delay(1000);
	
	//  initialize the driver and enable the motor outputs
	Serial.print("Starting driver... ID = 0x");
	Serial.println(myMotorDriver.begin(), HEX);
	Serial.println();
	
	myMotorDriver.inversionMode( 1, 1 ); //Invert 'B' channel

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
	if( remoteInputTimer.flagStatus() == PENDING)
	{
		debugLastTime[3] = debugStartTime[3];
		debugStartTime[3] = usTicks;
		failSafeTimer.mIncrement(1);
		if( failSafeTimer.mGet() > 1000 ) //Detected delay of communication
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
		while( i2cFaults > i2cFaultsServiced )
		{
			Serial.print("*");
			i2cFaultsServiced = i2cFaults;
			myMotorDriver.reset();
			//delay(200);//Wait for fault to clear
			myMotorDriver.setDrive(0,0,0);
			myMotorDriver.setDrive(1,0,0);
			lastDriveState = 7;
			
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
		
		switch(diagReadState)
		{
			case 0:
				SCMD_i2cFaults = myMotorDriver.readRegister(SCMD_MST_E_ERR);
			break;
			case 1:
				SCMD_i2cRdErr = myMotorDriver.readRegister(SCMD_E_I2C_RD_ERR);
			break;
			case 2:
				SCMD_i2cWrErr = myMotorDriver.readRegister(SCMD_E_I2C_WR_ERR);
			break;
			case 3:
				SCMD_devID = myMotorDriver.readRegister(SCMD_ID);
			break;
			case 4:
				SCMD_fSafeTime = myMotorDriver.readRegister(SCMD_FSAFE_TIME);
			break;
			case 5:
				SCMD_fSafeFaults = myMotorDriver.readRegister(SCMD_FSAFE_FAULTS);
			break;
			default:
			break;
		}
		diagReadState++;
		if( diagReadState == 6 ) diagReadState = 0;
		
		//Print cartesian positions:
		//Serial.print("Reading lastX1: 0x");
		Serial.print(lastX1, HEX);
		Serial.println("");
		//Serial.print("Reading lastY1: 0x");
		//Serial.print(lastY1, HEX);
		//Serial.println("");
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

		
		//Print polar stick calculation outputs:
		//Serial.print("lastR1: ");
		//Serial.print(lastR1);
		//Serial.println("");
		//Serial.print("lastT1: ");
		//Serial.print(lastT1);
		//Serial.println("");		
		//Serial.print("lastDeg1: ");
		//Serial.print( (lastT1 / 6.25) * 360 );
		//Serial.println("");		

		//Serial.print("lastR2: ");
		//Serial.print(lastR2);
		//Serial.println("");
		//Serial.print("lastT2: ");
		//Serial.print(lastT2);
		//Serial.println("");		
		//Serial.print("lastDeg2: ");
		//Serial.print( (lastT2 / 6.25) * 360 );
		//Serial.println("");		

		//Print drive levels
		//Serial.print("LeftDrive: ");
		//Serial.print(lastLD);
		//Serial.println("");
		//Serial.print("RightDrive: ");
		//Serial.print(lastRD);
		//Serial.println("");		
		//Serial.print("last Drive State: ");
		//Serial.print(lastRD);
		//Serial.println("");	
		//
		//Serial.print("lastDriveState: ");
		//Serial.print(lastDriveState);
		//Serial.println("");	

		//print durations of timers
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

		Serial.print("debug usTicks: ");
		Serial.println(debugStartTime[2]);
		
		Serial.println("");
		
		//Print teensy address space
		//uint8_t * address;
		//for( uint32_t tempAddress = 0x40066000; tempAddress <= 0x4006600B; tempAddress++)
		//{
		//	address = (uint8_t *)tempAddress;
		//	Serial.print("Address 0x");
		//	Serial.print((uint32_t)address, HEX);
		//	Serial.print(" contains 0x");
		//	Serial.println(*address, HEX);
		//}
        //
		//Serial.println("");
		
		//Print I2C diagnostic stuff
		Serial.print("failSafeCounter: ");
		Serial.println(failSafeCounter);

		Serial.print("i2cFaults ");
		Serial.println(i2cFaults);

		Serial.print("SCMD_i2cRdErr: ");
		Serial.println(SCMD_i2cRdErr);

		Serial.print("SCMD_i2cWrErr: ");
		Serial.println(SCMD_i2cWrErr);
		
		Serial.print("SCMD_devID: ");
		Serial.println(SCMD_devID);

		Serial.print("SCMD_i2cFaults: ");
		Serial.println(SCMD_i2cFaults);

		Serial.print("SCMD_fSafeTime: ");
		Serial.println(SCMD_fSafeTime);

		Serial.print("SCMD_fSafeFaults: ");
		Serial.println(SCMD_fSafeFaults);
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