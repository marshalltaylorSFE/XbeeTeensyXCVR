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

#include "SCMD.h" //Use #define USE_ALT_I2C in this file during compilation!!!!!!!!
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values

//**Timers and stuff**************************//
#include "timerModule32.h"
#include "timeKeeper.h"

uint8_t nonLinerLUT[256] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,112,113,114,115,116,117,118,119,120,121,122,123,124,125,127,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,144,145,146,147,148,149,150,151,152,153,154,155,156,158,159,160,161,162,163,164,165,166,167,168,169,170,172,173,174,175,176,177,178,179,180,181,182,183,184,186,187,188,189,190,191,192,193,194,195,196,197,198,199,201,202,203,204,205,206,207,208,209,210,211,212,213,215,216,217,218,219,220,221,222,223,224,225,226,227,229,230,231,232,233,234,235,236,237,238,239,240,241,243,244,245,246,247,248,249,250,251,252,253,254,255};

TimeKeeper failSafeTimer;

//Hardware locations
uint8_t wsPin = 6;
uint8_t debugPin = 8;

uint32_t maxTimer = 60000000;
uint32_t maxInterval = 2000000;

uint16_t i2cFaultsServiced = 0; //last serviced fault

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

SCMDDiagnostics myRobotDiagnostics;

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

	SerialUSB.begin(115200); // Initialize Serial Monitor USB
	Serial1.begin(115200);
	pinMode(debugPin, OUTPUT);
	digitalWrite(debugPin, 0);
	
	delay(3000);
	
	// Send a welcome message to the serial monitor:
	SerialUSB.println("Host Started");

#ifdef __SAMD21G18A__
    tcConfigure(0); //configure the timer to run at <sampleRate>Hertz
	tcStartCounter(); //starts the timer
#endif

	//***** Configure the Motor Driver's Settings *****//
    myMotorDriver.settings.commInterface = I2C_MODE;
    myMotorDriver.settings.I2CAddress = 0x5A; //config pattern "0101" on board for address 0x5A

	//  initialize the driver and enable the motor outputs
	uint8_t tempReturnValue = myMotorDriver.begin();
	while ( tempReturnValue != 0xA9 )
	{
		SerialUSB.print( "ID mismatch, read as 0x" );
		SerialUSB.println( tempReturnValue, HEX );
		delay(500);
		tempReturnValue = myMotorDriver.begin();
	}
	SerialUSB.println( "ID matches 0xA9" );
	
	SerialUSB.print("Waiting for enumeration...");
	while ( myMotorDriver.ready() == false );
	SerialUSB.println("Done.");
	
	//  initialize the driver and enable the motor outputs
	SerialUSB.print("Starting driver... ID = 0x");
	SerialUSB.println(myMotorDriver.begin(), HEX);
	SerialUSB.println();
	
	myMotorDriver.inversionMode( 1, 1 ); //Invert 'B' channel
	while(myMotorDriver.busy());
	
	myMotorDriver.enable(); //enable outputs
	while(myMotorDriver.busy());

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
		while( myMotorDriver.i2cFaults > i2cFaultsServiced )
		{
			SerialUSB.print("*");
			i2cFaultsServiced = myMotorDriver.i2cFaults;
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
					//SerialUSB.print(packet[k]);
					//SerialUSB.println("change marked");
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
				tempInterval = myRobotDiagnostics.U_I2C_RD_ERR;
				if(tempInterval > 0xFFFF) tempInterval = 0xFFFF;
				tempDuration = myRobotDiagnostics.U_I2C_WR_ERR;
				if(tempDuration > 0xFFFF) tempDuration = 0xFFFF;
			}
			if(lastPacketNumber == 11)
			{
				tempInterval = myRobotDiagnostics.U_BUF_DUMPED;
				if(tempInterval > 0xFFFF) tempInterval = 0xFFFF;
				tempDuration = myRobotDiagnostics.LOOP_TIME;
				if(tempDuration > 0xFFFF) tempDuration = 0xFFFF;
			}
			if(lastPacketNumber == 12)
			{
				tempInterval = myRobotDiagnostics.MST_E_ERR;
				if(tempInterval > 0xFFFF) tempInterval = 0xFFFF;
				tempDuration = myRobotDiagnostics.FSAFE_FAULTS;
				if(tempDuration > 0xFFFF) tempDuration = 0xFFFF;
			}
			if(lastPacketNumber == 13)
			{
				tempInterval = myRobotDiagnostics.REG_OOR_CNT;
				if(tempInterval > 0xFFFF) tempInterval = 0xFFFF;
				tempDuration = myRobotDiagnostics.REG_RO_WRITE_CNT;
				if(tempDuration > 0xFFFF) tempDuration = 0xFFFF;
			}
			if(lastPacketNumber == 14)
			{
				tempInterval = myMotorDriver.i2cFaults;
				if(tempInterval > 0xFFFF) tempInterval = 0xFFFF;
				tempDuration = myMotorDriver.readRegister(SCMD_ID);
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
				myRobotDiagnostics.U_I2C_RD_ERR = myMotorDriver.readRegister(SCMD_U_I2C_RD_ERR);
			break;
			case 1:
				myRobotDiagnostics.U_I2C_WR_ERR = myMotorDriver.readRegister(SCMD_U_I2C_WR_ERR);
			break;
			case 2:
				myRobotDiagnostics.U_BUF_DUMPED = myMotorDriver.readRegister(SCMD_U_BUF_DUMPED);
			break;
			case 3:
				myRobotDiagnostics.LOOP_TIME = myMotorDriver.readRegister(SCMD_LOOP_TIME);
			break;
			case 4:
				myRobotDiagnostics.MST_E_ERR = myMotorDriver.readRegister(SCMD_MST_E_ERR);
			break;
			case 5:
				myRobotDiagnostics.FSAFE_FAULTS = myMotorDriver.readRegister(SCMD_FSAFE_FAULTS);
			break;
			case 6:
				myRobotDiagnostics.REG_OOR_CNT = myMotorDriver.readRegister(SCMD_REG_OOR_CNT);
			break;
			case 7:
				myRobotDiagnostics.REG_RO_WRITE_CNT = myMotorDriver.readRegister(SCMD_REG_RO_WRITE_CNT);
			break;
			default:
			break;
		}
		diagReadState++;
		if( diagReadState == 8 ) diagReadState = 0;
		
		//Print cartesian positions:
		//SerialUSB.print("Reading lastX1: 0x");
		SerialUSB.print(lastX1, HEX);
		SerialUSB.println("");
		//SerialUSB.print("Reading lastY1: 0x");
		//SerialUSB.print(lastY1, HEX);
		//SerialUSB.println("");
		//SerialUSB.print("Reading lastX2: 0x");
		//SerialUSB.print(lastX2, HEX);
		//SerialUSB.println("");
		//SerialUSB.print("Reading lastY2: 0x");
		//SerialUSB.print(lastY2, HEX);
		//SerialUSB.println("");
		//SerialUSB.print("Reading lastB1: 0x");
		//SerialUSB.print(lastB1, HEX);
		//SerialUSB.println("");
		//SerialUSB.print("Reading lastB2: 0x");
		//SerialUSB.print(lastB2, HEX);
		//SerialUSB.println("");

		
		//Print polar stick calculation outputs:
		//SerialUSB.print("lastR1: ");
		//SerialUSB.print(lastR1);
		//SerialUSB.println("");
		//SerialUSB.print("lastT1: ");
		//SerialUSB.print(lastT1);
		//SerialUSB.println("");		
		//SerialUSB.print("lastDeg1: ");
		//SerialUSB.print( (lastT1 / 6.25) * 360 );
		//SerialUSB.println("");		

		//SerialUSB.print("lastR2: ");
		//SerialUSB.print(lastR2);
		//SerialUSB.println("");
		//SerialUSB.print("lastT2: ");
		//SerialUSB.print(lastT2);
		//SerialUSB.println("");		
		//SerialUSB.print("lastDeg2: ");
		//SerialUSB.print( (lastT2 / 6.25) * 360 );
		//SerialUSB.println("");		

		//Print drive levels
		//SerialUSB.print("LeftDrive: ");
		//SerialUSB.print(lastLD);
		//SerialUSB.println("");
		//SerialUSB.print("RightDrive: ");
		//SerialUSB.print(lastRD);
		//SerialUSB.println("");		
		//SerialUSB.print("last Drive State: ");
		//SerialUSB.print(lastRD);
		//SerialUSB.println("");	
		//
		//SerialUSB.print("lastDriveState: ");
		//SerialUSB.print(lastDriveState);
		//SerialUSB.println("");	

		//print durations of timers
		SerialUSB.print("0 Itvl: ");
		SerialUSB.println(debugStartTime[0] - debugLastTime[0]);
		SerialUSB.print("0 Dura: ");
		SerialUSB.println(debugStopTime[0] - debugStartTime[0]);
		SerialUSB.print("1 Itvl: ");
		SerialUSB.println(debugStartTime[1] - debugLastTime[1]);
		SerialUSB.print("1 Dura: ");
		SerialUSB.println(debugStopTime[1] - debugStartTime[1]);
		SerialUSB.print("2 Itvl: ");
		//Early
		debugStopTime[2] = usTicks;

		SerialUSB.println(debugStartTime[2] - debugLastTime[2]);
		SerialUSB.print("2 Dura: ");
		SerialUSB.println(debugStopTime[2] - debugStartTime[2]);

		SerialUSB.print("debug usTicks: ");
		SerialUSB.println(debugStartTime[2]);
		
		SerialUSB.println("");
		
		//Print teensy address space
		//uint8_t * address;
		//for( uint32_t tempAddress = 0x40066000; tempAddress <= 0x4006600B; tempAddress++)
		//{
		//	address = (uint8_t *)tempAddress;
		//	SerialUSB.print("Address 0x");
		//	SerialUSB.print((uint32_t)address, HEX);
		//	SerialUSB.print(" contains 0x");
		//	SerialUSB.println(*address, HEX);
		//}
        //
		//SerialUSB.println("");
		
		//Print I2C diagnostic stuff
		SerialUSB.print("U_I2C_RD_ERR: ");
		SerialUSB.println(myRobotDiagnostics.U_I2C_RD_ERR);

		SerialUSB.print("U_I2C_WR_ERR ");
		SerialUSB.println(myRobotDiagnostics.U_I2C_WR_ERR);

		SerialUSB.print("U_BUF_DUMPED: ");
		SerialUSB.println(myRobotDiagnostics.U_BUF_DUMPED);

		SerialUSB.print("LOOP_TIME: ");
		SerialUSB.println(myRobotDiagnostics.LOOP_TIME);
		
		SerialUSB.print("MST_E_ERR: ");
		SerialUSB.println(myRobotDiagnostics.MST_E_ERR);

		SerialUSB.print("FSAFE_FAULTS: ");
		SerialUSB.println(myRobotDiagnostics.FSAFE_FAULTS);

		SerialUSB.print("REG_OOR_CNT: ");
		SerialUSB.println(myRobotDiagnostics.REG_OOR_CNT);

		SerialUSB.print("REG_RO_WRITE_CNT: ");
		SerialUSB.println(myRobotDiagnostics.REG_RO_WRITE_CNT);

		SerialUSB.print("myMotorDriver.i2cFaults: ");
		SerialUSB.println(myMotorDriver.i2cFaults);

	}
}

//If SAMD21, do this
#ifdef __SAMD21G18A__
//this function gets called by the interrupt at <sampleRate>Hertz
void TC5_Handler (void) {
  //YOUR CODE HERE
	uint32_t returnVar = 0;
	if( usTicks >= ( maxTimer + maxInterval ) )
	{
		returnVar = usTicks - maxTimer;

	}
	else
	{
		returnVar = usTicks + 1;
	}
	usTicks = returnVar;
	usTicksLocked = 0;  //unlock
  // END OF YOUR CODE
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
}

/* 
 *  TIMER SPECIFIC FUNCTIONS FOLLOW
 *  you shouldn't change these unless you know what you're doing
 */

//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once
//each time the audio sample frequency period expires.
 void tcConfigure(int sampleRate)
{
 // Enable GCLK for TCC2 and TC5 (timer counter input clock)
 GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
 while (GCLK->STATUS.bit.SYNCBUSY);

 tcReset(); //reset TC5

 // Set Timer counter Mode to 16 bits
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
 // Set TC5 mode as match frequency
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
 //set prescaler and enable TC5
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
 //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
 TC5->COUNT16.CC[0].reg = (uint16_t) (48);
 while (tcIsSyncing());
 
 // Configure interrupt request
 NVIC_DisableIRQ(TC5_IRQn);
 NVIC_ClearPendingIRQ(TC5_IRQn);
 NVIC_SetPriority(TC5_IRQn, 0);
 NVIC_EnableIRQ(TC5_IRQn);

 // Enable the TC5 interrupt request
 TC5->COUNT16.INTENSET.bit.MC0 = 1;
 while (tcIsSyncing()); //wait until TC5 is done syncing 
} 

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5 
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}
#endif

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