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

//Radio PAN ID: 0x1A0F

//**UART #defines*****************************//
#define DEBUGSERIALPORT Serial
#define REMOTELINKPORT Serial2 //link port
//#define REMOTELINKPORT Serial //For now, dump packets on-screen



#include "ArduinoPSoCMotorDriverAPI.h"
uint16_t i2cFaults = 0;
PSoCMD myMotorDriver;


//#include "HOS_char.h"
#include <math.h>
#include "uCPacketClass.h"
#include "userPacketDefs.h"

//Globals
uint32_t maxTimer = 60000000;
uint32_t maxInterval = 2000000;

uint32_t packetNumber = 0;

#define leftButtonPin	 8
#define rightButtonPin   9 
#define upButtonPin      6
#define downButtonPin    7
#define startButtonPin   2 
#define selectButtonPin  3
#define aButtonPin       5
#define bButtonPin       4

#define LEDPIN 13
#include "timerModule32.h"
#include "stdint.h"

IntervalTimer myTimer; //Interrupt for Teensy

//**32 bit timer classes *********************//  
TimerClass32 debugTimer( 100000 ); //1 seconds
TimerClass32 serialSendTimer( 50000 ); //0.050 seconds
TimerClass32 remoteInputTimer( 3000 );
TimerClass32 robotMotionTimer(10000);
TimerClass32 motorUpdateTimer( 10000 );
TimerClass32 debounceTimer(5000);
TimerClass32 ledToggleTimer( 333000 );
TimerClass32 ledToggleFastTimer( 100000 );

//--tick variable for interrupt driven timer1
elapsedMicros usTickInput = 0;
uint32_t usTicks = 0;
uint8_t usTicksLocked = 1; //start locked out

//**Packets***********************************//
robotClientPacket packetFromClient;
robotHostPacket packetToClient;

//**Serial Machine****************************//
uCPacketUART dataLinkHandler((HardwareSerial*)&REMOTELINKPORT, 64); //64 byte buffers

//LEDs
#include <Adafruit_NeoPixel.h>
uint8_t wsPin = 6;
Adafruit_NeoPixel indicators = Adafruit_NeoPixel(2, wsPin, NEO_GRB + NEO_KHZ800);

//Control system defines
uint8_t frontSwap = 0;

//--Robot state machine
#include "RobotMotion.h"
RobotMotion myRobot;


void setup()
{
  DEBUGSERIALPORT.begin(115200);
	delay(1000);
	DEBUGSERIALPORT.println("Program Started");
	REMOTELINKPORT.begin(115200);
  
  //dataLinkHandler.initialize();
  
  pinMode( LEDPIN, OUTPUT );
  
  // initialize IntervalTimer
  //myTimer.begin(serviceUS, 1);  // serviceMS to run every 0.000001 seconds

  indicators.begin();
  indicators.setPixelColor(0, 0xFF20AF);
  indicators.setPixelColor(1, 0xFF20AF);
  indicators.show(); // Initialize all pixels

  //Ready the state machines
  myRobot.init();
  //Start the motor driver
  myMotorDriver.settings.commInterface = I2C_MODE;
  myMotorDriver.settings.I2CAddress = 0x5A;
  //myMotorDriver.settings.chipSelectPin = 10;
  //myMotorDriver.settings.invertA = 1;
  myMotorDriver.settings.invertB = 1;
  Serial.println(myMotorDriver.begin(), HEX);
  
}

void loop()
{
	//Update the timers, but only once per interrupt
	//if( usTicksLocked == 0 )
	{
		//**Give the timers the current time**********//  
		serialSendTimer.update(usTicks);
		remoteInputTimer.update(usTicks);
		ledToggleTimer.update(usTicks);
		ledToggleFastTimer.update(usTicks);
		robotMotionTimer.update(usTicks);
		motorUpdateTimer.update(usTicks);
		debounceTimer.update(usTicks);
		debugTimer.update(usTicks);
		
		//Done?  Lock it back up
		//I'm still not sure if this does anything
		//usTicksLocked = 1;
	}

	//**Read the input packet*********************//  
	if(remoteInputTimer.flagStatus() == PENDING)
	{
		dataLinkHandler.burstReadInputBuffer();  //Reads to next packet end
		if( dataLinkHandler.available() == sizeof packetFromClient )
		{
			dataLinkHandler.getPacket( (uint8_t *)&packetFromClient, sizeof packetFromClient );
			//Now do operations on returned packet
			//if( packetFromClient.someVar == blargle ) ...
		}
		else if( dataLinkHandler.available() != 0 ) //we have a wrong size packet
		{
			dataLinkHandler.abandonRxPacket();
		}
	}

	if(serialSendTimer.flagStatus() == PENDING)
	{
	//Check for new data
		uint8_t tempStatus = 0;
		tempStatus = 1; //This forces packet send every time.  Use to block tx when necessary.

		// If new, ship it!
		if( tempStatus )
		{
			dataLinkHandler.sendPacket( (uint8_t *)&packetToClient, sizeof packetToClient );
		}
	}
	//**Copy to make a new timer******************//  
	//if(usTimerA.flagStatus() == PENDING)
	//{
	//	//User code
	//}
	//**Debounce timer****************************//  
	if(debounceTimer.flagStatus() == PENDING)
	{
		myRobot.timersMIncrement(5);
	
	}
	//**Send commands timer***********************// 
	if(motorUpdateTimer.flagStatus() == PENDING)
	{
		if( frontSwap )
		{
			indicators.setPixelColor(1, 0xFF0000);
			indicators.setPixelColor(0, 0x00FF00);
		}
		else
		{
			indicators.setPixelColor(1, 0x00FF00);
			indicators.setPixelColor(0, 0xFF0000);
		}
		indicators.show();
		
		//set both drive levels

	}		
	//**Process the panel and state machine***********//  
	if(robotMotionTimer.flagStatus() == PENDING)
	{
		//Provide inputs
		
		//Tick the machine
		myRobot.processMachine();
		Serial.print(myRobot.velocity);
		Serial.print(" ");
		//Deal with outputs
		if( myRobot.velocity > 0.1 )
		{
			myMotorDriver.setDrive(0, 0, myRobot.velocity * 255); //chan, dir, lev
			myMotorDriver.setDrive(1, 0, myRobot.velocity * 255);
		}
		else if( myRobot.velocity < -0.1 )
		{
			myMotorDriver.setDrive(0, 1, myRobot.velocity * -255); //chan, dir, lev
			myMotorDriver.setDrive(1, 1, myRobot.velocity * -255);
		}
		else
		{
			myMotorDriver.setDrive(0, 0, 0); //chan, dir, lev
			myMotorDriver.setDrive(1, 0, 0);
		}
		frontSwap = myRobot.frontSwap;

	}
	if(debugTimer.flagStatus() == PENDING)
	{
		//User code
		digitalWrite( LEDPIN, digitalRead( LEDPIN ) ^ 0x01 );
		
		uint8_t * index = (uint8_t *)&packetFromClient;
		DEBUGSERIALPORT.println("RX Packet Dump:");
		for( int i = 0; i < (int)sizeof packetFromClient; i++ )
		{
			DEBUGSERIALPORT.print("0x");
			DEBUGSERIALPORT.print(i, HEX);
			DEBUGSERIALPORT.print(": 0x");
			DEBUGSERIALPORT.println( *index++, HEX );
		}
		DEBUGSERIALPORT.println("");
		DEBUGSERIALPORT.print("Payload in binary: b");
		for(int i = 0; i < 8; i++)
		{
			if(((( packetFromClient.gamepadButtons ) >> (7 - i) ) & 0x01 ) == 0)
			{
				DEBUGSERIALPORT.print("0");
			}
			else
			{
				DEBUGSERIALPORT.print("1");
			}
			
		}
		DEBUGSERIALPORT.println("");
		DEBUGSERIALPORT.print("32 bit test word: 0x");
		DEBUGSERIALPORT.println(packetFromClient.testHex, HEX);
		DEBUGSERIALPORT.println("");
		DEBUGSERIALPORT.println("");
		
		index = (uint8_t *)&packetToClient;
		DEBUGSERIALPORT.println("TX Packet Dump:");
		for( int i = 0; i < (int)sizeof packetToClient; i++ )
		{
			DEBUGSERIALPORT.print("0x");
			DEBUGSERIALPORT.print(i, HEX);
			DEBUGSERIALPORT.print(": 0x");
			DEBUGSERIALPORT.println( *index++, HEX );
		}

		DEBUGSERIALPORT.println("");
		
	}

	//Do the ISR with the teensy built-in timer
	if(usTickInput != usTicks)
	{
		uint32_t returnVar = 0;
		if(usTickInput >= ( maxTimer + maxInterval ))
		{
		usTickInput = usTickInput - maxTimer;
		
		}
		usTicks = usTickInput;
		//usTicksMutex = 0;  //unlock
	}
}
