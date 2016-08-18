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
//#define REMOTELINKPORT Serial1
#define REMOTELINKPORT Serial //For now, dump packets on-screen



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
TimerClass32 debugTimer( 1000000 ); //1 seconds
TimerClass32 serialSendTimer( 500000 ); //0.500 seconds
TimerClass32 remoteInputTimer( 10000 );

//--tick variable for interrupt driven timer1
elapsedMicros usTickInput = 0;
uint32_t usTicks = 0;
uint8_t usTicksLocked = 1; //start locked out

//**Packets***********************************//
robotClientPacket packetFromClient;
robotHostPacket packetToClient;

//**Serial Machine****************************//
uCPacketUART dataLinkHandler((HardwareSerial*)&REMOTELINKPORT, 64); //64 byte buffers

void setup()
{
  DEBUGSERIALPORT.begin(115200);
  REMOTELINKPORT.begin(115200);
  
  //dataLinkHandler.initialize();
  
  pinMode( LEDPIN, OUTPUT );
  
  // initialize IntervalTimer
  //myTimer.begin(serviceUS, 1);  // serviceMS to run every 0.000001 seconds

}

void loop()
{
	//Update the timers, but only once per interrupt
	//if( usTicksLocked == 0 )
	{
		//**Give the timers the current time**********//  
		serialSendTimer.update(usTicks);
		remoteInputTimer.update(usTicks);
		debugTimer.update(usTicks);
		
		//Done?  Lock it back up
		//I'm still not sure if this does anything
		//usTicksLocked = 1;
	}

	//**Read the input packet*********************//  
	if(remoteInputTimer.flagStatus() == PENDING)
	{
		dataLinkHandler.flushInputBuffer();
		if( dataLinkHandler.available() == sizeof packetFromClient )
		{
			dataLinkHandler.getPacket( (uint8_t *)&packetFromClient, sizeof packetFromClient );
			//Now do operations on returned packet
			//if( packetFromClient.someVar == blargle ) ...
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
