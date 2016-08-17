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
#include "HOS_char.h"
#include <math.h>
#include "uCPacketClass.h"
#include "userPacketDefs.h"

//Globals
uint32_t maxTimer = 60000000;
uint32_t maxInterval = 2000000;

uint32_t packetNumber = 0;

uint16_t lastX1;
uint16_t lastY1;
uint16_t lastX2;
uint16_t lastY2;

float lastR1;
float lastT1;
float lastR2;
float lastT2;
uint8_t lastB1;
uint8_t lastB2;

#define leftButtonPin	 2
#define rightButtonPin   3 
#define upButtonPin      4
#define downButtonPin    5
#define startButtonPin   6 
#define selectButtonPin  7
#define aButtonPin       8
#define bButtonPin       9

#define LEDPIN 13
#include "timerModule32.h"
#include "stdint.h"

IntervalTimer myTimer; //Interrupt for Teensy

//**32 bit timer classes *********************//  
TimerClass32 debugTimer( 1000000 ); //1 seconds
TimerClass32 serialSendTimer( 3000 ); //0.01 seconds
TimerClass32 remoteInputTimer( 10000 );

uint32_t usTicks = 0;
uint8_t usTicksLocked = 1; //start locked out

//**UART #defines*****************************//
#define DEBUGSERIALPORT Serial
#define REMOTELINKPORT Serial1

//**Packets***********************************//
robotClientPacket packetToHost;
robotHostPacket packetFromHost;

//**Serial Machine****************************//
uCPacketUART dataLinkHandler(&REMOTELINKPORT, 64); //64 byte buffer

void setup()
{
  DEBUGSERIALPORT.begin(115200);
  REMOTELINKPORT.begin(115200);
  
  //dataLinkHandler.initialize();
  
  pinMode( LEDPIN, OUTPUT );
  pinMode( leftButtonPin   , INPUT_PULLUP );
  pinMode( rightButtonPin  , INPUT_PULLUP );
  pinMode( upButtonPin     , INPUT_PULLUP );
  pinMode( downButtonPin   , INPUT_PULLUP );
  pinMode( startButtonPin  , INPUT_PULLUP );
  pinMode( selectButtonPin , INPUT_PULLUP );
  pinMode( aButtonPin      , INPUT_PULLUP );
  pinMode( bButtonPin      , INPUT_PULLUP );

  // initialize IntervalTimer
  myTimer.begin(serviceUS, 1);  // serviceMS to run every 0.000001 seconds
 
}

void loop()
{
	//Update the timers, but only once per interrupt
	if( usTicksLocked == 0 )
	{
		//**Give the timers the current time**********//  
		serialSendTimer.update(usTicks);
		remoteInputTimer.update(usTicks);
		debugTimer.update(usTicks);
		
		//Done?  Lock it back up
		//I'm still not sure if this does anything
		usTicksLocked = 1;
	}

	//**Read the input packet*********************//  
	if(remoteInputTimer.flagStatus() == PENDING)
	{
		dataLinkHandler.flushInputBuffer();
		if( dataLinkHandler.available() == sizeof packetFromHost )
		{
			dataLinkHandler.getPacket( (uint8_t *)&packetFromHost, sizeof packetFromHost );
			//Now do operations on returned packet
			//if( packetFromHost.someVar == blargle ) ...
		}
	}

	if(serialSendTimer.flagStatus() == PENDING)
	{
	//Check for new data
		uint8_t tempStatus = 0;
		tempStatus = 1; //This forces packet send every time.  Use to block tx when necessary.

		//Read sticks -- a 10 bit number into an 8 bits, uint
		uint8_t leftButtonState = digitalRead(leftButtonPin) << 7;
		uint8_t rightButtonState = digitalRead(rightButtonPin) << 6;
		uint8_t upButtonState = digitalRead(upButtonPin) << 5;
		uint8_t downButtonState = digitalRead(downButtonPin) << 4;
		uint8_t startButtonState = digitalRead(startButtonPin) << 3;
		uint8_t selectButtonState = digitalRead(selectButtonPin) << 2;
		uint8_t aButtonState = digitalRead(aButtonPin) << 1;
		uint8_t bButtonState = digitalRead(bButtonPin);
		
		uint8_t outputByte = leftButtonState | rightButtonState | upButtonState | downButtonState | startButtonState | selectButtonState | aButtonState | bButtonState;
		
		packetToHost.gamepadButtons = outputByte; // The payload

		// If new, ship it!
		if( tempStatus )
		{
			dataLinkHandler.sendPacket( (uint8_t *)&packetToHost, sizeof packetToHost );
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
		
		uint8_t * index = (uint8_t *)&packetToHost;
		DEBUGSERIALPORT.print("TX Packet Dump: 0x");
		for( int i = 0; i < (int)sizeof packetToHost; i++ )
		{
			DEBUGSERIALPORT.print("ADDR: 0x");
			DEBUGSERIALPORT.println( *index++, HEX );
		}

		DEBUGSERIALPORT.print("Payload binary: ");
		DEBUGSERIALPORT.print(dataLinkHandler.gamepadButtons, BIN);
		DEBUGSERIALPORT.println("");
		
		index = (uint8_t *)&packetFromHost;
		DEBUGSERIALPORT.print("Last RX Packet Dump: 0x");
		for( int i = 0; i < (int)sizeof packetFromHost; i++ )
		{
			DEBUGSERIALPORT.print("ADDR: 0x");
			DEBUGSERIALPORT.println( *index++, HEX );
		}

		DEBUGSERIALPORT.println("");
		
	}
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


