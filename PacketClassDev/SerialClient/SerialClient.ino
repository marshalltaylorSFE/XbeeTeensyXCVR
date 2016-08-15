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

#define INPUTPINX1 20
#define INPUTPINY1 21
#define INPUTPINX2 22
#define INPUTPINY2 23
#define INPUTPINB1 3
#define INPUTPINB2 2

#define LEDPIN 13
#include "timerModule32.h"
#include "stdint.h"

IntervalTimer myTimer; //Interrupt for Teensy

//**Copy to make a new timer******************//  
//TimerClass32 usTimerA( 20000 ); //20 ms

//**Current list of timers********************//
TimerClass32 debugTimer( 1000000 ); //1 seconds
TimerClass32 serialSendTimer( 3000 ); //0.01 seconds

//Note on TimerClass-
//Change with usTimerA.setInterval( <the new interval> );

uint32_t usTicks = 0;

//  The lock works like this:
//
//    When the interrupt fires, the lock is removed.  Now
//    the main free-wheeling loop can update the change to
//    the timerModules.  Once complete, the lock is replaced
//    so that it can't update more than once per firing
//    of the interrupt

uint8_t usTicksLocked = 1; //start locked out

//**Packet handling stuff*********************//
#define PACKET_LENGTH 24
#define START_SYMBOL '~'

char lastchar;
char rxPacket[PACKET_LENGTH];
char txPacket[PACKET_LENGTH];
char lastPacket[PACKET_LENGTH];
char packetPending = 0;

char packet_ptr;


void setup()
{
  Serial.begin(115200);
  
  Serial1.begin(115200);
  
  pinMode(LEDPIN, OUTPUT);
  pinMode(INPUTPINB1, INPUT_PULLUP);
  pinMode(INPUTPINB2, INPUT_PULLUP);

  // initialize IntervalTimer
  myTimer.begin(serviceUS, 1);  // serviceMS to run every 0.000001 seconds

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
  
  
 
}

void loop()
{
	//Update the timers, but only once per interrupt
	if( usTicksLocked == 0 )
	{
		//**Copy to make a new timer******************//  
		//msTimerA.update(usTicks);
		debugTimer.update(usTicks);
		serialSendTimer.update(usTicks);

		//Done?  Lock it back up
		usTicksLocked = 1;
	}  //The ISR will unlock.

	//**Copy to make a new timer******************//  
	//if(usTimerA.flagStatus() == PENDING)
	//{
	//	//User code
	//}
	
	if(debugTimer.flagStatus() == PENDING)
	{
		//User code
		digitalWrite( LEDPIN, digitalRead( LEDPIN ) ^ 0x01 );
//		
////		Serial.print("Reading lastX1: 0x");
//		Serial.print(lastX1, HEX);
//		Serial.println("");
////		Serial.print("Reading lastY1: 0x");
//		Serial.print(lastY1, HEX);
//		Serial.println("");
////		Serial.print("Reading lastX2: 0x");
//		Serial.print(lastX2, HEX);
//		Serial.println("");
////		Serial.print("Reading lastY2: 0x");
//		Serial.print(lastY2, HEX);
//		Serial.println("");
////		Serial.print("Reading lastB1: 0x");
//		Serial.print(lastB1, HEX);
//		Serial.println("");
////		Serial.print("Reading lastB2: 0x");
//		Serial.print(lastB2, HEX);
//		Serial.println("");
//
////		Serial.print("lastR1: ");
//		Serial.print(lastR1);
//		Serial.println("");
////		Serial.print("lastT1: ");
//		Serial.print(lastT1);
//		Serial.println("");		
//
////		Serial.print("lastDeg1: ");
//		Serial.print( (lastT1 / 6.25) * 360 );
//		Serial.println("");		
//
////		Serial.print("lastR2: ");
//		Serial.print(lastR2);
//		Serial.println("");
////		Serial.print("lastT2: ");
//		Serial.print(lastT2);
//		Serial.println("");		
//
////		Serial.print("lastDeg2: ");
//		Serial.print( (lastT2 / 6.25) * 360 );
//		Serial.println("");		
//
		Serial.println("");
		
	}
	if(serialSendTimer.flagStatus() == PENDING)
	{
	//Check for new data
		uint8_t tempStatus = 0;
		tempStatus = 1; //This forces packet send every time.  Use to block tx when necessary.

		//Read sticks -- a 10 bit number into an 8 bits, uint
		lastX1 = (analogRead(INPUTPINX1)) >> 2;
		lastY1 = (0x3FF - analogRead(INPUTPINY1)) >> 2;
		lastX2 = (analogRead(INPUTPINX2)) >> 2;
		lastY2 = (0x3FF - analogRead(INPUTPINY2)) >> 2;
		//Read the state of the buttons
		lastB1 = 0x1 ^ digitalRead(INPUTPINB1);
		lastB2 = 0x1 ^ digitalRead(INPUTPINB2);
		//Calculate polar coordinates 
		//  Left stick
		//int16_t lastX1i = (int16_t)lastX1 - 0x7D;  //Needs to be well centered by hardcoded value here
		//int16_t lastY1i = (int16_t)lastY1 - 0x82;  //Needs to be well centered by hardcoded value here
		//cart2polar((float)lastX1i / 0x90,(float)lastY1i / 0x90,lastR1,lastT1);
		//
		////  Right stick
		//int16_t lastX2i = (int16_t)lastX2 - 0x83;  //Needs to be well centered by hardcoded value here
		//int16_t lastY2i = (int16_t)lastY2 - 0x7A;  //Needs to be well centered by hardcoded value here
		//cart2polar((float)lastX2i / 0x90,(float)lastY2i / 0x90,lastR2,lastT2);
				
		// If new, ship it!
		if( tempStatus )
		{
			txPacket[0] = '~';
			txPacket[1] = hex2char(packetNumber & 0x000F);
			txPacket[2] = hex2char(0);
			txPacket[3] = hex2char((lastX1 & 0x00F0) >> 4);
			txPacket[4] = hex2char(lastX1 & 0x000F);
			txPacket[5] = hex2char((lastY1 & 0x00F0) >> 4);
			txPacket[6] = hex2char(lastY1 & 0x000F);
			txPacket[7] = hex2char((lastX2 & 0x00F0) >> 4);
			txPacket[8] = hex2char(lastX2 & 0x000F);
			txPacket[9] = hex2char((lastY2 & 0x00F0) >> 4);
			txPacket[10] = hex2char(lastY2 & 0x000F);
			txPacket[11] = hex2char(lastB1 & 0x01);
			txPacket[12] = hex2char(lastB2 & 0x01);
			txPacket[13] = ' ';
			txPacket[14] = ' ';
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
			Serial1.write(0x0A);
			packetNumber++;
		}
	}
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
	
	//Find angle
	if(inputx == 0) inputx = 0.01;
	outputt = atan(inputy / inputx);
	//If either one is negative add pi
	//if y is between 0 and 1, and x is > 0 add another pi
	if((inputx < 0)||(inputy < 0)) outputt += 3.14159;
	if((inputy < 0)&&(inputy > -1)&&(inputx>0)) outputt += 3.14159;

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


