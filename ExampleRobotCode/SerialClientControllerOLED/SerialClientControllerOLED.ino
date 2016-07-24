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
#include "stdint.h"
#include "HOS_char.h"
#include <math.h>
#include <Wire.h>  // Include Wire if you're using I2C
#include <SPI.h>  // Include SPI if you're using SPI
#include "SFE_MicroOLED.h"  // Include the SFE_MicroOLED library
#include "timerModule32.h"

//Globals
uint32_t maxTimer = 60000000;
uint32_t maxInterval = 2000000;

uint8_t displayIncrement = 0;

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

#define DEBUG_TIME_SLOTS 20
volatile uint32_t debugLastTime[DEBUG_TIME_SLOTS];
volatile uint32_t debugStartTime[DEBUG_TIME_SLOTS];
volatile uint32_t debugStopTime[DEBUG_TIME_SLOTS];
volatile uint32_t remoteNumbers[20];


uint8_t displayPage = 0;
uint8_t peakHold = 0;
uint32_t peakValues[2][DEBUG_TIME_SLOTS];
//////////////////////////
// MicroOLED Definition //
//////////////////////////
#define PIN_RESET 9  // Connect RST to pin 9 (req. for SPI and I2C)
#define PIN_DC    8  // Connect DC to pin 8 (required for SPI)
#define PIN_CS    10 // Connect CS to pin 10 (required for SPI)
#define DC_JUMPER 0
// Also connect pin 13 to SCK and pin 11 to MOSI

//////////////////////////////////
// MicroOLED Object Declaration //
//////////////////////////////////
// Declare a MicroOLED object. The parameters include:
// 1 - Reset pin: Any digital pin
// 2 - D/C pin: Any digital pin (SPI mode only)
// 3 - CS pin: Any digital pin (SPI mode only, 10 recommended)
MicroOLED oled(PIN_RESET, PIN_DC, PIN_CS);
//MicroOLED oled(PIN_RESET, DC_JUMPER); // Example I2C declaration


IntervalTimer myTimer; //Interrupt for Teensy

//**Copy to make a new timer******************//  
//TimerClass32 usTimerA( 20000 ); //20 ms

//**Current list of timers********************//
TimerClass32 debugTimer( 150000 ); //1 seconds
TimerClass32 serialSendTimer( 20000 ); //0.01 seconds
TimerClass32 oledSendTimer( 2000 ); //0.01 seconds
TimerClass32 rxCheckTimer( 200 );
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
char rxPacket_ptr;

void setup()
{
  Serial.begin(115200);
  
  Serial1.begin(115200);
  
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
  
   oled.begin();
   oled.clear(ALL);  // Clear the display's memory (gets rid of artifacts)
  oled.display();  
  delay(3000);
//   oled.clear(page);  // Clear the display's memory (gets rid of artifacts)
//   oled.display();
  delay(500);
  oled.setFontType(0);  // Set the text to small (10 columns, 6 rows worth of characters).
	oled.clear(PAGE);
	oled.print("TX Itvl:\n");
	oled.print("\nTX Dura:\n");
	oled.setCursor(26,40);
	oled.print("Peak:");
	oled.print(peakHold);

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
		oledSendTimer.update(usTicks);
		rxCheckTimer.update(usTicks);
		//Done?  Lock it back up
		usTicksLocked = 1;
	}  //The ISR will unlock.
	//**Copy to make a new timer******************//  
	//if(usTimerA.flagStatus() == PENDING)
	//{
	//	//User code
	//}
	//**Check incomming data timer****************//  
	if(rxCheckTimer.flagStatus() == PENDING)
	{
		debugLastTime[4] = debugStartTime[4];
		debugStartTime[4] = usTicks;
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
					rxPacket[i] = 0;
				}
				//write the start char
				rxPacket[0] = START_SYMBOL;
				//reset the pointer
				rxPacket_ptr = 1;
			}
			else if( ( rxPacket_ptr < PACKET_LENGTH ) && (rxPacket_ptr > 0) )//check for room in the packet, and that the start char has been seen
			{
				//put the char in the packet
				rxPacket[rxPacket_ptr] = lastchar;
				//advance the pointer
				rxPacket_ptr++;
				//turn on LED
			}
			else
			{
				//Just overwrite to the last position
				rxPacket[PACKET_LENGTH - 1] = lastchar;
			}
		}
		if((packetPending == 1) && ((rxPacket_ptr == PACKET_LENGTH) && ((rxPacket[PACKET_LENGTH - 1] == 0x0A) || (rxPacket[PACKET_LENGTH - 1] == 0x0D))) )
		{
			digitalWrite(13, digitalRead(13)^1);
			//check for new data
			packetPending = 0;

			uint8_t lastPacketNumber = char2hex(rxPacket[1]);
			if(lastPacketNumber < 3)
			{
				remoteNumbers[2*lastPacketNumber] = (uint16_t)char2hex(rxPacket[10]) | ((uint16_t)char2hex(rxPacket[9]) << 4) | ((uint16_t)char2hex(rxPacket[8]) << 8) | ((uint16_t)char2hex(rxPacket[7]) << 12);
				remoteNumbers[2*lastPacketNumber + 1] = (uint16_t)char2hex(rxPacket[14]) | ((uint16_t)char2hex(rxPacket[13]) << 4) | ((uint16_t)char2hex(rxPacket[12]) << 8) | ((uint16_t)char2hex(rxPacket[11]) << 12);
			}
			if(lastPacketNumber == 10)
			{
				remoteNumbers[9] = (uint16_t)char2hex(rxPacket[10]) | ((uint16_t)char2hex(rxPacket[9]) << 4) | ((uint16_t)char2hex(rxPacket[8]) << 8) | ((uint16_t)char2hex(rxPacket[7]) << 12);
				remoteNumbers[10] = (uint16_t)char2hex(rxPacket[14]) | ((uint16_t)char2hex(rxPacket[13]) << 4) | ((uint16_t)char2hex(rxPacket[12]) << 8) | ((uint16_t)char2hex(rxPacket[11]) << 12);
			}
		}

		debugStopTime[4] = usTicks;
	}
	//**Display clock*****************************//  
	if(oledSendTimer.flagStatus() == PENDING)
	{
		debugLastTime[3] = debugStartTime[3];
		debugStartTime[3] = usTicks;
		
		displayIncrement++;
		if(displayIncrement > 5) displayIncrement = 0;

		oled.display(displayIncrement); // Draw to the screen

		debugStopTime[3] = usTicks;
	}
	if(debugTimer.flagStatus() == PENDING)
	{
		debugLastTime[1] = debugStartTime[1];
		debugStartTime[1] = usTicks;
		//User code
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
		
//		oled.display();
		if((lastY2 > 0xA0)&&(lastB2))
		{
			displayPage++;
			if(displayPage >= DEBUG_TIME_SLOTS)displayPage = 0;
			oled.clear(PAGE);
			oled.setCursor(0,0);
			switch(displayPage)
			{
				case 0:
					oled.print("TX Itvl:\n");
					oled.print("\nTX Dura:\n");
				break;
				case 1:
					oled.print("DBG Itvl:\n");
					oled.print("\nDBG Dura:\n");
				break;
				case 2:
					oled.print("oled Itvl:");
					oled.print("\noled Dura:");
				break;
				case 3:
					oled.print("TX AFW:\n");
					oled.print("\nRX AVL:\n");
				break;
				case 4:
					oled.print("RX Itvl:\n");
					oled.print("\nRX Dura:\n");
				break;
				case 5:
					oled.print("R0 Intv:\n");
					oled.print("\nR0 Dura:\n");
				break;
				case 6:
					oled.print("R1 Itvl:\n");
					oled.print("\nR1 Dura:\n");
				break;
				case 7:
					oled.print("R2 Itvl:\n");
					oled.print("\nR2 Dura:\n");
				break;
				case 8:
					oled.print("FSAFE:\n");
					oled.print("\nI2C FLT:\n");
				break;
				default:
					oled.print("No pg: ");
					oled.print(displayPage);
				break;
			}
			oled.setCursor(26,40);
			oled.print("Peak:");
			oled.print(peakHold);

		}
		if((lastY2 < 0x60)&&(lastB2))
		{
			peakHold ^= 0x01;
			for( int i = 0; i < DEBUG_TIME_SLOTS; i++)
			{
				peakValues[0][i] = 0;
				peakValues[1][i] = 0;
			}
			oled.setCursor(26,40);
			oled.print("Peak:");
			oled.print(peakHold);
		}
		uint32_t tempValue = 0;
		oled.setCursor(0,8);
		oled.print("         ");
		oled.setCursor(0,24);
		oled.print("         ");
		switch(displayPage)
		{
			case 0:
				oled.setCursor(0,8);
				tempValue = debugStartTime[0] - debugLastTime[0];
				if(peakHold)
				{
					if(tempValue > peakValues[0][0])
					{
						peakValues[0][0] = tempValue;
						oled.print(tempValue);
					}
					else
					{
						oled.print(peakValues[0][0]);
					}
				}
				else
				{
					oled.print(tempValue);
				}
				oled.setCursor(0,24);
				tempValue = debugStopTime[0] - debugStartTime[0];
				if(peakHold)
				{
					if(tempValue > peakValues[1][0])
					{
						peakValues[1][0] = tempValue;
						oled.print(tempValue);
					}
					else
					{
						oled.print(peakValues[1][0]);
					}
				}
				else
				{
					oled.print(tempValue);
				}
				break;
			case 1:
				oled.setCursor(0,8);
				tempValue = debugStartTime[2] - debugLastTime[2];
				if(peakHold)
				{
					if(tempValue > peakValues[0][2])
					{
						peakValues[0][2] = tempValue;
						oled.print(tempValue);
					}
					else
					{
						oled.print(peakValues[0][2]);
					}
				}
				else
				{
					oled.print(tempValue);
				}
				oled.setCursor(0,24);
				tempValue = debugStopTime[2] - debugStartTime[2];
				if(peakHold)
				{
					if(tempValue > peakValues[1][2])
					{
						peakValues[1][2] = tempValue;
						oled.print(tempValue);
					}
					else
					{
						oled.print(peakValues[1][2]);
					}
				}
				else
				{
					oled.print(tempValue);
				}
			break;
			case 2:
				oled.setCursor(0,8);
				tempValue = debugStartTime[3] - debugLastTime[3];
				if(peakHold)
				{
					if(tempValue > peakValues[0][3])
					{
						peakValues[0][3] = tempValue;
						oled.print(tempValue);
					}
					else
					{
						oled.print(peakValues[0][3]);
					}

				}
				else
				{
					oled.print(tempValue);
				}
				oled.setCursor(0,24);
				tempValue = debugStopTime[3] - debugStartTime[3];
				if(peakHold)
				{
					if(tempValue > peakValues[1][3])
					{
						peakValues[1][3] = tempValue;
						oled.print(tempValue);
					}
					else
					{
						oled.print(peakValues[1][3]);
					}
				}
				else
				{
					oled.print(tempValue);
				}
			break;
			case 3:
				oled.setCursor(0,8);
				oled.print(Serial1.availableForWrite());
				oled.setCursor(0,24);
				oled.print(Serial1.available());
			break;
			case 4:
				oled.setCursor(0,8);
				tempValue = debugStartTime[4] - debugLastTime[4];
				if(peakHold)
				{
					if(tempValue > peakValues[0][4])
					{
						peakValues[0][4] = tempValue;
						oled.print(tempValue);
					}
					else
					{
						oled.print(peakValues[0][4]);
					}

				}
				else
				{
					oled.print(tempValue);
				}
				oled.setCursor(0,24);
				tempValue = debugStopTime[4] - debugStartTime[4];
				if(peakHold)
				{
					if(tempValue > peakValues[1][4])
					{
						peakValues[1][4] = tempValue;
						oled.print(tempValue);
					}
					else
					{
						oled.print(peakValues[1][4]);
					}
				}
				else
				{
					oled.print(tempValue);
				}
			break;
			case 5:
				oled.setCursor(0,8);
				tempValue = remoteNumbers[0];
				if(peakHold)
				{
					if(tempValue > peakValues[0][4])
					{
						peakValues[0][4] = tempValue;
						oled.print(tempValue);
					}
					else
					{
						oled.print(peakValues[0][4]);
					}

				}
				else
				{
					oled.print(tempValue);
				}
				oled.setCursor(0,24);
				tempValue = remoteNumbers[1];
				if(peakHold)
				{
					if(tempValue > peakValues[1][4])
					{
						peakValues[1][4] = tempValue;
						oled.print(tempValue);
					}
					else
					{
						oled.print(peakValues[1][4]);
					}
				}
				else
				{
					oled.print(tempValue);
				}
			break;
			case 6:
				oled.setCursor(0,8);
				tempValue = remoteNumbers[2];
				if(peakHold)
				{
					if(tempValue > peakValues[0][4])
					{
						peakValues[0][4] = tempValue;
						oled.print(tempValue);
					}
					else
					{
						oled.print(peakValues[0][4]);
					}

				}
				else
				{
					oled.print(tempValue);
				}
				oled.setCursor(0,24);
				tempValue = remoteNumbers[3];
				if(peakHold)
				{
					if(tempValue > peakValues[1][4])
					{
						peakValues[1][4] = tempValue;
						oled.print(tempValue);
					}
					else
					{
						oled.print(peakValues[1][4]);
					}
				}
				else
				{
					oled.print(tempValue);
				}
			break;
			case 7:
				oled.setCursor(0,8);
				tempValue = remoteNumbers[4];
				if(peakHold)
				{
					if(tempValue > peakValues[0][4])
					{
						peakValues[0][4] = tempValue;
						oled.print(tempValue);
					}
					else
					{
						oled.print(peakValues[0][4]);
					}

				}
				else
				{
					oled.print(tempValue);
				}
				oled.setCursor(0,24);
				tempValue = remoteNumbers[5];
				if(peakHold)
				{
					if(tempValue > peakValues[1][4])
					{
						peakValues[1][4] = tempValue;
						oled.print(tempValue);
					}
					else
					{
						oled.print(peakValues[1][4]);
					}
				}
				else
				{
					oled.print(tempValue);
				}
			break;
			case 8:
				oled.setCursor(0,8);
				tempValue = remoteNumbers[9];
				if(peakHold)
				{
					if(tempValue > peakValues[0][4])
					{
						peakValues[0][4] = tempValue;
						oled.print(tempValue);
					}
					else
					{
						oled.print(peakValues[0][4]);
					}

				}
				else
				{
					oled.print(tempValue);
				}
				oled.setCursor(0,24);
				tempValue = remoteNumbers[10];
				if(peakHold)
				{
					if(tempValue > peakValues[1][4])
					{
						peakValues[1][4] = tempValue;
						oled.print(tempValue);
					}
					else
					{
						oled.print(peakValues[1][4]);
					}
				}
				else
				{
					oled.print(tempValue);
				}
			break;
			default:
			break;
		}

		debugStopTime[1] = usTicks;
		//copy to usable section
		debugLastTime[2] = debugLastTime[1];
		debugStartTime[2] = debugStartTime[1];
		debugStopTime[2] = debugStopTime[1];
	}
	if(serialSendTimer.flagStatus() == PENDING)
	{
	//Check for new data
		debugLastTime[0] = debugStartTime[0];
		debugStartTime[0] = usTicks;

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
			//Serial1.write(0x0A);
			packetNumber++;
		}
		debugStopTime[0] = usTicks;
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


