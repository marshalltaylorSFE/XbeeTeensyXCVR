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

//**Timers and stuff**************************//
#include "timerModule32.h"
#include "timeKeeper.h"

//Hardware locations
uint8_t wsPin = 6;
uint8_t debugPin = 13;

uint32_t maxTimer = 60000000;
uint32_t maxInterval = 2000000;

IntervalTimer myTimer; //ISR for Teensy

//Other timers
TimerClass32 remoteInputTimer( 1000 );
TimerClass32 debugTimer(1000000);

//Serial packet defines
#define PACKET_LENGTH 24
#define START_SYMBOL '~'

char lastchar;
char packet[PACKET_LENGTH];
char lastPacket[PACKET_LENGTH];
char packetPending = 0;

uint8_t packet_ptr;

//--tick variable for interrupt driven timer1
elapsedMicros usTickInput = 0;
uint32_t usTicks = 0;
uint8_t usTicksMutex = 1; //start locked out

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

void setup()
{
	Serial.begin(9600); // Initialize Serial Monitor USB
	Serial1.begin(115200); // Initialize hardware serial port, pins 0/1
	Serial2.begin(9600);
	pinMode(debugPin, OUTPUT);
	digitalWrite(debugPin, 0);
	
	delay(1000);
	
	// Send a welcome message to the serial monitor:
	Serial.println("Host Started");
	
}

void loop()
{
	//**Update the timers*************************//  
	remoteInputTimer.update(usTicks);
	debugTimer.update(usTicks);

	//**Read changes from the controller**********//  
	if(remoteInputTimer.flagStatus() == PENDING)
	{
		if (Serial2.available())
		{
		lastchar = Serial2.read();
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
		else
		if( ( packet_ptr < PACKET_LENGTH ) && (packet_ptr > 0) )//check for room in the packet, and that the start char has been seen
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
		
		//if the packet is full and the last char is LF or CR, *do something here*
		if((packetPending == 1) && ((packet_ptr == PACKET_LENGTH) && ((packet[PACKET_LENGTH - 1] == 0x0A) || (packet[PACKET_LENGTH - 1] == 0x0D))) )
		{
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
			lastX1 = char2hex(packet[4]) | (char2hex(packet[3]) << 4);
			lastY1 = char2hex(packet[6]) | (char2hex(packet[5]) << 4);
			lastX2 = char2hex(packet[8]) | (char2hex(packet[7]) << 4);
			lastY2 = char2hex(packet[10]) | (char2hex(packet[9]) << 4);
			lastB1 = char2hex(packet[11]) & 0x01;
			lastB2 = char2hex(packet[12]) & 0x01;
			
			//Calculate polar coordinates 			
			//  Left stick
			int16_t lastX1i = (int16_t)lastX1 - 0x7D;  //Needs to be well centered by hardcoded value here
			int16_t lastY1i = (int16_t)lastY1 - 0x82;  //Needs to be well centered by hardcoded value here
			cart2polar((float)lastX1i / 0x90,(float)lastY1i / 0x90,lastR1,lastT1);
			
			//  Right stick
			int16_t lastX2i = (int16_t)lastX2 - 0x83;  //Needs to be well centered by hardcoded value here
			int16_t lastY2i = (int16_t)lastY2 - 0x7A;  //Needs to be well centered by hardcoded value here
			cart2polar((float)lastX2i / 0x90,(float)lastY2i / 0x90,lastR2,lastT2);

		}
	}

	//**Debug timer*******************************//  
	if(debugTimer.flagStatus() == PENDING)
	{
		digitalWrite( debugPin, digitalRead(debugPin) ^ 1 );
		
		Serial.print("Reading lastX1: 0x");
		Serial.print(lastX1, HEX);
		Serial.println("");
		Serial.print("Reading lastY1: 0x");
		Serial.print(lastY1, HEX);
		Serial.println("");
		Serial.print("Reading lastX2: 0x");
		Serial.print(lastX2, HEX);
		Serial.println("");
		Serial.print("Reading lastY2: 0x");
		Serial.print(lastY2, HEX);
		Serial.println("");
		Serial.print("Reading lastB1: 0x");
		Serial.print(lastB1, HEX);
		Serial.println("");
		Serial.print("Reading lastB2: 0x");
		Serial.print(lastB2, HEX);
		Serial.println("");

		
		
		Serial.print("lastR1: ");
		Serial.print(lastR1);
		Serial.println("");
		Serial.print("lastT1: ");
		Serial.print(lastT1);
		Serial.println("");		

		Serial.print("lastDeg1: ");
		Serial.print( (lastT1 / 6.25) * 360 );
		Serial.println("");		

		Serial.print("lastR2: ");
		Serial.print(lastR2);
		Serial.println("");
		Serial.print("lastT2: ");
		Serial.print(lastT2);
		Serial.println("");		

		Serial.print("lastDeg2: ");
		Serial.print( (lastT2 / 6.25) * 360 );
		Serial.println("");		

		Serial.println("");

	}

	
	//Do the ISR with the teensy built-in timer --
	//  Update the usTicks from usTickInput, alwasy keepin
	if(usTickInput != usTicks)
	{
		if(usTickInput >= ( maxTimer + maxInterval ))
		{
		usTickInput = usTickInput - maxTimer;
		
		}
		usTicks = usTickInput;
		usTicksMutex = 0;  //unlock
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