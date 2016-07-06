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

//**LED control*******************************//
#include <Adafruit_NeoPixel.h>
//**Timers and stuff**************************//
#include "timerModule32.h"
#include "timeKeeper.h"

//Hardware defines
uint8_t wsPin = 6;
uint8_t debugPin = 13;

uint32_t maxTimer = 60000000;
uint32_t maxInterval = 2000000;

IntervalTimer myTimer; //ISR for Teensy

Adafruit_NeoPixel indicators = Adafruit_NeoPixel(2, wsPin, NEO_GRB + NEO_KHZ800);

//Serial packet defines
#define PACKET_LENGTH 24
#define START_SYMBOL '~'

char lastchar;
char packet[PACKET_LENGTH];
char lastPacket[PACKET_LENGTH];
char packetPending = 0;

char packet_ptr;

//Control system defines
uint8_t frontSwap = 0;

uint8_t ledToggleState = 0;
uint8_t ledToggleFastState = 0;

TimerClass32 remoteInputTimer( 1000 );
TimerClass32 robotMotionTimer(10000);
TimerClass32 ledToggleTimer( 333000 );
TimerClass32 ledToggleFastTimer( 100000 );
TimerClass32 motorUpdateTimer( 4000 );
TimerClass32 debounceTimer(5000);
TimerClass32 debugTimer(1000000);

char tempSpeedCharL = '0';
char tempSpeedCharR = '0';
char tempSpeedDirL = 'R';
char tempSpeedDirR = 'R';

uint8_t LRWord = 0;

//--tick variable for interrupt driven timer1
elapsedMicros usTickInput = 0;
uint32_t usTicks = 0;
uint8_t usTicksMutex = 1; //start locked out

//--Robot state machine
#include "RobotMotion.h"
RobotMotion myRobot;

void setup()
{
	Serial.begin(9600); // Initialize Serial Monitor USB
	Serial1.begin(115200); // Initialize hardware serial port, pins 0/1
	Serial2.begin(9600);
	pinMode(debugPin, OUTPUT);
	digitalWrite(debugPin, 0);
	
	delay(1000);
	
	// Send a welcome message to the serial monitor:
	Serial.println("Started");
	indicators.begin();
	indicators.setPixelColor(0, 0xFF20AF);
	indicators.setPixelColor(1, 0xFF20AF);
	indicators.show(); // Initialize all pixels to 'off'
	
	//Ready the state machines
	myRobot.init();
	
}

void loop()
{
//**Copy to make a new timer******************//  
	remoteInputTimer.update(usTicks);
	ledToggleTimer.update(usTicks);
	ledToggleFastTimer.update(usTicks);
	robotMotionTimer.update(usTicks);
	motorUpdateTimer.update(usTicks);
	debounceTimer.update(usTicks);
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
			myRobot.lastX1 = char2hex(packet[4]) | (char2hex(packet[3]) << 4);
			myRobot.lastY1 = char2hex(packet[6]) | (char2hex(packet[5]) << 4);
			myRobot.lastX2 = char2hex(packet[8]) | (char2hex(packet[7]) << 4);
			myRobot.lastY2 = char2hex(packet[10]) | (char2hex(packet[9]) << 4);
			myRobot.lastB1 = char2hex(packet[11]) & 0x01;
			//Serial.println(myRobot.lastB1);
			myRobot.lastB2 = char2hex(packet[12]) & 0x01;
			//Serial.println(myRobot.lastB2);
		}
	}
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
		
		
		if( LRWord )
		{
			Serial1.print('1');
			Serial1.print(tempSpeedDirL);
			Serial1.print(tempSpeedCharL);
			Serial1.print('\r');
		}
		else
		{
			//Flip a channel at the output here
			//if(tempSpeedDirR == 'R')
			//{
			//	tempSpeedDirR = 'F';
			//}
			//else
			//{
			//	tempSpeedDirR = 'R';
			//}
			Serial1.print('2');
			Serial1.print(tempSpeedDirR);
			Serial1.print(tempSpeedCharR);
			Serial1.print('\r');
		}
		LRWord ^= 0x01;
	}		
	//**Process the panel and state machine***********//  
	if(robotMotionTimer.flagStatus() == PENDING)
	{
		//Provide inputs

		//Tick the machine
		myRobot.processMachine();
		
		//Serial.print("DIR: ");
		//Serial.println(myRobot.direction);
		
		//Deal with outputs
		//if( myRobot.velocity > 0.1 )
		//{
		//	setDrive((myRobot.velocity * 9), myRobot.direction);
		//}
		//else if( myRobot.velocity < -0.1 )
		//{
		//	setDrive((myRobot.velocity * -9), myRobot.direction);
		//}
		//else
		//{
		//	setDrive(0, myRobot.direction);
		//}
		setDrive((myRobot.velocity * 9), myRobot.direction);
		frontSwap = myRobot.frontSwap;

	}
	//**Debug timer*******************************//  
	//**Fast LED toggling of the panel class***********//  
	if(ledToggleFastTimer.flagStatus() == PENDING)
	{
		myRobot.toggleFastFlasherState();
		
	}

	//**LED toggling of the panel class***********//  
	if(ledToggleTimer.flagStatus() == PENDING)
	{
		myRobot.toggleFlasherState();
		
	}
	if(debugTimer.flagStatus() == PENDING)
	{
		digitalWrite( debugPin, digitalRead(debugPin) ^ 1 );
		//Serial.println(myRobot.leftButton.buttonDebounceTimeKeeper.mGet());
		//Serial.println(myRobot.b1Button.getState());
		//Serial.println(myRobot.b2Button.getState());
		//Serial.println(lastPacket[1]);
		//Serial.println(myRobot.velocity);
		//Serial.println(myRobot.direction);
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
		usTicksMutex = 0;  //unlock
	}
}

void setDrive( int32_t speed, float direction)
{
	tempSpeedCharL = '0';
	tempSpeedCharR = '0';
	tempSpeedDirL = 'R';
	tempSpeedDirR = 'R';
	

//	if( speed == 0 && direction == 1 )
//	{
//		//Serial.println("Right");
//		//full right
//		tempSpeedCharL += 9;
//		tempSpeedCharR += 9;
//		tempSpeedDirL = 'R';
//		tempSpeedDirR = 'F';
//	}
//	else if( speed == 0 && direction == -1 )
//	{
//		//Serial.println("Left");
//
//		//full right
//		tempSpeedCharL += 9;
//		tempSpeedCharR += 9;
//		tempSpeedDirL = 'F';
//		tempSpeedDirR = 'R';
//	}
//	else
	{
		if(frontSwap)
		{
			speed *= -1;
			direction *= -1;
		}
		if(speed < 0)
		{
			//flip
			//Serial.println("Neg speed");
			speed *= -1;
			tempSpeedDirL = 'F';
			tempSpeedDirR = 'F';
			direction *= 1;
		}
		//Serial.println("Normal");
		if( direction > 0.1) //right
		{
			//Serial.print("Right: ");
			tempSpeedCharR = speed+2;
			tempSpeedCharL = speed + direction * 9;
			//Serial.print(tempSpeedCharL);
			//Serial.print(" ");
			//Serial.println(tempSpeedCharR);
		}
		else if( direction < -0.1)
		{
			//Serial.print("Left: ");
			tempSpeedCharR = speed+(direction * -9);
			tempSpeedCharL = speed+2;
			//Serial.print(tempSpeedCharL);
			//Serial.print(" ");
			//Serial.println(tempSpeedCharR);
		}
		else if( speed > 0.1 )//stright
		{
			tempSpeedCharL = speed+2;
			tempSpeedCharR = speed+2;
		}
		else
		{
			tempSpeedCharL = 0;
			tempSpeedCharR = 0;
		}

		if(tempSpeedCharL > 9) tempSpeedCharL = 9;
		if(tempSpeedCharR > 9) tempSpeedCharR = 9;			
		
		tempSpeedCharL = hex2char(tempSpeedCharL);
		tempSpeedCharR = hex2char(tempSpeedCharR);
		
		Serial.print(tempSpeedCharL);
		Serial.print(" ");
		Serial.print(tempSpeedCharR);
		Serial.print(" ");
		Serial.print(tempSpeedDirL);
		Serial.print(" ");
		Serial.println(tempSpeedDirR);
	}
}