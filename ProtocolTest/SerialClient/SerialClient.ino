//HOW TO OPERATE
//  Make TimerClass objects for each thing that needs periodic service
//  pass the interval of the period in ticks
//
//  Set maxInterval to rollover rate
//  Set maxTimer to the max foreseen interval of any timer.
//  maxTimer + maxInterval = max countable value.

//NOTICE:
//  The timerModule32 only works on teensy / fast processors.  It works the same
//  but keeps track of everything in us counts.


//Not used by this sketch but dependant on one 
#include "Wire.h"

//Globals
uint32_t maxTimer = 60000000;
uint32_t maxInterval = 2000000;

uint16_t lastX1;
uint16_t lastY1;
uint16_t lastX2;
uint16_t lastY2;

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
TimerClass32 debugTimer( 1000000 ); //1 second

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
void setup()
{
  Serial.begin(115200);
  
  pinMode(LEDPIN, OUTPUT);
  pinMode(INPUTPINB1, INPUT_PULLUP);
  pinMode(INPUTPINB2, INPUT_PULLUP);

  // initialize IntervalTimer
  myTimer.begin(serviceUS, 1);  // serviceMS to run every 0.000001 seconds

}

void loop()
{
	//Update the timers, but only once per interrupt
	if( usTicksLocked == 0 )
	{
		//**Copy to make a new timer******************//  
		//msTimerA.update(usTicks);
		debugTimer.update(usTicks);

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
		
		lastX1 = (analogRead(INPUTPINX1)) >> 6;
		lastY1 = (0x3FF - analogRead(INPUTPINY1)) >> 6;
		lastX2 = (analogRead(INPUTPINX2)) >> 6;
		lastY2 = (0x3FF - analogRead(INPUTPINY2)) >> 6;
		lastB1 = 0x1 ^ digitalRead(INPUTPINB1);
		lastB2 = 0x1 ^ digitalRead(INPUTPINB2);

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
		Serial.println("");
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


