//**RobotMotion*******************************//
#include "RobotMotion.h"
//#include "PanelComponents.h"
#include "Panel.h"
#include "Arduino.h"
#include "flagMessaging.h"

#define KNOBDELTADETECT 5  //5 units of 255 for knob change detect
#define ACCEL_RATE 0.12
#define DECEL_RATE 0.02
#define BK_HOLDOFFMS 250
#define FW_HOLDOFFMS 250

RobotMotion::RobotMotion( void )
{
	//Controls
	state = PInit;
	velocity = 0;
	velocityShadow = 0;
	direction = 0;
	lastFB = 1;
	frontSwap = 0;
	bkupHoldTimeKeeper.mClear();
	frwdHoldTimeKeeper.mClear();
	
}

void RobotMotion::reset( void )
{
	//Set explicit states
	
	update();
	
}

//---------------------------------------------------------------------------//
//
//  To process the machine,
//    take the inputs from the system
//    process human interaction hard-codes
//    process the state machine
//    clean up and post output data
//
//---------------------------------------------------------------------------//
void RobotMotion::processMachine( void )
{
	
	
	//Do main machine
	tickStateMachine();
	
	update();
}

void RobotMotion::tickStateMachine()
{
	//***** PROCESS THE LOGIC *****//
	if( b2Button.serviceRisingEdge() )
	{
		frontSwap ^= 0x01;
	}

	//No state machine, it would go here
	direction = ((float)lastX1 - 0x80) / 0x80;
//	if((direction < 0.13)&&(direction > -0.13))
//	{
//		direction = 0;
//	}
	
	velocity = ((float)lastY1 - 0x80) / 0x80;
//	if((velocity < 0.13)&&(velocity > -0.13))
//	{
//		velocity = 0;
//	}
}

void RobotMotion::timersMIncrement( uint8_t inputValue )
{
	b1Button.buttonDebounceTimeKeeper.mIncrement(inputValue);
	b2Button.buttonDebounceTimeKeeper.mIncrement(inputValue);
	bkupHoldTimeKeeper.mIncrement(inputValue);

}