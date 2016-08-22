#include "Panel.h"
#include "Arduino.h"
extern char lastPacket[];

//Pins are now Position
#define b1Pos 10
#define b2Pos 11

//---Button------------------------------------------------------
RobotButton::RobotButton( void )
{
	beingHeld = 0;
	bank = 0;
	risingEdgeFlag = 0;
	fallingEdgeFlag = 0;
	holdRisingEdgeFlag = 0;
	holdFallingEdgeFlag = 0; 
	
}

void RobotButton::init( uint8_t pinInput )
{
	init( pinInput, 0 );
}

void RobotButton::init( uint8_t pinInput, uint8_t bankInput )
{
	bank = bankInput;
	pinNumber = pinInput;
	if( bank == 0 )
	{
		update();

	}
	else
	{
		//Do bank related initialization here
	}
}

//This is the intended operation:
// Button is pressed. If it has been long enough since last movement:
//   update newData
//   clear timer
// If This has already been done, check for button held.  If so,
//   update newData
//   clear timer
void RobotButton::update( void )
{
	uint8_t freshData;
	if( bank == 0 )
	{
		freshData = (lastPacket[ pinNumber + 1 ] - 48) ^ 0x01;
	}
	else
	{
		freshData = cache ^ 0x01;  //Take externally provided data
	}
	
	buttonState_t nextState = state;
	switch( state )
	{
	case BUTTONOFF: //Last state was BUTTONOFF
		if(( freshData == 1 ) && ( buttonDebounceTimeKeeper.mGet() > 50 ))
		{
			//Start the timer
			buttonDebounceTimeKeeper.mClear();
			nextState = BUTTONON;
			risingEdgeFlag = 1;
		}
		else
		{
			nextState = BUTTONOFF;
		}
		break;
	case BUTTONON: //Last state was BUTTONON
		if( freshData == 1 )
		{
			if( buttonDebounceTimeKeeper.mGet() > 1000 )
			{
				nextState = BUTTONHOLD;
				holdRisingEdgeFlag = 1;
			}
		}
		//No break;, do buttonhold's state too
	case BUTTONHOLD: //In the process of holding
		if( freshData == 0 )
		{
			buttonDebounceTimeKeeper.mClear();
			nextState = BUTTONOFF;
		}
		break;
	default:
		break;
	}
	
	state = nextState;

}

buttonState_t RobotButton::getState( void )
{
	return state;
}

uint8_t RobotButton::serviceRisingEdge( void )
{
	uint8_t returnVar = 0;
	if( risingEdgeFlag == 1 )
	{
		risingEdgeFlag = 0;
		returnVar = 1;
	}
	
	return returnVar;
}

uint8_t RobotButton::serviceFallingEdge( void )
{
	uint8_t returnVar = 0;
	if( fallingEdgeFlag == 1 )
	{
		fallingEdgeFlag = 0;
		returnVar = 1;
	}
	
	return returnVar;
}

uint8_t RobotButton::serviceHoldRisingEdge( void )
{
	uint8_t returnVar = 0;
	if( holdRisingEdgeFlag == 1 )
	{
		holdRisingEdgeFlag = 0;
		returnVar = 1;
	}
	
	return returnVar;
}

uint8_t RobotButton::serviceHoldFallingEdge( void )
{
	uint8_t returnVar = 0;
	if( holdFallingEdgeFlag == 1 )
	{
		holdFallingEdgeFlag = 0;
		returnVar = 1;
	}
	
	return returnVar;
}

void RobotButton::setBank( uint8_t newBank )
{
	bank = newBank;

}
//...
//End button

Panel::Panel( void )
{
	flasherState = 0;
	fastFlasherState = 0;
}

void Panel::init( void )
{
	b1Button.init(b1Pos);
	b2Button.init(b2Pos);
	
	flasherState = 0;
	fastFlasherState = 0;
	
}

void Panel::update( void )
{
	b1Button.update();
	b2Button.update();

}

void Panel::toggleFlasherState( void )
{
	flasherState ^= 0x01;
}

void Panel::toggleFastFlasherState( void )
{
	fastFlasherState ^= 0x01;
}