//Header
#ifndef RobotMotion_H_INCLUDED
#define RobotMotion_H_INCLUDED


#include "stdint.h"
#include "timeKeeper.h"
#include "Panel.h"
#include "flagMessaging.h"

enum PStates
{
	PInit,
	PIdle,
	PSleepy,
	PForward,
	PStop,
	PBackward,

};

class RobotMotion : public Panel
{
public:
	RobotMotion( void );
	
	void reset( void );
	
	//State machine stuff  
	void processMachine( void );
	void tickStateMachine( void );

	void timersMIncrement( uint8_t );
	
//	//Flags coming in from the system
//
//	uint8_t rxLedFlag;
//	uint8_t txLedFlag;
//	
//	//Internal - and going out to the system - Flags
//	//MessagingFlag clearSongFlag;
//	
//	//  ..and data.
//	uint8_t displayMode;
//	uint8_t selectorPosition;
//	uint8_t leftSelectorPosition;
//	uint8_t rightSelectorPosition;
//	uint8_t leftKnobPosition;
//	uint8_t rightKnobPosition;
	
	uint16_t lastX1;
	uint16_t lastY1;
	uint16_t lastX2;
	uint16_t lastY2;
	
	uint8_t lastB1;
	uint8_t lastB2;
	

//private:
	//Internal Flags
	//  ..and data
	//MessagingFlag newSelector;
	//MessagingFlag newLeftKnob;
	//MessagingFlag newRightKnob;
	TimeKeeper bkupHoldTimeKeeper;
	TimeKeeper frwdHoldTimeKeeper;

	volatile float velocity;
	volatile float velocityShadow;
	volatile float direction;
	volatile float lastFB;
	uint8_t frontSwap;
	//State machine stuff  
	PStates state;
	
};

#endif