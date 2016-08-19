#ifndef PANEL_H
#define PANEL_H

//#include <Arduino.h>
#include "timeKeeper.h"
#include "flagMessaging.h"

// The regular method is to #include "PanelComponents.h" and let
// it control the io.  Instead, the button section is manually
// imported and the update function modified.

//---Button------------------------------------------------------
enum buttonState_t
{
  BUTTONOFF = 0,
  BUTTONON = 1,
  BUTTONHOLD = 2,

};

class RobotButton
{
public:
	TimeKeeper buttonDebounceTimeKeeper;
	
	RobotButton( void );
	void update( void );
	void init( uint8_t );
	void init( uint8_t, uint8_t );
	buttonState_t getState( void );
	uint8_t serviceRisingEdge( void );
	uint8_t serviceFallingEdge( void );
	uint8_t serviceHoldRisingEdge( void );
	uint8_t serviceHoldFallingEdge( void );
	void setBank( uint8_t );
	buttonState_t state;
	uint8_t invert;
	uint8_t pinNumber;
	uint8_t cache;
protected:
private:
	uint8_t beingHeld;
	uint8_t bank;
	uint8_t risingEdgeFlag;
	uint8_t fallingEdgeFlag;
	uint8_t holdRisingEdgeFlag;
	uint8_t holdFallingEdgeFlag;
};

class Panel
{
public:
	Panel( void );
	void update( void );
	void init( void );
	void toggleFlasherState( void );
	void toggleFastFlasherState( void );

	RobotButton leftButton;
	RobotButton rightButton;
	RobotButton upButton;
	RobotButton downButton;
	RobotButton selectButton;
	RobotButton startButton;
	RobotButton aButton;
	RobotButton bButton;

private:
	volatile uint8_t flasherState;
	volatile uint8_t fastFlasherState;

};

#endif // PANEL_H


