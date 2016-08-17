//License information:
//Haha!  You use this code.  Use it!

#include "Arduino.h"
#include "uCPacketClass.h"

uCPacketUART::uCPacketUART( HardwareSerial * inSerial, uint16_t requestedBufferSize )
{
	gamepadButtons = 0;
	rxBufferIndex = 0;
	
	//Save the size
	bytesAllocated = requestedBufferSize;
	rxBuffer = new uint8_t[bytesAllocated];
	linkSerial = inSerial;
};

uCPacketUART::~uCPacketUART( void )
{
	//Trash it
	delete[] rxBuffer;
	bytesAllocated = 0;
};

void uCPacketUART::flushInputBuffer( void )
{
	;
};

uint16_t uCPacketUART::available( void )
{
	return 0;
};

void uCPacketUART::getPacket( uint8_t * packetVar, uint16_t sizeVar )
{
	;
};

void uCPacketUART::sendPacket( uint8_t * packetVar, uint16_t sizeVar )
{
	;
};

