//License information:
//Haha!  You use this code.  Use it!

#include "Arduino.h"
#include "uCPacketClass.h"
#include "HOS_char.h"

uCPacketUART::uCPacketUART( HardwareSerial * inSerial, uint16_t requestedBufferSize )
{
	rxBufferIndex = 0;
	txBufferIndex = 0;
	
	txPacketInProgress = 0;
	
	//Configure packets
	startSymbol = '~';
	tail[0] = 0x0D;
	tail[1] = 0x0A;
	tail[2] = 0x00;
  
	//Save the size
	bytesAllocated = requestedBufferSize;
	rxBuffer = new uint8_t[bytesAllocated];
	txBuffer = new uint8_t[bytesAllocated];
	linkSerial = inSerial;
};

uCPacketUART::~uCPacketUART( void )
{
	//Trash it
	delete[] rxBuffer;
	delete[] txBuffer;
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

uint8_t uCPacketUART::sendPacket( uint8_t * packetVar, uint16_t sizeVar )
{
	if( txPacketInProgress )
	{
		return 1;
	}
	else
	{
		txBufferIndex = 0;
		txBuffer[txBufferIndex++] = startSymbol;
		for( int inputPacketPointer = 0; inputPacketPointer < sizeVar; inputPacketPointer++ )
		{
			txBuffer[txBufferIndex++] = hex2char((packetVar[inputPacketPointer] >> 4) & 0x0F);
			txBuffer[txBufferIndex++] = hex2char((packetVar[inputPacketPointer]) & 0x0F);
		}
		txBuffer[txBufferIndex++] = tail[0];
		txBuffer[txBufferIndex++] = tail[1];
		txBuffer[txBufferIndex++] = tail[2];

		for( int i = 0; txBuffer[i] != 0x00; i++ )
		{
			linkSerial->print((char)txBuffer[i]);
		}
		return 0;
	}
	
};

