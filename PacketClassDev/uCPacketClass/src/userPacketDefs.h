//This file specifies the packets used by the application.
//These are structures the must be transmitted and recieved
//As a stream of bytes somehow.
#ifndef USERPACKETDEFS_H
#define USERPACKETDEFS_H
struct robotClientPacket
{
	char startSymbol;
	uint8_t packetNumber;
	uint8_t status;
	uint8_t gamepadButtons;
	char tail[2];
};

struct robotHostPacket
{
	char startSymbol;
	uint8_t packetNumber;
	uint8_t status;
	char tail[2];
	
};
#endif