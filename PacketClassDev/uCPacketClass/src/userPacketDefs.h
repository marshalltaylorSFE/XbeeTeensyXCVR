//This file specifies the packets used by the application.
//These are structures the must be transmitted and recieved
//As a stream of bytes somehow.
#ifndef USERPACKETDEFS_H
#define USERPACKETDEFS_H
struct robotClientPacket
{
	uint8_t packetStatus;
	uint8_t gamepadButtons;
	int16_t test;
};

struct robotHostPacket
{
	uint8_t packetStatus;
	
};
#endif