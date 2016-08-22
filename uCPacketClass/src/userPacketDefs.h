//This file specifies the packets used by the application.
//These are structures the must be transmitted and recieved
//As a stream of bytes somehow.
#ifndef USERPACKETDEFS_H
#define USERPACKETDEFS_H
struct robotClientPacket
{
	uint8_t packetStatus;
	uint8_t gamepadButtons;
	uint32_t testHex;
};

struct robotHostPacket
{
	uint8_t packetStatus;
	
};
#endif