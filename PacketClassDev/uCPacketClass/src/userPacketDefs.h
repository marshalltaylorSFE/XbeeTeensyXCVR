//This file specifies the packets used by the application.
//These are structures the must be transmitted and recieved
//As a stream of bytes somehow.

struct robotClientPacket
{
	char startSymbol;
	uint8_t packetNumber;
	uint8_t status;
	uint8_t nintendoButtons;
	char tail[2];
};

struct robotHostPacket
{
	char startSymbol;
	uint8_t packetNumber;
	uint8_t status;
	char tail[2];
	
};