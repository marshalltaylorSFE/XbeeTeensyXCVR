//License information:
//Haha!  You use this code.  Use it!
#ifndef UCPACKETCLASS_H
#define UCPACKETCLASS_H
class uCPacketUART
{
public:
	uCPacketUART( HardwareSerial *, uint16_t ); //( uint16_t bytesAllocated )
	~uCPacketUART();
//	void initialize( void );
//	void write( uint8_t, uint8_t ); //address, value.
//	uint8_t read( uint8_t );//address
	void flushInputBuffer( void );
	uint16_t available( void ); //Returns size of packet available
	void getPacket( uint8_t *, uint16_t ); //give address and size
	void sendPacket( uint8_t *, uint16_t );// Write entire packet to port
//	void queuePacket( ptr, size );
//  uint8_t sendQueueBurst( size ); //Returns remaining bytes or at least empty state

	uint8_t gamepadButtons;
	uint16_t bytesAllocated; //Indication of how much ram has been allocated
	uint8_t * rxBuffer;
	uint16_t rxBufferIndex;
	private:
	HardwareSerial * linkSerial;
};
#endif