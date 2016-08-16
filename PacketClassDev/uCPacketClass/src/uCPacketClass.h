//License information:
//Haha!  You use this code.  Use it!

class uCPacket
{
public:
	uCPacket( void );
	void initialize( void );
	void write( uint8_t, uint8_t ); //address, value.
	uint8_t read( uint8_t );//address
	void send( transmit_t, trasmitSize );// Ship the packet OUT THE PORT!
	
	uint8_t nintendoButtons;
	
	
};