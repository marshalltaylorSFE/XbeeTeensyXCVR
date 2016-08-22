# XbeeTeensyXCVR

This arduino library is used to packetize structures and operate the serial port.

## Library object

### Construction
Construct with two arguments, pass the serial port name (may need to cast as type HardwareSerial), then the number of bytes to use for the rx buffer.  This should be `sizeof` yourStruct * 2 + 3, or number of nybbles plus a few bytes for control chars.

`uCPacketUART dataLinkHandler((HardwareSerial*)Serial1, 64); //64 byte buffers`

### Functions

* `void burstReadInputBuffer( void )`

This will read a number of bytes from the input buffer.  Default is three.  This wil **only read up to the next delimiter**, and thus locks up the bus if the received packets aren't dealt with.

TODO:  Pass number of bytes to burst.

* `void abandonRxPacket( void )`

Get rid of the received packet so we can get a fresh one.

* `uint16_t available( void )`

Returns size of packet available.  If this returns `0`, no packet available

* `void getPacket( uint8_t *, uint16_t )`

Pass the structure you wish to receive into, and the sizeof it.  getPacket will write into your structure but only if it is the same size (no other matching occurs between received data and requested structure type) 

* `uint8_t sendPacket( uint8_t *, uint16_t )`

Pass the structre you wish to send, and the sizeof it.  This puts the whole thing into the output buffer, so this shouldn't be used for large structures.

### Notes:

Both sending and receiving parties should be using the same structure definitions.  Create a `.h` file that is consisitent between the two.  Or, place it in the /`src` folder to be sure.  Then, create local globals or something and create code to send and receive to them periodically.  Both ends can then use the other's transmitted object as if in the same scope.

### Change log
* 8/15/2016:
  * This should be the final library development branch.
  * Created library template.  Copy to libraries folder and remember to copy back if pushing changes.
* 8/17/2016:
  * The client currently compiles with the library, but does not do much else.
* 8/17/2016:
  * The client now sends its data as asciized nybles.
  * The start symbol and tail are now added by UCPacketUART and not exposed to the user
  * The host currently comppiles and reports its packet dumps.
* 8/18/2016:
  * Reading parser is now written and compiles but has not been tested on actual hardware
  * Neither example has been tested.
* 8/21/2016:
  * Tested examples and created working example that uses 
This library has two examples, one as a serial host and the other, the client.  Think of it as the client sends raw control input to the host, which is a continous being.

