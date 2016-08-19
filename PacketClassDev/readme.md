Library creation projects
------------

8/15/2016:
This should be the final library development branch.
Created library template.  Copy to libraries folder and remember to copy back if pushing changes.
8/17/2016:
The client currently compiles with the library, but does not do much else.
8/17/2016:
The client now sends its data as asciized nybles.
The start symbol and tail are now added by UCPacketUART and not exposed to the user
The host currently comppiles and reports its packet dumps.
8/18/2016:
Reading parser is now written and compiles but has not been tested on actual hardware
Neither example has been tested.

This library has two examples, one as a serial host and the other, the client.  Think of it as the client sends raw control input to the host, which is a continous being.

