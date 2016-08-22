# XbeeTeensyXCVR


This repository contains a library and example code for building XBee wireless links between two entities.

### Repository contents

* **Documentation** -- Mostly spreadsheets for keeping track of program data and calculations
* **ExampleProjects** -- Contains host and client sketches for actually driving a robot.
* **PacketTestingCode** -- Contains host and client sketches for development of the project.
* **PaddleDesign** -- Graphics used to create the controller shape

## Example projects
The examples run on either the Teensy 3.1 or LCs for now.

### Library requirements
These projects rely on [uCModules](https://github.com/marshalltaylorSFE/uCModules) to deal with time.  Install to libraries directory or manually add timerModule32.h, timerModule32.cpp, timeKeeper.h, and timeKeeper.cpp to the sketches where necessary.

#### Analog controller, 4 wheeled robot using teensy 3.1s

* Client sends (X,Y) data on both sticks, button data and calculates polar locally for the debug window
* Host receives (X,Y) data and calculates polar
* Host returns packet containing diagnostic information
* Client has a menu through the OLED that can show debug info
* Fail-safe mechanism implemented.  Turns off in about 1 second.

#### Digital controller, 2 wheeled 'hamburger bot'

##### Usage:

* SerialHostHamburgerBot - Teensy LC, XBee, Serial controlled motor driver v2.0, two WS2812s
* SerialClientControllerDigital8Button - Teensy LC, XBee, 8 buttons wired to GPIO

##### Notes:

* Uses uCPacketClass
* Controller (Client) packs button states into a byte which is in a structure.
* Robot (Host) interprets data in packet as IO pins through use of a modified PanelClass, a library that handles button and panel object states.


## uCPacketClass library

See [https://github.com/marshalltaylorSFE/XbeeTeensyXCVR/tree/master/uCPacketClass](https://github.com/marshalltaylorSFE/XbeeTeensyXCVR/tree/master/uCPacketClass) for details.

To use, copy `/uCPacketClass` to your arduino libraries, or copy the contents of `/uCPacketClass/src` to your project.

### Library Status


### ExampleRobotCode Status
* Library functions in a basic mode. 
* TODO: Write batch serial operation code
* TODO: Provide a method for including packet ID for struct type

This is released under the [Creative Commons ShareAlike 4.0 International](https://creativecommons.org/licenses/by-sa/4.0/) license. 

Please consider contributing back to this library or others to help the open-source hardware community continue to thrive and grow! 