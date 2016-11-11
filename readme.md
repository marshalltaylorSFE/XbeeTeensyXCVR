# XbeeTeensyXCVR

This repository contains a library and example code for building XBee wireless links between two entities.

### Repository contents

* **Documentation** -- Mostly spreadsheets for keeping track of program data and calculations
* **DualDriveBot** -- Robot and controller code for the dual two-wheeled bot and analog controller
* **HamburgerBot** -- Robot and controller code for the hamburger shaped bot
* **uCPacketClass** -- Arduino library that covers a serial link to provide consistant structures between two microcontrollers.

### Library requirements

The robots rely on [uCModules](https://github.com/marshalltaylorSFE/uCModules) to deal with time.  Install to libraries directory or manually add timerModule32.h, timerModule32.cpp, timeKeeper.h, and timeKeeper.cpp to the sketches where necessary.

#### Analog controller, 4 wheeled robot using teensy 3.1s (DualDriveBot)

* Client sends (X,Y) data on both sticks, button data and calculates polar locally for the debug window
* Host receives (X,Y) data and calculates polar
* Host returns packet containing diagnostic information
* Client has a menu through the OLED that can show debug info
* Fail-safe mechanism implemented.  Turns off in about 1 second.
* Hardware:
  * SerialClientControllerOLED - Teensy 3.2, XBee, XBee breakout, MicroOLED, Analog sticks, lipos, powercell
  * SerialHostTeensy32Robot - Teensy 3.2, Xbee, XBee breakout, SCMD

#### Digital controller, 2 wheeled robot (HamburgerBot)

* Client sends button data in a structure over uCPacketClass
* Host has state machine to determine how to handle the input
* Green LED indicates front of bot, button swaps front for back on the controls.
* Hardware:
  * SerialHostHamburgerBot - Teensy LC, XBee, Serial controlled motor driver v2.0, two WS2812s, SCMD
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