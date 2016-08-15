XbeeTeensyXCVR
====================================

This project's goal is to develop an ascii based wireless link protocol for controlling robots.

Hardware requirements
-------------------
* Teensy 3.x as a controller, reads analog data for stick input
* Teensy LC as a host
* Serial link between Teensys

Library requirements
-------------------
These projects rely on [uCModules](https://github.com/marshalltaylorSFE/uCModules) to deal with time.  Install to libraries directory or manually add timerModule32.h, timerModule32.cpp, timeKeeper.h, and timeKeeper.cpp to the sketches where necessary.

Repository contents
-------------------

* **ExampleRobotCode** -- Contains host and client sketches for actually driving a robot.
* **PacketClassDev** -- Contains host and client sketches for development of the project.
* **PaddleDesign** -- Graphics used to create the controller shape

PacketClassDev Status (Protocol Library branch)
-------------------
THIS DATA NOT VALID:
* Client sends (X,Y) data on both sticks, button data and calculates polar locally for the debug window
* Host receives (X,Y) data but incorrectly calculates polar
* TODO: establish reply data, send time
* TODO: Implement error checking
* TODO: Implement fail-safe mechanism

ExampleRobotCode Status
-------------------
* Robot code is driveable but may incorrectly reverse directions.
* Protocol uses cartesian system and needs to be updated.  This makes it hard to make gradual movements.

This is released under the [Creative Commons ShareAlike 4.0 International](https://creativecommons.org/licenses/by-sa/4.0/) license. 

Please consider contributing back to this library or others to help the open-source hardware community continue to thrive and grow! 