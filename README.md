# HATino
Head tracking using ARDUINO and BLUETOOTH reusing code from HATIRE, using GY-521 breakout board (MPU-6050) and HC-05 breakout board for
bluetooth communications. The system is powered by standard AA-batteries, so it's a complete standalone solution.

This project is aimed to create a head tracking system based on code released by FuraX49 for the HATIRE plugin for OpenTRACK.

OpenTRACK is an OpenSource project focused on head tracking for MSWindows, Linux and Apple OSX. 
Manages multiple tracking sources and support multiple outputs. See https://github.com/opentrack/opentrack
for more details.

HATino provides a full solution (hardware and software) for a head tracking system based on arduino,
using bluetooth as comunication layer, in order to avoid "wires". HATino is implemented on an
ARDUINO MICRO, with a custom PCB Board, using GY-521 IMU breakout board for gyro information, and
HC-05 bluetooth breakout board for communications. The project includes both hardware (schematics,
pcb HOW-TO info) and software (ARDUINO custom firmware and modifications, Bluetooth layer) in order
to get a complete, OpenSource system. The software is based on FuraX49' code, and some of the 
examples on i2clib for arduino.

Libraries:

I2Cdev			https://github.com/jrowberg/i2cdevlib.git
Wire			(included)
SoftwareSerial	(included)

For information on installing libraries, see: http://arduino.cc/en/Guide/Libraries