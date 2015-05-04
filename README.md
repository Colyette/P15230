P15230
======

Used for P15230 Quadcopter Project
Home Directory
------
Use command:
$make test_grid 
Builds the map that A* Search for navigation path planning

NOTE: Updated version in mainProg folder. This implementation does not have backtracking if it finds itself into
a newly discovered corner.

start Folder
-----
Add to home directory of the RPi
Contains scripts to start the main program at a start of a button.
note: first script needs to run as reboot of system

mainProg Folder
----
Add to home directory of the RPi
Contains the classes required for the program to run on the RPi. Makefile
contains test compiles as well as the main program compile:
$make all

MpiCom Folder
-------
Used for testing (soon to be deleted)

Arduino Folder
------
Contains code to run on the slave Arduino. All projects but teensy slave
are obsolete for the project moved to using a Teensy instead of 2 Unos.

WiFi_Tri Folder
------
C code to test radial distance calculations derived from signal strength. 
Currently cannot scan multiple times in one program run, does not test
scaning of multiple routers.
