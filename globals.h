#ifndef globals_h
#define globals_h

#include <Arduino.h>

#define WEELDIAM 0.105 // Diameter of the weels
#define WEELSDISTANCE 0.17 // Distance between center of weels
#define PULSEPERROTATION 780 // Pulses every rotation (data sheet)
#define NUMMODES 7

#define DEBUG 0

enum MOTION_MODE
{
  STANDBY,
  ROTACIO,
  TESTANGLE,
  MOVEMENT,
  AUTONAV,
  STOP,
  START,
};

MOTION_MODE motion_mode;
MOTION_MODE oldMotionMode;

String controlModeStr="";

// estats sequencies test
int estatTest = 0;

#endif