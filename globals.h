#ifndef globals_h
#define globals_h

#include "mode.h"

#define WEELDIAM 0.105 // Diameter of the weels
#define WEELSDISTANCE 0.17 // Distance between center of weels
#define PULSEPERROTATION 780 // Pulses every rotation (data sheet)

FUNCTION_MODE function_mode;
MOTION_MODE motion_mode;

String controlModeStr="";


#endif