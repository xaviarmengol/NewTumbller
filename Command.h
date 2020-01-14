#include <Arduino.h>
#include <PinChangeInt.h>
#include <Pins.h>
#include "globals.h"

char key_value = '\0';
char key_flag = '\0';
char prev_key_mode = 0;
unsigned long key_mode_time = 0;

void increaseMotionMode() {
  if (millis() - key_mode_time > 500) {
    motion_mode = MOTION_MODE((motion_mode + 1)%NUMMODES);


    key_mode_time = millis();
  }
}

void keyMotionModeInit()
{
  pinMode(KEY_MODE, INPUT_PULLUP);
  attachPinChangeInterrupt(KEY_MODE, increaseMotionMode, FALLING);
}


bool getBluetoothData()
{
  if (Serial.available())
  {
    char c = Serial.read();
    if (c != '\0' && c != '\n')
    {
      key_value = c;
     
    }
  }
  return false;
}
