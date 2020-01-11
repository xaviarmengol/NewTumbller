#include <Arduino.h>
#include "PinChangeInt.h"
#include "Pins.h"
#include "mode.h"
#include "BalanceCar.h"
#include "Ultrasonic.h"
#include "voltage.h"
#include "globals.h"

#include "mainFuncAux.h" // TODO: Arreglar

// Objectes aplicació

Voltage volt;
DebugPrint debugPrint(1000);

// Main variables

double voltage;
double distanceUltraSo;
unsigned long lastTimeMillis;

unsigned long startTime=0;
bool flagStart = true;


void setup()
{
    Serial.begin(115200);
    ultrasonicInit();
    //irSensor.irInit();
    

    // inicialització de les globals

    function_mode = IDLE;
    motion_mode = START;

    //tasca principal ciclica que mante l'equilibri
    carInitialize();

    //test////////////////////////////////////////////////////////////////////////////////////////////7
    //pwmManual = true;
    //pwmLeftManual = 0.0;//double(pasTest) * 30.0;
    //pwmRightManual = 0.0;//double(pasTest) * 30.0;

    //velocitatManual = true;
    //settingCarSpeedManual = 0;

    startTime = millis();
    start_prev_time = millis();

}

void loop()
{
    getDistance();
    checkObstacle();

    voltage = volt.measure();
    if (volt.isLowVoltage()) {
        Serial.println("Low Battery. Going to STOP");
        motion_mode = STOP;
    }
    
    setMotionState();

    debugPrint.print();
}