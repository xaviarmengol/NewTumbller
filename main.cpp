#include <Arduino.h>
#include "PinChangeInt.h"
#include "Pins.h"
#include "BalanceCar.h"
#include "Ultrasonic.h"
#include "voltage.h"
#include "globals.h"
#include "Command.h"

#include "mainFuncAux.h" // TODO: Arreglar

// Objectes aplicació

Voltage volt;
DebugPrint debugPrint(1000);

// Main variables

double voltage;
double distanceUltraSo;

unsigned long startTime=0;
bool flagStart = true;


void setup()
{
    Serial.begin(115200);
    ultrasonicInit();
        
    kalmanfilter.setGyroOffset(-400, -56, -218); // From calibration

    keyMotionModeInit();

    // inicialització de les globals

    motion_mode = START;

    carInitialize();

    initializeEncoders();

    //tasca principal ciclica que mante l'equilibri

    manualSpeed = false;
    settingCarSpeedManual = 0;

    manualTurnSpeed = false;
    settingTurnSpeedManual = 0;

    controlModeStr="std"; //pwmManual onlySpeedControl onlyBalanceControl onlyRotationControl noSincronControl

    launchControlTask(carControlTask, 5);

    pwmLeftManual = 0.0;//double(pasTest) * 30.0;
    pwmRightManual = 0.0;//double(pasTest) * 30.0;

    startTime = millis();
    start_prev_time = millis();

}


// El control d'equilibri NO es fa al LOOP. Es fa a una tasca síncrona que s'ha llançat al SETUP

void loop()
{
    getDistance();
    checkObstacle();

    //right_is_obstacle;
    //left_is_obstacle;
    //distance_value;

    voltage = volt.measure();
    if (volt.isLowVoltage()) {
        if (DEBUG==1){
            Serial.println("Low Battery. Going to STOP: " + String(voltage) + "V");
            motion_mode = STOP;
        }
    }

    getBluetoothData(); //LBE uses standar serial port, as is linked with external module

    setMotionState();

    //debugPrint.printAccGyro(250);
    //debugPrint.print(250);
    //debugPrint.printAngleControl(250);
    debugPrint.printRotControl(250);
}