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
        
    kalmanfilter.setGyroOffset(-400, -56, -218); // From calibration

    // inicialització de les globals

    function_mode = IDLE;
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

void loop()
{
    // El control d'equilibri NO es fa al LOOP. Es fa a una tasca síncrona que s'ha llançat al SETUP

    getDistance();
    checkObstacle();

    //right_is_obstacle;
    //left_is_obstacle;
    //distance_value;

    voltage = volt.measure();
    if (volt.isLowVoltage()) {
        Serial.println("Low Battery. Going to STOP: " + String(voltage) + "V");
        motion_mode = STOP;
    }

    setMotionState();

    //debugPrint.printAccGyro(250);
    debugPrint.print(250);
}