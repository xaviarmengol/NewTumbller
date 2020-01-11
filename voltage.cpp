#include <Arduino.h>
#include "voltage.h"
#include "Pins.h"

Voltage::Voltage() {
    low_voltage_flag = 1;
    vol_measure_time = 0;
    analogReference(INTERNAL);
}

double Voltage::measure() {

    if (millis() - vol_measure_time > 1000) {

        vol_measure_time = millis();

        voltage = (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5); //读取电压值

        if (voltage > 7.8) {
            if (low_voltage_flag == 1) {
                //rgb.lightOff();
                digitalWrite(STBY_PIN,HIGH);
            }
            low_voltage_flag = 0; //满电压标志
        }

        else {
            
            if (voltage < 7.0) { //The battery is low in power and needs to be charged.

                digitalWrite(STBY_PIN,LOW);
            }

            low_voltage_flag = 1; 
        }
    }
    
    return voltage;
}

bool Voltage::isLowVoltage(){
    return low_voltage_flag;
}

