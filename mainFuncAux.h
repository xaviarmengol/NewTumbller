
unsigned long start_prev_time = 0;
boolean carInitialize_en = true;

// test posicio
int estatTest = 0;

void setMotionState() {
  switch (motion_mode) {

  case STANDBY:

    Serial.println("Dins Standby");

    
    if (posicioCar<0.5 && estatTest==0){
      settingCarSpeed = 80.0;
      settingTurnSpeed = 0;
    } else if (posicioCar>=0.5 && estatTest==0) {
      estatTest++;
    } else if (posicioCar>0.0 && estatTest==1) {
      settingCarSpeed = -80.0;
      settingTurnSpeed = 0;
    } else if (posicioCar<=0.0 && estatTest==1) {
      estatTest++;
    } else if (estatTest==2 && rotacioCarGraus<=180.0) {
      settingCarSpeed = 0.0;
      settingTurnSpeed = 100.0;
    } else if (estatTest==2 && rotacioCarGraus>180.0) {
      estatTest++;
    } else if (estatTest==3 && rotacioCarGraus>0.0) {
      settingCarSpeed = 0.0;
      settingTurnSpeed = -100.0;
    } else if (estatTest==3 && rotacioCarGraus<=0.0) {
      estatTest=0;
    }

    break;

  case STOP:

    if (millis() - start_prev_time > 1000)
    {
      
      //function_mode = IDLE;
      if (balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
      {
      
        motion_mode = STANDBY;
        //rgb.lightOff();
      }
    }
    break;

  case START:

    if (millis() - start_prev_time > 2000)
    {
      if (balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
      {
        car_speed_error_integral = 0;
        settingCarSpeed = 0;
        motion_mode = STANDBY;
        //rgb.lightOff();
      }
      else
      {
        motion_mode = STOP;
        carStop();
        //rgb.brightRedColor();
      }
    }
    break;

  default:
    break;
  }
}

/// Clase DEBUG

class DebugPrint {
    public:
        DebugPrint(unsigned long period);
        void printAccGyro(unsigned long period=0);
        void print(unsigned long period=0);

    private:
        unsigned long _periodMillis;
        unsigned long _lastTimeMillis=0;
};

DebugPrint::DebugPrint(unsigned long period){
    _periodMillis = period;
}


void DebugPrint::printAccGyro(unsigned long period){

  if (period != 0) _periodMillis = period;

  if (millis() - _lastTimeMillis > _periodMillis) {

    String printStr = "";

    printStr += "ax: " + String(ax) + " ay: " + String(ay) + " az: " + String(az) + "\n" + 
    "gx: " + String(kalmanfilter.gx) +  " gy: " + String(kalmanfilter.gy) + " gz: "+ String(kalmanfilter.gz) + "\n" +
    "angle: " + String(kalmanfilter_angle) + "\n";

    Serial.print(printStr);

    _lastTimeMillis = millis();
  }

}


void DebugPrint::print(unsigned long period){

    if (period !=0) _periodMillis = period;
   
    if (millis() - _lastTimeMillis > _periodMillis) 
    {
        Serial.println("............");
        Serial.print("VelAngular       : ");
        Serial.println(velAngularRoda);
        Serial.print("Car Speed Settin : ");
        Serial.println(settingCarSpeed);
        Serial.print("Error llaç vel   : ");
        Serial.println(car_speed_error);
        Serial.print("speedcrtoutput   : ");
        Serial.println(speed_control_output);
        Serial.print("Integral Error   : ");
        Serial.println(car_speed_error_integral);
        Serial.print("Posicio Car m    : ");
        Serial.println(posicioCar);
        Serial.print("Posicio Car pul : ");
        Serial.println(posicioPulsos);
        Serial.print("Rotacio Car encoder: ");
        Serial.println(rotationCar);
        Serial.print("Angle           : ");
        Serial.println(kalmanfilter_angle);
        Serial.print("Motion Mode: ");
        Serial.println(motion_mode);

        // Serial.println(kalmanfilter.angle);
        // Serial.println(ultraSo.getDistance());
        // Serial.println(irSensor.obstacleLeft());
        // Serial.println(irSensor.obstacleRight());

        _lastTimeMillis = millis();
    }

}