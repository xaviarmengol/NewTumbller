
unsigned long start_prev_time = 0;
boolean carInitialize_en = true;

// test posicio
int estatTest = 0;

void setMotionState() {
  switch (motion_mode) {

  case STANDBY:

    Serial.println("Dins Standby");
    //setting_turn_speed = 0;
    // TEST
    //setting_car_speed = 0;
    
    if (posicioCar<0.5 && estatTest==0){
      setting_car_speed = 80.0;
      setting_turn_speed = 0;
    } else if (posicioCar>=0.5 && estatTest==0) {
      estatTest++;
    } else if (posicioCar>0.0 && estatTest==1) {
      setting_car_speed = -80.0;
      setting_turn_speed = 0;
    } else if (posicioCar<=0.0 && estatTest==1) {
      estatTest++;
    } else if (estatTest==2 && rotacioCarGraus<=180.0) {
      setting_car_speed = 0.0;
      setting_turn_speed = 100.0;
    } else if (estatTest==2 && rotacioCarGraus>180.0) {
      estatTest++;
    } else if (estatTest==3 && rotacioCarGraus>0.0) {
      setting_car_speed = 0.0;
      setting_turn_speed = -100.0;
    } else if (estatTest==3 && rotacioCarGraus<=0.0) {
      estatTest=0;
    }

    break;

  case STOP:

    Serial.println("Stop");
    if (millis() - start_prev_time > 1000)
    {
      Serial.println("Entra if");
      //function_mode = IDLE;
      if (balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
      {
        Serial.println("Entra canvi estat");
        motion_mode = STANDBY;
        //rgb.lightOff();
      }
    }
    break;

  case START:
    Serial.println("Start");

    if (millis() - start_prev_time > 2000)
    {
      if (balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
      {
        car_speed_error_integral = 0;
        setting_car_speed = 0;
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
        void print();

    private:
        unsigned long _periodMillis;
        unsigned long _lastTimeMillis=0;
};

DebugPrint::DebugPrint(unsigned long period){
    _periodMillis = period;
}

void DebugPrint::print(){
   
    if (millis() - _lastTimeMillis > _periodMillis) 
    {
        Serial.println("............");
        Serial.print("VelAngular       : ");
        Serial.println(velAngularRoda);
        Serial.print("Car Speed Settin : ");
        Serial.println(setting_car_speed);
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
        Serial.println(rotacioCarGrausEncoder);
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