unsigned long lastTimeMillis=0;
unsigned long start_prev_time = 0;
boolean carInitialize_en = true;

int offSetBT = 0; // OffSet to be modified using BT


void resetStates() {

  settingCarSpeedTarget=0;
  settingCarSpeed = 0;
  settingTurnSpeed=0;
  settingAngle = 0;
  estatTest=0;

}


void setMotionState() {

  if (oldMotionMode != motion_mode) resetStates();

  switch (motion_mode) {

  case STANDBY:

    resetStates();

    break;

  case ROTACIO:

    if (estatTest==0 && rotationCarDeg <= 210) {
      settingTurnSpeed = 80;
      settingCarSpeed = 0;
    } else if (estatTest ==0 && rotationCarDeg>210) {
      estatTest++;
    } else if (estatTest==1 && rotationCarDeg >=30) {
      settingTurnSpeed = -80;
      settingCarSpeed = 0;
    } else if (estatTest == 1 && rotationCarDeg<30) {
      estatTest=0;
    }

    break;

  case MOVEMENT:

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
    } else if (estatTest==2 && rotationCarDeg<=180.0) {
      settingCarSpeed = 0.0;
      settingTurnSpeed = 100.0;
    } else if (estatTest==2 && rotationCarDeg>180.0) {
      estatTest++;
    } else if (estatTest==3 && rotationCarDeg>0.0) {
      settingCarSpeed = 0.0;
      settingTurnSpeed = -100.0;
    } else if (estatTest==3 && rotationCarDeg<=0.0) {
      estatTest=0;
    }

    break;

  case TESTANGLE:
    car_speed_error_integral = 0;
    settingCarSpeedTarget=0;
    settingCarSpeed = 0;
    settingTurnSpeed=0;
    settingAngle = 5 + offSetBT;

    break;

  case AUTONAV:

    // Turn right to find a free way

    settingCarSpeedTarget = map(100, 0, 50, 0, distance_value);
    settingTurnSpeedTarget = map(0, 100, 50, 0, distance_value);

    // limit acceleration 

    settingCarSpeed += constrain((settingCarSpeedTarget - settingCarSpeed), -accMax, accMax);
    settingTurnSpeed += constrain((settingTurnSpeedTarget - settingTurnSpeed), -accTurnMax, accTurnMax);
    lastTimeMillis = millis();

    break;

  case STOP:

    if (millis() - start_prev_time > 1000)
    {
      
      //function_mode = IDLE;
      if (carInSafeAngle())
      {
      
        motion_mode = STANDBY;
        //rgb.lightOff();
      }
    }
    break;

  case START:

    if (millis() - start_prev_time > 2000)
    {
      if (carInSafeAngle())
      {
        car_speed_error_integral = 0;
        settingCarSpeed = 0;
        motion_mode = STANDBY;
        //rgb.lightOff();
      }
      else
      {
        motion_mode = STOP;
        stopCar();
        //rgb.brightRedColor();
      }
    }
    break;

  default:
    break;
  }

  oldMotionMode = motion_mode;
  
}

/// Clase DEBUG

class DebugPrint {
    public:
        DebugPrint(unsigned long period);
        void printAccGyro(unsigned long period=0);
        void print(unsigned long period=0);
        void printAngleControl(unsigned long period=0);
        void printRotControl(unsigned long period=0);

    private:
        unsigned long _periodMillis;
        unsigned long _lastTimeMillis=0;
};

DebugPrint::DebugPrint(unsigned long period){
    _periodMillis = period;
}

//angleControlOutput = -1.0 * (kpAngleControl * (settingAngle - angle) + kdAngleControl * (-kalmanfilter.Gyro_x));

void DebugPrint::printRotControl(unsigned long period){

  if (period != 0) _periodMillis = period;

  if (millis() - _lastTimeMillis > _periodMillis) {
    Serial.println("............");
    Serial.print("Setting Rot Speed: ");
    Serial.println(settingTurnSpeed);
    Serial.print("Rotation Deg     : ");
    Serial.println(rotationCarDeg);
    Serial.print("Rot Speed        : ");
    Serial.println(kalmanfilter.Gyro_z);
    Serial.print("Rotation Setting : ");
    Serial.println(settingTurnSpeed);
    Serial.print("Rot OuputCtr     : ");
    Serial.println(rotationControlOutput);
    Serial.print("PWM Left         : ");
    Serial.println(pwm_left);
    Serial.print("PWM Right        : ");
    Serial.println(pwm_right);

    _lastTimeMillis = millis();

  }

}


void DebugPrint::printAngleControl(unsigned long period){

  if (period != 0) _periodMillis = period;

  if (millis() - _lastTimeMillis > _periodMillis) {
    Serial.println("............");
    Serial.print("Setting Angle    : ");
    Serial.println(settingAngle);
    Serial.print("Angle            : ");
    Serial.println(angle);
    Serial.print("Angle OuputCtr   : ");
    Serial.println(angleControlOutput);
    Serial.print("PWM Left       : ");
    Serial.println(pwm_left);
    Serial.print("PWM Right       : ");
    Serial.println(pwm_right);

    _lastTimeMillis = millis();

  }

}


void DebugPrint::printAccGyro(unsigned long period){

  if (period != 0) _periodMillis = period;

  if (millis() - _lastTimeMillis > _periodMillis) {

    String printStr = "";

    printStr += "ax: " + String(ax) + " ay: " + String(ay) + " az: " + String(az) + "\n" + 
    "gx: " + String(kalmanfilter.gx) +  " gy: " + String(kalmanfilter.gy) + " gz: "+ String(kalmanfilter.gz) + "\n" +
    "angle: " + String(angle) + "\n";

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
        Serial.println(angle);
        Serial.print("Motion Mode: ");
        Serial.println(motion_mode);

        // Serial.println(kalmanfilter.angle);
        // Serial.println(ultraSo.getDistance());
        // Serial.println(irSensor.obstacleLeft());
        // Serial.println(irSensor.obstacleRight());

        _lastTimeMillis = millis();
    }

}


// H_out = Highest possible output
// L_out = Lowest possible output
// H_in = Highest possible input
// L_in = Lowest possible input
// calib = final calibration
// input = the scaling start-off (probably te analog input)

float mapCalibrate(float H_out, float L_out, float H_in, float L_in, float Calib, float Input){

 return  L_out +  ((Input - L_in) /(H_in - L_in)) * (H_out - L_out) + Calib;
}


// Function to manage BT inputs

void keyEventHandle()
{
  if (key_value != '\0')
  {
    key_flag = key_value;

    switch (key_value)
    {
    case 's':
      motion_mode = STANDBY;
      break;
    case 'f': // fordward
      offSetBT++;
      break; // backward
    case 'b':
      break;
      offSetBT--;
    case 'l':
      motion_mode = STANDBY;
      break;
    case 'i':
      motion_mode = STANDBY;
      break;
    case '1': // Follow
      motion_mode = MOVEMENT;
      break;
    case '2': // obstacle
      motion_mode = TESTANGLE;
      break;
    case '3':
      motion_mode = AUTONAV;
      break;
    case '4':
      motion_mode = STOP;
      break;
    case '5':
      motion_mode = START;
      break;
    case '6':
      // Buit
      break;
    case '7':
      // Buit
      break;
    case '8':
      // Buit
      break;
    case '9':
      // Buit
      break;
    case '0':
      // Buit
      break;
    case '*':
      break;
    case '#':
      break;
    default:
      break;
    }
    if (key_flag == key_value)
    {
      key_value = '\0';
    }
  }
}