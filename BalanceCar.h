#include "MsTimer2.h"
#include "KalmanFilter.h"
#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"

#include "MPU6050.h"
#include "Wire.h"
#include "globals.h"


// Functions declaration short period

void computeInputs();
void acumEncoderPulses();
void readAngle();
void balanceControl();
void combineControls();
void setPwmToMotor();

// Functions in long period

void readLinearAndAngularMovement();
void speedControl();
void angularSpeedControl();

// Objects initialitzation

MPU6050 mpu;
KalmanFilter kalmanfilter;

// Definició de temps de cicle
int dtTascaPrincipalMilis = 5;
int multipleDtLlacVelocitat = 8;

//Setting PID parameters
double kp_balance = 55, kd_balance = 0.75;
double kp_turn = 2.5, kd_turn = 0.5;

double kp_speed = 4.0; // Original
double kp_car_speed_error = 0.0;
double ki_car_speed_error = 0.26; // Original = 0.26 better name thant ki_speed
double kd_car_speed_error = 1.0;

double car_speed_to_pwm = 2.57; // TODO: Calcular amb càrrega. En buit es 2,57

//Setting MPU6050 calibration parameters
double angle_zero = 0;            //x axle angle calibration
double angular_velocity_zero = 0; //x axle angular velocity calibration

volatile unsigned long encoder_count_right_a = 0;
volatile unsigned long encoder_count_left_a = 0;
int16_t ax, ay, az, gx, gy, gz;

float dtSegons = float(dtTascaPrincipalMilis) / 1000.0;

// Kalman filter parameters

float Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;

// posicio cotxe

double moveCarLongPeriod=0.0;
double posicioCar=0.0;
double posicioPulsos=0.0;

// velocitats lineals

double velocitatCar;


// velocitats angulars

double velAngularRoda;

double rotacioParcialGraus = 0.0;
double rotacioCarGraus = 0.0;

// Output control

double balance_control_output;



int leftEncoderPulseAcum = 0;
int rightEncoderPulseAcum = 0;
int encoderRotacioPulse = 0;
int rotationCarPulse = 0;
float rotationCar=0.0;

double speed_control_output = 0.0;
double rotation_control_output = 0.0;
double car_speed_filtered = 0.0;
int speed_control_period_count = 0;

double moveCarPulseLongPeriod = 0.0;
double car_speed_error = 0.0;
double car_speed_error_integral = 0.0;
double car_speed_filtered_old = 0.0;
double carSpeedErrorDif;
double carSpeedErrorOld;

int settingCarSpeed = 0;
int settingTurnSpeed = 0;

int settingCarSpeedManual = 0;
bool manualSpeed = false;

int settingTurnSpeedManual = 0;
bool manualTurnSpeed = false;

// PWM Motor

double pwm_left = 0.0;
double pwm_right = 0.0;

bool pwmManual = false;
double pwmLeftManual = 0.0;
double pwmRightManual = 0.0;

FilterChBp1 filtrePwmL;
FilterChBp1 filtrePwmR;


float kalmanfilter_angle;
// char balance_angle_min = -27;
// char balance_angle_max = 27;
char balance_angle_min = -22;
char balance_angle_max = 22;

void carStop()
{
    digitalWrite(AIN1, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(STBY_PIN, HIGH);
    analogWrite(PWMA_LEFT, 0);
    analogWrite(PWMB_RIGHT, 0);
}

void carForward(unsigned char speed)
{
    digitalWrite(AIN1, 0);
    digitalWrite(BIN1, 0);
    analogWrite(PWMA_LEFT, speed);
    analogWrite(PWMB_RIGHT, speed);
}

void carBack(unsigned char speed)
{
  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}


void carControlTask() {

    // Activar interrupcions?
    sei();

    computeInputs();

    acumEncoderPulses();

    readAngle();

    balanceControl();

    // Section executed at a lower frequency
    speed_control_period_count++;

    if (speed_control_period_count >= multipleDtLlacVelocitat)
    {
        speed_control_period_count = 0;

        readLinearAndAngularMovement();

        speedControl();

        angularSpeedControl();
    }

    combineControls();

    setPwmToMotor();
}

// Functions in the short period cicle

void computeInputs(){

    if (manualSpeed) settingCarSpeed = settingCarSpeedManual;
    if (manualTurnSpeed) settingTurnSpeed = settingTurnSpeedManual;
    
}


void acumEncoderPulses() {

    //suma el que ha comptat durant els ultims n ms al encoder de la roda
    leftEncoderPulseAcum += pwm_left < 0 ? -encoder_count_left_a : encoder_count_left_a;
    rightEncoderPulseAcum += pwm_right < 0 ? -encoder_count_right_a : encoder_count_right_a;

    // Prepara per tornar a comptar
    encoder_count_left_a = 0;
    encoder_count_right_a = 0;

}

void readAngle(){

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dtSegons, Q_angle, Q_gyro, R_angle, C_0, K1);
    kalmanfilter_angle = kalmanfilter.angle;
}


void balanceControl(){

    //PD sobre l'error d'angle TODO: perque Gyro_x no està filtrat? Gyro_x mediex velocitat
    kp_balance = 55;
    kd_balance = 0.75;

    balance_control_output = kp_balance * (kalmanfilter_angle - angle_zero) + kd_balance * (kalmanfilter.Gyro_x - angular_velocity_zero);

}


void combineControls(){

    if (controlModeStr == "pwmManual" ) {
        pwm_left = pwmLeftManual;
        pwm_right = pwmRightManual;
    } else if (controlModeStr == "onlySpeedControl") {
        pwm_left = speed_control_output;
        pwm_right = speed_control_output;
    } else if (controlModeStr == "onlyBalanceControl") {
        pwm_left = balance_control_output;
        pwm_right = balance_control_output;
    } else if (controlModeStr == "onlyRotationControl") {
        pwm_left = -rotation_control_output;
        pwm_right = rotation_control_output;
    } else if (controlModeStr ==  "noSincronControl") {
        
    } else {
        pwm_left = balance_control_output + speed_control_output - rotation_control_output;
        pwm_right = balance_control_output + speed_control_output + rotation_control_output;
    }
        
    
    // TODO: Posar filtre anti resonancia (veure wikipedia). Un fet a la llibreria KalmanFilter.
    // http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=ch&passmode=bp&order=1&chebrip=-3&usesr=usesr&sr=200&frequencyLow=11.4&noteLow=&frequencyHigh=12.2&noteHigh=&pw=pw&calctype=double&run=Send
    // Frequencia a filtrar w=sqrt(g/l)
    // pwm_left = filtrePwmL.step(pwm_left);
    // pwm_right = filtrePwmR.step(pwm_right);

}


void setPwmToMotor(){

    // TODO: Avisar quan es satura. No interessa que s'acosti massa als màxims.
    // Veure grafiques del xip i la seva saturació.

    if (pwm_left<-255 || pwm_left>255) Serial.println("Output: PWM LEFT saturated : " + String(pwm_left));
    if (pwm_right<-255 || pwm_right>255) Serial.println("Output: PWM RIGHT saturated : " + String(pwm_right));

    pwm_left = constrain(pwm_left, -255, 255);
    pwm_right = constrain(pwm_right, -255, 255);

    if (motion_mode != START && motion_mode != STOP && (kalmanfilter_angle < balance_angle_min || balance_angle_max < kalmanfilter_angle)) {
        motion_mode = STOP;
        carStop();
    }

    if (motion_mode == STOP) {
        car_speed_error_integral = 0;
        settingCarSpeed = 0;
        pwm_left = 0;
        pwm_right = 0;
        carStop();
    } else {
        if (pwm_left < 0) {
            digitalWrite(AIN1, 1);
            analogWrite(PWMA_LEFT, -pwm_left);
        } else {
            digitalWrite(AIN1, 0);
            analogWrite(PWMA_LEFT, pwm_left);
        }
        
        if (pwm_right < 0) {
            digitalWrite(BIN1, 1);
            analogWrite(PWMB_RIGHT, -pwm_right);
        } else {
            digitalWrite(BIN1, 0);
            analogWrite(PWMB_RIGHT, pwm_right);
        }
    }

}



/// Functions in long period part


void readLinearAndAngularMovement(){

    moveCarPulseLongPeriod = (leftEncoderPulseAcum + rightEncoderPulseAcum) * 0.5;
    encoderRotacioPulse = leftEncoderPulseAcum - rightEncoderPulseAcum;

    // Resetejem els conmptadors per tal que les interrupcions els tornin a omplir
    leftEncoderPulseAcum = 0;
    rightEncoderPulseAcum = 0;

    rotationCarPulse += encoderRotacioPulse;
    rotationCar = (2.0 * 3.1416) * float(rotationCarPulse) / (PULSEPERROTATION * 2.0 * (WEELSDISTANCE / WEELDIAM));

    velAngularRoda = (2.0 * 3.1416) * (moveCarPulseLongPeriod / PULSEPERROTATION) * (1000.0 / (multipleDtLlacVelocitat*dtTascaPrincipalMilis));

    moveCarLongPeriod = (moveCarPulseLongPeriod / PULSEPERROTATION) * WEELDIAM * 3.1416;
    velocitatCar = (moveCarLongPeriod * 1000.0) / (multipleDtLlacVelocitat*dtTascaPrincipalMilis);

    posicioCar += moveCarLongPeriod;
    posicioPulsos += moveCarPulseLongPeriod;

}


void speedControl() {

    // TODO: Filtre de velocitat. Perquè aquest?
    car_speed_filtered = car_speed_filtered_old * 0.7 + double(moveCarPulseLongPeriod) * 0.3;
    car_speed_filtered_old = car_speed_filtered;

    // Calcul del error
    car_speed_error = settingCarSpeed - car_speed_filtered;

    // Calcul de la integral
    car_speed_error_integral += car_speed_error;
    car_speed_error_integral = constrain(car_speed_error_integral, -3000, 3000);

    // Calcul de la derivada
    carSpeedErrorDif = car_speed_filtered - carSpeedErrorOld;
    carSpeedErrorOld = car_speed_filtered;
    
    // Tal i com funciona BE!!! Algoritme original (sembla un control inestable per ferlo més agil)
    // No hi ha ni proporcional ni derivativa, pero no anar en contra del llaç superior.
    // això explicaria els signes al revés
    // TODO: No entenc perque utilitza car_speed_filtered (enlloc de la consigna) i perque la integral va en negatiu
    kp_speed = 4.0;
    kp_car_speed_error = 0;

    speed_control_output = kp_speed * car_speed_to_pwm * car_speed_filtered; 
    speed_control_output += kp_car_speed_error * car_speed_error - ki_car_speed_error * car_speed_error_integral; // Corregim segons error


    // Test: Llaç teòric de velocitat (funciona be com a llaç de velocitat)
    // kp_speed = 1.5; // compensar cárrega al feed fordward
    // kp_car_speed_error = 0.0; // no pot haver proporcional perquè aniria contra la correcció del angle
    // ki_car_speed_error = -1.0; // La integral ha de ser negativa. No ho entenc, pero funciona
    // kd_car_speed_error = 0.0; // no pot haver diferencial perquè aniria contra la correcció del angle
    // speed_control_output = kp_speed * car_speed_to_pwm * setting_car_speed; // Feed fordward amb la velocitat real
    // speed_control_output += kp_car_speed_error * car_speed_error  + ki_car_speed_error * car_speed_error_integral; // Corregim segons error
    // speed_control_output += kd_car_speed_error * car_speed_error;

}

void angularSpeedControl(){

    // TODO: Llaç de rotació. Corregeig amb un P contra la velocitat, i no contra l'error. Peruquè??
    rotacioParcialGraus = kalmanfilter.Gyro_z * multipleDtLlacVelocitat*dtTascaPrincipalMilis / 1000.0;
    rotacioCarGraus += rotacioParcialGraus;
    //rotacioCarGraus = fmod(rotacioCarGraus, 360.0);

    rotation_control_output = kp_turn * settingTurnSpeed + kd_turn * kalmanfilter.Gyro_z;

}




void encoderCountRightA()
{
  encoder_count_right_a++;
}

void encoderCountLeftA()
{
  encoder_count_left_a++;
}

void carInitialize()
{
    pinMode(AIN1, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(PWMA_LEFT, OUTPUT);
    pinMode(PWMB_RIGHT, OUTPUT);
    pinMode(STBY_PIN, OUTPUT);

    carStop();

    Wire.begin();
    mpu.initialize();
}

void initializeEncoders(){

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), encoderCountLeftA, CHANGE);
    attachPinChangeInterrupt(ENCODER_RIGHT_A_PIN, encoderCountRightA, CHANGE);
}

void launchControlTask(void (*f)(), unsigned long ms){

    //llança una tasca ciclica de n ms
    MsTimer2::set(dtTascaPrincipalMilis, f);
    MsTimer2::start();

}
