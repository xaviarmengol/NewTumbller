//#include <Arduino.h>
//#include <PinChangeInt.h>

bool left_flag[10] = {false};
unsigned char left_count_flag = 0;
unsigned char left_index = 0;
bool right_flag[10] = {false};
unsigned char right_count_flag = 0;
unsigned char right_index = 0;

char measure_flag = 0;
double distance_value = 0;
unsigned long measure_prev_time = 0;
unsigned long get_distance_prev_time = 0;
char obstacle_avoidance_flag = 0;
unsigned long obstacle_avoidance_prev_time = 0;
int follow_flag = 0;
unsigned long follow_prev_time = 0;
long int turn_count = 0;

unsigned long ir_send_time = 0;
unsigned long left_receive_time = 0;
unsigned char left_receive_flag = 0;
unsigned int left_count = 0;
unsigned long left_count_time = 0;
unsigned long left_test_time = 0;
bool left_test_flag = false;
bool left_is_obstacle = false;

unsigned long right_receive_time = 0;
unsigned char right_receive_flag = 0;
unsigned int right_count = 0;
unsigned long right_count_time = 0;
unsigned long right_test_time = 0;
bool right_test_flag = false;
bool right_is_obstacle = false;



void rightFilter(bool value)
{
  if (right_flag[right_index])
    right_count_flag--;
  if (value)
    right_count_flag++;
  right_flag[right_index] = value;
  right_index++;
  if (right_index >= 10)
    right_index = 0;
}
void leftFilter(bool value)
{
  if (left_flag[left_index])
    left_count_flag--;
  if (value)
    left_count_flag++;
  left_flag[left_index] = value;
  left_index++;
  if (left_index >= 10)
    left_index = 0;
}


void send38K(int pin)
{
  for (int i = 0; i < 39; i++)
  {
    digitalWrite(pin, LOW);
    delayMicroseconds(9);
    digitalWrite(pin, HIGH);
    delayMicroseconds(9);
  }
}


void leftReceive()
{
  if (left_receive_flag == 0)
  {
    left_receive_time = micros();
    left_receive_flag = 1;
    attachPinChangeInterrupt(LEFT_RECEIVE_PIN, leftReceive, RISING);
  }
  else if (left_receive_flag == 1)
  {
    left_test_time = micros() - left_receive_time;
    left_count++;
    left_receive_flag = 0;
    attachPinChangeInterrupt(LEFT_RECEIVE_PIN, leftReceive, FALLING);
  }
}

void rightReceive()
{
  if (right_receive_flag == 0)
  {
    right_receive_time = micros();
    right_receive_flag = 1;
    attachPinChangeInterrupt(RIGHT_RECEIVE_PIN, rightReceive, RISING);
  }
  else if (right_receive_flag == 1)
  {
    right_test_time = micros() - right_receive_time;
    right_count++;
    right_receive_flag = 0;
    attachPinChangeInterrupt(RIGHT_RECEIVE_PIN, rightReceive, FALLING);
  }
}

void ultrasonicInit()
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(IR_SEND_PIN, OUTPUT);
  pinMode(LEFT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_RECEIVE_PIN, INPUT_PULLUP);
  attachPinChangeInterrupt(LEFT_RECEIVE_PIN, leftReceive, FALLING);
  attachPinChangeInterrupt(RIGHT_RECEIVE_PIN, rightReceive, FALLING);
}


//left_is_obstacle
//right_is_obstacle

void checkObstacle()
{
  if (millis() - ir_send_time > 15)
  {
    send38K(IR_SEND_PIN);
    ir_send_time = millis();
  }

  if (millis() - left_count_time > 50)
  {
    // Serial.print(left_count);
    if (left_count >= 3)
    {
      // Serial.print("\tLeft");
      left_is_obstacle = true;
      leftFilter(true);
    }
    else
    {
      left_is_obstacle = false;
      leftFilter(false);
    }

    // Serial.print(left_count_flag);
    if (left_count_flag >= 5)
    {
      // Serial.print("\tLeft");
      left_is_obstacle = true;
    }
    else
    {
      left_is_obstacle = false;
    }

    // Serial.println();
    left_count = 0;
    left_count_time = millis();
  }

  if (millis() - right_count_time > 50)
  {
    // Serial.print("\t\t\t");
    // Serial.print(right_count);
    if (right_count >= 3)
    {
      // Serial.print("\tRight");
      rightFilter(true);
    }
    else
    {
      rightFilter(false);
    }

    // Serial.print("\t\t\t");
    // Serial.print(right_count_flag);
    if (right_count_flag >= 5)
    {
      // Serial.print("\tRight");
      right_is_obstacle = true;
    }
    else
    {
      right_is_obstacle = false;
    }
    // Serial.println();

    right_count = 0;
    right_count_time = millis();
  }
}


//distance_value

void measureDistance() 
{
  if (measure_flag == 0)
  {
    measure_prev_time = micros();
    attachPinChangeInterrupt(ECHO_PIN, measureDistance, FALLING);
    measure_flag = 1;
  }
  else if (measure_flag == 1)
  {
    distance_value = (micros() - measure_prev_time) * 0.017; //340.29 m/s / 2 -> (340.29*100 cm) /(1000*1000 us) / 2 = 0.0170145
    measure_flag = 2;
  }
}


void getDistance()
{
  if (millis() - get_distance_prev_time > 50)
  {
    get_distance_prev_time = millis();
    measure_flag = 0;
    attachPinChangeInterrupt(ECHO_PIN, measureDistance, RISING);
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  }
}

