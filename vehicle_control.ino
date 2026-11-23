#include <string.h>
#include <Car_Library.h>
#include <MsTimer2.h>

String subData;
String data;
String data_unuse;
String transmit_string;

int User_speed, User_angle;
int sensor_flag = 0;
int receive_value[3];

int handle_err = 0;
long sensor_val[7] = {0, };
int cnt = 0;

int analogPin = A5;
int trig_arr[6] = {30, 32, 34, 36, 38, 40};
int echo_arr[6] = {31, 33, 35, 37, 39, 41};
int motorL1 = 4;
int motorL2 = 5;
int motorR1 = 6;
int motorR2 = 7;
int motorH1 = 2;
int motorH2 = 3;

void setup() 
{
  Serial.begin(230400);
  for(int i = 0; i < 6; i++)
  {
    pinMode(trig_arr[i], OUTPUT);
    pinMode(echo_arr[i], INPUT);
  }
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(motorH1, OUTPUT);
  pinMode(motorH2, OUTPUT);

  
  User_speed = 20;
  User_angle = 560;

  delay(100);

//  if(Serial.available())
//  {

    data_unuse = Serial.readStringUntil('\n');
    data = Serial.readStringUntil('\n');
    
    
    int initialVal = 0;
    int val;
    
    for(int i = 0; i < 3; i++)
    {
      val = data.indexOf(',', initialVal);
      subData = data.substring(initialVal, val);
      receive_value[i] = subData.toInt();
      initialVal = val + 1;
    }
    User_speed = receive_value[0];
    User_angle = receive_value[1];
    sensor_flag = receive_value[2];

    
    if(sensor_flag == 1)
      MsTimer2::set(80, motor_compute);
    else if(sensor_flag == 2)
      MsTimer2::set(10, motor_compute);
      
    //delay(100);
    transmit_string += String(sensor_val[0]);
    transmit_string += String(',');
    transmit_string += String(sensor_val[1]);
    transmit_string += String(',');
    transmit_string += String(sensor_val[2]); 
    transmit_string += String(',');
    transmit_string += String(User_speed);
    transmit_string += String(',');
    transmit_string += String(User_angle);
    transmit_string += String(',');
    transmit_string += String(sensor_flag);
  
    Serial.println(transmit_string);
    transmit_string = "";
  
    MsTimer2::start();
//  }
  
}

void loop() 
{
  if(Serial.available())
  {
    MsTimer2::stop();
    motor_forward(motorH1, motorH2, 0);
    data = Serial.readStringUntil('\n');

    int initialVal = 0;
    int val;

    for(int i = 0; i < 3; i++)
    {
      val = data.indexOf(',', initialVal);
      subData = data.substring(initialVal, val);
      receive_value[i] = subData.toInt();
      initialVal = val + 1;
    }
    User_speed = receive_value[0];
    User_angle = receive_value[1];
    sensor_flag = receive_value[2];

    if(sensor_flag!=1)
    {
      sensor_flag = 1;
    }

    transmit_string += String(sensor_val[0]);
    transmit_string += String(',');
    transmit_string += String(sensor_val[1]);
    transmit_string += String(',');
    transmit_string += String(sensor_val[2]);
// 
//    for(int i = 1; i < 4; i++)
//    {
//      transmit_string += String(',');
//      transmit_string += String(sensor_val[i]);
//    }

    transmit_string += String(',');
    transmit_string += String(User_speed);
    transmit_string += String(',');
    transmit_string += String(User_angle);
    transmit_string += String(',');
    transmit_string += String(sensor_flag);

    Serial.println(transmit_string);
    transmit_string = "";
    MsTimer2::start();
  }
  delay(10);
}

void motor_compute(void)
{
  if(sensor_flag == 1)
  {
      sensor_val[0] = ultrasonic_distance(trig_arr[0], echo_arr[0]);
      sensor_val[1] = ultrasonic_distance(trig_arr[1], echo_arr[1]);
      sensor_val[2] = ultrasonic_distance(trig_arr[2], echo_arr[2]);
//    sensor_val[cnt] = ultrasonic_distance(trig_arr[cnt], echo_arr[cnt]);
//    sensor_val[cnt + 3] = ultrasonic_distance(trig_arr[cnt + 3], echo_arr[cnt + 3]);
//    
//    cnt++;
//    if(cnt > 2)
//     cnt = 0;
  }
//  if(sensor_flag == 2)
//  {
//    motor_hold(motorL1, motorL2);
//    motor_hold(motorR1, motorR2);
//    motor_hold(motorH1, motorH2);
//  }
  
  sensor_val[6] = analogRead(A5);
  handle_err = (User_angle - sensor_val[6]) / 2;

  if(abs(handle_err) < 100)
  {
    if(handle_err < 0)
      handle_err = -100;
    else
      handle_err = 100;
  }

  if(handle_err < 0)
    motor_backward(motorH1, motorH2, -(handle_err));
  if(handle_err > 0)
    motor_forward(motorH1, motorH2, handle_err);

  if(User_speed >= 0)
  {
    motor_forward(motorL1, motorL2, User_speed);
    motor_forward(motorR1, motorR2, User_speed);  
  }
  else
  {
    motor_backward(motorL1, motorL2, -User_speed);
    motor_backward(motorR1, motorR2, -User_speed);
  }
}