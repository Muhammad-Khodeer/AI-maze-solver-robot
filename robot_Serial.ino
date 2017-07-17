/***************************************************************************************************   
  ****  About the Project : 
  ****  [Flash memory - program memory ] : Maximum : 32,256 bytes | used : 10,860 bytes | (33%) used
  ****  [Dynamic memory - part of Ram]   : Maximum : 2,048 bytes  | used : 748 bytes    | (36%) used
  ****  Team NO :
  ****  Date :
  **** 
****************************************************************************************************/
//[Libraries]----------------------------------------------------------------------------------
#include <math.h>
#include <Timer.h>
Timer t;

//[hash defines]--------------------------------------------------------------------------------
#define encoder_positive 8       // power for left Encoder | second encoder is direct power

#define encoder0PinA  2          // left Encoder at interrupt pin [pin 2]
#define motorL 5                 // Left Motor = motor 1
#define motor1_pwm2 6            // left PWM

#define motorR 9                 // Right Motor 
#define encoder1PinA  3          // right Encoder at interrupt pin [pin 3]
#define motor2_pwm2 10           // right PWM

#define enable 7

#define SIZE 89

//[variables]-----------------------------------------------------------------------------------

//Define Variables we'll be connecting to
double motor_output_R;
double motor_output_L;

volatile unsigned long encoderR=0, encoderL=0;
unsigned long encoderR_old=0, encoderL_old=0;
int encoderR_dif=0, encoderL_dif=0;

char incomingByte; // for incoming serial data

int get_x_y_new_Event;
int go_to_goal_Event;

double DR = 0, DL =0;
double DC, delta_theta, prev_theta, new_theta;
double x, y, x_new, y_new, x_goal, y_goal;

double diff_x, diff_y, theta_goal, theta_error;

int send_data_event;


String message="";
int commaPosition;
unsigned int value[SIZE];
unsigned int i;
int val_x;
int val_y;
unsigned int index = 0;
unsigned int num_no = 0;
//------------------------------------------------------------------------------------------------

void setup() 
{
  //opens serial port, sets data rate to 115200 bps
  Serial.begin(115200);
  /*********************[Set variables]***********************/
  motor_output_R = 0;    
  motor_output_L = 0;
  /*******************[inputs pins][2 pins]***************/
  // Encoder0 reader pin
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);// turn on pullup resisto
  // Encoder1 reader pin
  pinMode(encoder1PinA, INPUT); 
  digitalWrite(encoder1PinA, HIGH);

  /*****************[outputs pins][ pins]*****************/
  // Enable
  pinMode(enable , OUTPUT);     
  digitalWrite(enable,HIGH);      
  // Encoder Power
  pinMode(encoder_positive,OUTPUT);
  digitalWrite(encoder_positive, HIGH);
  // Right Motor
  pinMode(motorR,OUTPUT);
  attachInterrupt(0, doEncoder_0, RISING);  // encoder pin on interrupt 0 - pin 2
  analogWrite(motorR,motor_output_R);
  //Left Motor 
  pinMode(motorL,OUTPUT);
  attachInterrupt(1, doEncoder_1, RISING);  // encoder pin on interrupt 1 - pin 3
  analogWrite(motorL,motor_output_L);
  //
  analogWrite(motor1_pwm2 , LOW);   
  analogWrite(motor2_pwm2 , LOW);  
  /*******************************************************/
  // timer Run [send_data] function every 1000
  send_data_event = t.every(1000, send_data);  
}
//-------------------------------------------------------------------------------------------
void loop() 
{
  // 
  t.update();
  // Main Runs only when serial is available
  while (Serial.available())
  {
    // Convert string data from serial to List of integers [value]
    // and get number of elements
    char msg = Serial.read();
    if (msg == '*')
    {
      index = 0;
      num_no = 0; 
      message = Serial.readStringUntil('$');

      Serial.print("Message is: ");
      Serial.println(message);

      for (i = 0; i < SIZE; i++)
      { 
        commaPosition = message.indexOf(',');
        if (commaPosition != -1 && commaPosition >= 0)
        { 
          value[i] = message.substring(0, commaPosition).toInt();
          message = message.substring(commaPosition + 1, message.length());
          num_no += 1;
        }
        else if (commaPosition == -1 && message.length() > 0)
        {
          if (message != "")
            value[i] = message.toInt();
          num_no += 1;
          break;
        }
      }
      // refresh position (x, y) from value list
      // goal position to reach 
      val_x=value[index];
      val_y=value[index + 1];
      index += 2;
      x_goal = val_x;
      y_goal = val_y;
      // get the current position of the Robot by Encoder
      get_x_y_new_Event = t.every(100, get_x_y_new_direct);
      // move 
      go_to_goal_Event = t.every(100, go_to_goal);
    }
  }// End of serial
 // >> here serial is not connected anymore
}
//----------------------------------------------------------------------------------------------
void send_data()
{
  Serial.print(" x: ");
  Serial.print(x);
  Serial.print(" y: ");
  Serial.println(y);
}
//---------------------------------------------------------------------------------------------
void go_to_goal()
{
  // every 100 ms
  //  velocity = 6;
  diff_x = x_goal - x;
  diff_y = y_goal - y;
  //  distance_error = sqrt((diff_x*diff_x)+(diff_y*diff_y));
  theta_goal = atan2(y_goal ,x_goal);  //0.785
  theta_error = theta_goal - prev_theta;
  theta_error = atan2(sin(theta_error), cos(theta_error));

  if(theta_error > 0)
  {
    analogWrite(motorR, 130); //125  // if one motor is faster 
    analogWrite(motorL, 0);
  }
  else
  {
    analogWrite(motorR, 0);
    analogWrite(motorL, 125); //125
  }

  // here we reach our point
  if((abs(diff_x)<20) && (abs(diff_y)<20))
  {
    // print goal position
    Serial.print(" x_goal: ");
    Serial.print(x_goal);
    Serial.print(" y_goal: ");
    Serial.println(y_goal);
    //then stop the robot
    analogWrite(motorR, 0);
    analogWrite(motorL, 0);
    //t.stop(go_to_goal_Event);
    //t.stop(get_x_y_new_Event);
    if(index>=num_no)
    {
      Serial.println("Disconnected !");
      t.stop(go_to_goal_Event);
      t.stop(get_x_y_new_Event); 
    }
    else
    {
      delay(500);
      val_x=value[index];
      val_y=value[index + 1];
      x_goal = val_x;
      y_goal = val_y;
      
      Serial.print(" x_goal: ");
      Serial.print(x_goal);
      Serial.print(" y_goal: ");
      Serial.print(y_goal);
      Serial.print("   index:   ");
      Serial.print(index);
      Serial.println();
      index += 2;
    }
  }
}
//-----------------------------------------------------------------------------------------
void get_x_y_new_direct()
{
  // every 100 ms
  encoderR_dif = encoderR - encoderR_old;
  encoderR_old = encoderR;
  encoderL_dif = encoderL - encoderL_old;
  encoderL_old = encoderL;

  DR = 2*3.1418*3.25 * encoderR_dif/8.0;
  DL = 2*3.1418*3.25 * encoderL_dif/8.0;

  DC = (DR+DL)/2.0;
  delta_theta = (DL-DR)/17.0;

  x_new = x + DC * cos(prev_theta);
  y_new = y + DC * sin(prev_theta);
  new_theta = prev_theta + delta_theta;

  x = x_new;
  y = y_new;
  prev_theta = new_theta;
  prev_theta = atan2(sin(prev_theta), cos(prev_theta));
} 
//-----------------------------------------------------------------------------------------------
void doEncoder_0()
{
  encoderR++;
}
//-----------------------------------------------------------------------------------------------
void doEncoder_1()
{
  encoderL++;
}
