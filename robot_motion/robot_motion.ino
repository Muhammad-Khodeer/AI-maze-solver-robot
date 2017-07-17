/*
 vmax = 95 pulses per 200 ms = 23.75 pulses per 50ms
 vmax = 2*pi*R*(23/420) cm per 50ms
 2*pi*R = 31, L = 35
 vmax = 31*(23.75/420) = 1.75 cm per 50ms
 v = (R/2)(vr+vl)
 w = (R/L)(vr-vl)
 wheel_radius = R = 31/(2*3.1418);
 v_max = (1/2)(vr+vl)
 w_max = (1/L)(vr-vl) = (1/L)((1.75 per 50ms) - 0)
 = (1.75 per 50ms)/35 = 0.05
 */
 
int v_max = 1.75;
int w_max = 0.05;

#include <math.h>
#include <Timer.h>

Timer t;

//Define Variables we'll be connecting to
double motor_output_R;
double motor_output_L;

#define encoder_positive 8
#define encoder0PinA  2
#define motorR 5
#define encoder1PinA  3
#define motorL 11

volatile unsigned long encoderR=0, encoderL=0;
unsigned long encoderR_old=0, encoderL_old=0;
int encoderR_dif=0, encoderL_dif=0;

char incomingByte; // for incoming serial data

int get_x_y_new_Event;
int go_to_goal_Event;

void setup() {
  motor_output_R = 0;
  motor_output_L = 0;

  pinMode(encoder_positive,OUTPUT);
  digitalWrite(encoder_positive, HIGH);

  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resisto
  pinMode(motorR,OUTPUT);
  attachInterrupt(0, doEncoder_0, RISING);  // encoder pin on interrupt 0 - pin 2
  analogWrite(motorR,motor_output_R);

  pinMode(encoder1PinA, INPUT); 
  digitalWrite(encoder1PinA, HIGH);
  pinMode(motorL,OUTPUT);
  attachInterrupt(1, doEncoder_1, RISING);  // encoder pin on interrupt 1 - pin 3
  analogWrite(motorL,motor_output_L);

  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
}

double DR = 0, DL =0;
double DC, delta_theta, radius, prev_theta, new_theta;
double x, y, x_new, y_new, x_goal, y_goal;
double Kp, Ki, Kd, prev_err, accum_err;
boolean has_reached_goal = false;

void loop() {
  t.update();

  get_x_y_new_Event = t.every(50, get_x_y_new_direct);
  x_goal = 60;
  y_goal = 60;
  Kp = 0.05; 
  Ki = 0.0001; 
  Kd = 0.001;
  prev_err = 0; 
  accum_err = 0;
  has_reached_goal = false;
  go_to_goal_Event = t.every(250, go_to_goal);
}

double diff_x, diff_y, distance_error, theta_goal, theta_error;
double Prop_error, Int_error, Dif_error, PID_output;
double omega, velocity, vel_r, vel_l, wheel_radius;
double vel_r_in_pulses, vel_l_in_pulses, vel_rl_max, vel_rl_min;
double vel_Kp, vel_PID_output;

void go_to_goal()
{
  // every 250 ms
  velocity = 1.4;
  diff_x = x_goal - x;
  diff_y = y_goal - y;
  distance_error = sqrt((diff_x*diff_x)+(diff_y*diff_y));
  theta_goal = atan2(diff_y ,diff_x);  //0.785
  theta_error = theta_goal - prev_theta;
  theta_error = atan2(sin(theta_error), cos(theta_error));

  //Kp = 0.05; Ki = 0.0001; Kd = 0.001; 
  Prop_error = theta_error;
  Int_error = accum_err + theta_error;
  Dif_error = theta_error - prev_err;
  PID_output = Kp*Prop_error + Ki*Int_error + Kd*Dif_error;

  accum_err = Int_error;
  prev_err = theta_error;

  omega = PID_output;
  if(omega > w_max)
    omega = w_max;
  else if(omega < -w_max)
    omega = -w_max;

  unicycle_to_diff_drive();
  output_shaping();

  motor_output_R = map(vel_r*100, 0, v_max*100, 0, 255);
  motor_output_L = map(vel_l*100, 0, v_max*100, 0, 255);
  analogWrite(motorR, motor_output_R);
  analogWrite(motorL, motor_output_L);

  if((abs(diff_x)<5) && (abs(diff_y)<5))
  {
    // stop
    analogWrite(motorR, 0);
    analogWrite(motorL, 0);
    t.stop(go_to_goal_Event);
    t.stop(get_x_y_new_Event);
  }
}

//with output shaping
void unicycle_to_diff_drive()
{
  // 2*pi*R=31
  wheel_radius = 31/(2*3.1418);	
  vel_r = (2*velocity+omega*35)/(2*wheel_radius);
  vel_l = (2*velocity-omega*35)/(2*wheel_radius);
}

void output_shaping()
{
  vel_rl_max = max(vel_r, vel_l);
  vel_rl_min = min(vel_r, vel_l);
  if(vel_rl_max > v_max)
  {
    vel_r = vel_r - (vel_rl_max-1.75);
    vel_l = vel_l - (vel_rl_max-1.75);
  }
  if(vel_rl_min < 0)
  {
    vel_r = vel_r + (0 - vel_rl_min);
    vel_l = vel_l + (0 - vel_rl_min);
  }
  else
  {
    vel_r = vel_r;
    vel_l = vel_l;
  }
}

void get_x_y_new_direct()
{
  // every 50 ms
  encoderR_dif = encoderR - encoderR_old;
  encoderR_old = encoderR;
  encoderL_dif = encoderL - encoderL_old;
  encoderL_old = encoderL;

  DR = 31.0 * encoderR_dif/420.0;
  DL = 31.0 * encoderL_dif/420.0;

  DC = (DR+DL)/2.0;
  delta_theta = (DR-DL)/35.0;

  x_new = x + DC * cos(prev_theta);
  y_new = y + DC * sin(prev_theta);
  new_theta = prev_theta + delta_theta;

  x = x_new;
  y = y_new;
  prev_theta = new_theta;
} 

void doEncoder_0()
{
  encoderR++;
}

void doEncoder_1()
{
  encoderL++;
}



