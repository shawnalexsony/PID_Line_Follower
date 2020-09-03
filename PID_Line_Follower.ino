/*
 * LINE FOLLOWER ROBOT 
 * 
 * Uses 5 sensors and PID algorithm 
 * Conditional checking for right angles and 2 way splits
 * Adjust code for junctions according to the track
 * 
 * Get Kp,Kd,Ki values by trial and error
 * First set all to 0
 * Set Kp as 10 and check
 * tune Kp such that it will follow line even if its shaky
 * Change Kd from 0 to some value
 * increase Kd to minimize the wobbling
 * adjust speed and Kd
 * Kp < Kd
 * Set Ki to 0.02 to 0.1
 * adjust Ki by 0.01 values. Very important
 * high Ki value causes large swinging.Keep it minimum
 * Skip Ki if possible
 * tune till you get the perfect performance
*/


//Tuned values

//float Kp = 60, Ki = 0.01, Kd = 90;
//int initial_motor_speed = 150;

//float Kp = 70, Ki = 0.01, Kd = 180;
//int initial_motor_speed = 150;

//float Kp = 30, Ki = 0.0, Kd = 150;
//int initial_motor_speed = 150;

//float Kp = 30, Ki = 0.0, Kd = 170;
//int initial_motor_speed = 150;

float Kp = 30, Ki = 0.0, Kd = 170;
int initial_motor_speed = 160;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int sensor [5] = {0, 0, 0, 0, 0};


void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);

void setup ()
{
// pinMode (10, OUTPUT); // PWM Pin 1
// pinMode (11, OUTPUT); // PWM Pin 2

 pinMode (4, OUTPUT); // Left Motor Pin 1 PWM
 pinMode (5, OUTPUT); // Left Motor Pin 2
 pinMode (6, OUTPUT); // Right Motor Pin 1 PWM
 pinMode (7, OUTPUT); // Right Motor Pin 2
 
 Serial.begin (9600); // Enable Serial Communications
}

void loop ()
{
    read_sensor_values();
    calculate_pid ();
    motor_control ();
}

void read_sensor_values()
{
  sensor [0] = (digitalRead (A2)>0)?0:1;
  sensor [1] = (digitalRead (A3)>0)?0:1;
  sensor [2] = (digitalRead (A4)>0)?0:1;
  sensor [3] = (digitalRead (A5)>0)?0:1;
  sensor [4] = (digitalRead (A1)>0)?0:1;

  for(int ser=0;ser<5;ser++)
  {
    Serial.print(sensor[ser]);
    Serial.print("  ");
  }
  Serial.println(" ");
  
     if (     (sensor [0] == 0) && (sensor [1] == 0) && (sensor [2] == 0) && (sensor [4] == 0) && (sensor [4] == 1)) // 0 0 0 0 1
     error = 4;
     else if ((sensor [0] == 0) && (sensor [1] == 0) && (sensor [2] == 0) && (sensor [3] == 1) && (sensor [4] == 1))// 0 0 0 1 1
     error = 3;
     else if ((sensor [0] == 0) && (sensor [1] == 0) && (sensor [2] == 0) && (sensor [3] == 1) && (sensor [4] == 0))// 0 0 0 1 0
     error = 2;
     else if ((sensor [0] == 0) && (sensor [1] == 0) && (sensor [2] == 1) && (sensor [3] == 1) && (sensor [4] == 0))// 0 0 1 1 0
     error = 1;
     else if ((sensor [0] == 0) && (sensor [1] == 0) && (sensor [2] == 1) && (sensor [3] == 0) && (sensor [4] == 0))// 0 0 1 0 0 // center of line
     error = 0;
     else if ((sensor [0] == 0) && (sensor [1] == 1) && (sensor [2] == 1) && (sensor [3] == 0) && (sensor [4] == 0))// 0 1 1 0 0
     error = -1;
     else if ((sensor [0] == 0) && (sensor [1] == 1) && (sensor [2] == 0) && (sensor [3] == 0) && (sensor [4] == 0))// 0 1 0 0 0
     error = -2;
     else if ((sensor [0] == 1) && (sensor [1] == 1) && (sensor [2] == 0) && (sensor [3] == 0) && (sensor [4] == 0))// 1 1 0 0 0
     error = -3;
     else if ((sensor [0] == 1) && (sensor [1] == 0) && (sensor [2] == 0) && (sensor [3] == 0) && (sensor [4] == 0))// 1 0 0 0 0
     error = -4;
     else if((sensor [0] == 1) && (sensor [1] == 1) && (sensor [2] == 1) && (sensor [3] == 0) && (sensor [4] == 0)) // 1 1 1 0 0 // right angle left
     error = -7;
     else if((sensor [0] == 0) && (sensor [1] == 0) && (sensor [2] == 1) && (sensor [3] == 1) && (sensor [4] == 1)) // 0 0 1 1 1 // right angle right
     error = 7;
     else if((sensor [0] == 0) && (sensor [1] == 1) && (sensor [2] == 1) && (sensor [3] == 1) && (sensor [4] == 0)) // 0 1 1 1 0 // 2 way junction turn right
     error = 2;
     else if((sensor [0] == 0) && (sensor [1] == 1) && (sensor [2] == 0) && (sensor [3] == 1) && (sensor [4] == 0)) // 0 1 0 1 0 // 2 way junction turn right
     error = 3;
     else if((sensor [0] == 1) && (sensor [1] == 1) && (sensor [2] == 0) && (sensor [3] == 1) && (sensor [4] == 1)) // 1 1 0 1 1 // 2 way junction turn right
     error = 4;
     else if((sensor [0] == 1) && (sensor [1] == 0) && (sensor [2] == 0) && (sensor [3] == 0) && (sensor [4] == 1)) // 1 0 0 0 1 // 2 way junction turn right
     error = 5;
     else if((sensor [0] == 1) && (sensor [1] == 1) && (sensor [2] == 1) && (sensor [3] == 1) && (sensor [4] == 1)) // 1 1 1 1 1 // 2 way junction turn slight right
     error = 2;
     else if ((sensor [0] == 0) && (sensor [1] == 0) && (sensor [2] == 0) && (sensor [3] == 0) && (sensor [4] == 0)) // 0 0 0 0 0 // out of line
      {
      if (error <= -1)
        error = -5;
      else if(error >= 1)
        error = 5;
       else 
        error = 0;
      }
}

void calculate_pid()
{
    P = error;
    I = I + error;
    D = error - previous_error;
    
    PID_value = (Kp * P) + (Ki * I) + (Kd * D);
    
    previous_I = I;
    previous_error = error;
}

void motor_control()
{
    // Calculating the effective motor speed:
    int l_motor_speed = initial_motor_speed - PID_value;
    int r_motor_speed = initial_motor_speed + PID_value;
    
    // The motor speed should not exceed the max PWM value
    if(l_motor_speed<0)
    l_motor_speed=0;
    if(l_motor_speed>255)
    l_motor_speed=255;

    if(r_motor_speed<0)
    r_motor_speed=0;
    if(r_motor_speed>255)
    r_motor_speed=255;

    analogWrite (5, l_motor_speed); // Left Motor Speed
    analogWrite (6, r_motor_speed); // Right Motor Speed
    digitalWrite (4, LOW);
    digitalWrite (7, LOW);
}
