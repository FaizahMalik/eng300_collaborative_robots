/*
 * rosserial motor interface
 */

#include <ros.h>
#include <autonomous_robot/LR_wheel_target_speed.h>
#include <autonomous_robot/LR_wheel_speed_and_odom.h>

ros::NodeHandle mi; //Motor_Interface
autonomous_robot::LR_wheel_speed_and_odom LR_output;
autonomous_robot::LR_wheel_target_speed LR_input;

//storage variables
int ML_delay = 100;
int pin_order[4] = {0, 1, 3, 2}; // order of the hall sensor output


int LAB_pins[2] = {6, 5};
int RAB_pins[2] = {3, 4};
int L_PWM_pin = 11;
int R_PWM_pin = 3;
int L_CW_CCW[2] = {12, 13};
int R_CW_CCW[2] = {8, 9};

boolean LAB[2] = {0, 0}; //L wheel hall sensors
boolean RAB[2] = {0, 0}; //R wheel hall sensors
int L_counter = 0;
int R_counter = 0;
float L_duty = 0;
float R_duty = 0;
float L_current_speed = 0;
float R_current_speed = 0;
float L_target_speed = 0;
float R_target_speed = 0;
int L_angular_odom = 0;
int R_angular_odom = 0;

float L_ik_1 = 0;
float R_ik_1 = 0;
float L_ek_1 = 0;
float R_ek_1 = 0;

float PID_P = 30;
float PID_I = 5;
float PID_D = 0;
float PID_ek_1 = 0;
float PID_ik_1 = 0;


int n = 1;

void target_speed_cb( const autonomous_robot::LR_wheel_target_speed& target_speed){
  L_target_speed = target_speed.left;
  R_target_speed = target_speed.right;
}

ros::Publisher LR_speed_odom("LR_speed_odom", &LR_output);
ros::Subscriber<autonomous_robot::LR_wheel_target_speed> sub("LR_target_speed", &target_speed_cb);

void setup()
{
  cli(); //stop interrupts
  Serial.begin(57600); // open the serial port at 57600 bps:
  mi.initNode();
  mi.advertise(LR_speed_odom);
  mi.subscribe(sub);
  
  //set pins as outputs
  pinMode(LAB[0], INPUT); //LA
  pinMode(LAB[1], INPUT); //LB
  pinMode(RAB[0], INPUT); //RA
  pinMode(RAB[1], INPUT); //RB
  pinMode(L_CW_CCW[0], OUTPUT);
  pinMode(L_CW_CCW[1], OUTPUT);
  pinMode(R_CW_CCW[0], OUTPUT);
  pinMode(R_CW_CCW[1], OUTPUT);
  pinMode(L_PWM_pin,   OUTPUT);
  pinMode(R_PWM_pin,   OUTPUT);

  digitalWrite(L_CW_CCW[0], LOW);
  digitalWrite(L_CW_CCW[1], LOW);
  digitalWrite(R_CW_CCW[0], LOW);
  digitalWrite(R_CW_CCW[1], LOW);
  analogWrite(L_PWM_pin, 0);
  analogWrite(R_PWM_pin, 0);


//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

//set timer2 interrupt at 2kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 100;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A); 


  sei();//allow interrupts
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt
  mi.spinOnce();
  return;
}

ISR(TIMER2_COMPA_vect){//timer0 interrupt
  L_counter += read_hall_effect(LAB, LAB_pins, 1);
  R_counter += read_hall_effect(RAB, RAB_pins, 0);
  return;
}

int boolean_array_to_int(boolean input[2]){
  int output = input[1] + input[0]*2;
  return output;
}

int wrap_logic(int input, int min, int max){
  int range = max - min + 1;
  if (input < min){
    return input + range;
  }
  if (input > max){
    return input - range;
  }
  return input;
}

int read_hall_effect(boolean AB_then[2], int pin[2], boolean LR){
  int digiread;
  boolean AB_now[2] = {digitalRead(pin[0]), digitalRead(pin[1])};
  int AB_now_int = boolean_array_to_int(AB_now); 
  int AB_then_int = boolean_array_to_int(AB_then); 
  int AB_now_n = pin_order[AB_now_int];
  int AB_then_n = pin_order[AB_then_int];
  if (LR){
    LAB[0] = AB_now[0];
    LAB[1] = AB_now[1];
  }
  else{
    RAB[0] = AB_now[0];
    RAB[1] = AB_now[1];
  }
  
  if (AB_now_n == AB_then_n){
    return 0;
  }
  if (AB_now_n == wrap_logic(AB_then_n + 1, 0, 3)){
    return  1;
  }
  if (AB_now_n == wrap_logic(AB_then_n - 1, 0, 3)){
    return -1;
  }
  else{
    return 0;
  }
}

float calc_wheel_speed(int counter, int sample_time){
  float sensor_freq = counter/(sample_time/1000);
  float wheel_freq = sensor_freq/840;
  float wheel_rads = wheel_freq;// * 2 * 3.1416;
  return wheel_rads;
}

void calc_wheel_speed_LR(){
  L_current_speed = calc_wheel_speed(L_counter, ML_delay);
  R_current_speed = calc_wheel_speed(R_counter, ML_delay);
}

void reset_ML(){
  L_counter = 0;
  R_counter = 0;  
}

float clamp(float input ,float min_n ,float max_n){
  if (input < min_n){
    return min_n;  
  }
  if (input > max_n){
    return max_n;
  }
  return input;
}

int PID(int rk, int yk, float ik_1, float ek_1, float ts, float P, float I, float D){
  float ek = rk - yk;
  float ik = ik_1 + ts * (ek + ek_1) / 2;
  ik = clamp(ik/I, -255, 255);
  float dk = (ek - ek_1) / ts;
  float uk = (P * ek) + (I * ik) + (D * dk);
  uk = clamp(uk, -255, 255);
  PID_ik_1 = ik;
  PID_ek_1 = ek;
  return int(uk);
}

void PID_LR(){
  L_duty = PID(L_target_speed, L_counter, L_ik_1, L_ek_1, ML_delay/1000, PID_P, PID_I, PID_D);
  L_ik_1 = PID_ik_1;
  L_ek_1 = PID_ek_1;
  R_duty = PID(R_target_speed, R_counter, R_ik_1, R_ek_1, ML_delay/1000, PID_P, PID_I, PID_D);
  R_ik_1 = PID_ik_1;
  R_ek_1 = PID_ek_1;
}

void output_motor_power(float duty, int PWM_pin, int CW_CCW[2]){
  if (duty > 0){
    digitalWrite(CW_CCW[0], LOW);
    digitalWrite(CW_CCW[1], HIGH); 
  }
  if (duty < 0){
    digitalWrite(CW_CCW[1], LOW);
    digitalWrite(CW_CCW[0], HIGH);
  }
  if (abs(duty) < 20){
    digitalWrite(CW_CCW[0], LOW);
    digitalWrite(CW_CCW[0], HIGH);
  }
  analogWrite(PWM_pin, abs(duty) * 255/100);
}

void output_motor_power_LR(){
  output_motor_power(L_duty, L_PWM_pin, L_CW_CCW);
  output_motor_power(R_duty, R_PWM_pin, R_CW_CCW);
}

int calc_angular_odometry(float counter){
  float angular_odometry = counter * (360 / 840);
  return (int)angular_odometry;
}

void calc_wheel_odometry_LR(){
  L_angular_odom += L_counter;
  R_angular_odom += R_counter;
}

void publish_speed_odom_LR(){
//  LR_output.left_odom   = L_angular_odom;
//  LR_output.right_odom  = R_angular_odom;
  LR_output.left_odom   = L_duty;
  LR_output.right_odom  = L_ik_1;
  LR_output.left_speed  = L_current_speed;
  LR_output.right_speed = L_target_speed;
  LR_speed_odom.publish( &LR_output );
}

void loop()
{
  while (1){
    delay(ML_delay);
    calc_wheel_speed_LR();
    PID_LR();
    output_motor_power_LR();
    calc_wheel_odometry_LR();
    publish_speed_odom_LR();
    reset_ML();
  }
}
