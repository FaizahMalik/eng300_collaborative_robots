/*
 * rosserial motor interface
 */

#include <ros.h>
#include <autonomous_robot/LR_wheel_target_speed.h>
#include <autonomous_robot/LR_wheel_odom.h>

ros::NodeHandle mi; //Motor_Interface
autonomous_robot::LR_wheel_target_speed LR_input;
autonomous_robot::LR_wheel_odom LR_output;

//storage variables

/*
 * ARD - arduino
 * MD = L298n
 * PWR = power source
 * LM = left motor
 * RM = right motor(
 *   M+  = Red
 *   M-  = Black
 *   VCC = Brown
 *   GND = Green
 *   CHA = Blue
 *   CHB = Purple)
 *   
 * arduino uno:
 * ARD P2 -> LM CHA
 * ARD P3 -> LM CHB
 * ARD P4 -> RM CHB
 * ARD P5 -> MD ENB
 * ARD P6 -> MD ENA
 * ARD P7 -> RM CHA
 * ARD P8 -> MD IN4
 * ARD P9 -> MD IN3
 * ARD P10-> MD IN2
 * ARD P11-> MD IN1
 * ARD 5V -> LM VCC, RM VCC, MD 5V
 * ARD GND-> LM GND, RM GND, MD GND, PWR GND
 * 
 * L298n:
 * MD OUT1 -> LM M-
 * MD OUT2 -> LM M+
 * MD OUT3 -> RM M+
 * MR OUT4 -> RM M-
 * MD 5V   -> LM VCC, RM VCC, ARD 5V 
 * MD GND  -> ARD GND, LM GND, RM GND
 * MD 12V  -> PWR 12V
 * MD ENA  -> ARD P6
 * MD IN1  -> ARD P11
 * MD IN2  -> ARD P10
 * MD IN3  -> ARD P9
 * MD IN4  -> ARD P8
 * MD ENB  -> ARD P5
 * 
 * left motor:
 * LM M+  -> MD OUT2
 * LM M-  -> MD OUT1
 * LM VCC -> ARD 5V
 * LM GND -> GND
 * LM CHA -> ARD P2
 * LM CHB -> ARD P3
 * 
 * right motor:
 * RM M+  -> MD OUT3
 * RM M-  -> MD OUT4
 * RM VCC -> ARD 5V
 * RM GND -> GND
 * RM CHA -> ARD P7
 * RM CHB -> ARD P4 
 */

int pin_order[4] = {0, 1, 3, 2}; // order of the hall sensor output
int LAB_pins[2] = {3, 2};
int RAB_pins[2] = {7, 4};
int L_PWM_pin = 6;
int R_PWM_pin = 5;
int L_CW_CCW[2] = {10, 11}; //should be 13, requires replacement arduino
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

float PID_P = 2;
float PID_I = 0.001;
float PID_D = -0.2;
float PID_ek_1 = 0;
float PID_ik_1 = 0;
float PID_rate = 0.2;

void tws_cb( const autonomous_robot::LR_wheel_target_speed& LR_tws){
  L_target_speed = LR_tws.left;
  R_target_speed = LR_tws.right;
}

ros::Publisher LR_odom("LR_odom", &LR_output);
ros::Subscriber<autonomous_robot::LR_wheel_target_speed> sub("LR_TWS", &tws_cb);

void setup()
{
  cli(); //stop interrupts
  Serial.begin(57600); // open the serial port at 57600 bps:
  mi.initNode();
  mi.advertise(LR_odom);
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
  OCR1A = 1042;// = (16*10^6) / (1*1024) - 1 (must be <65536) # 1562 = 10hz
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  sei();//allow interrupts
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt
  publish_odom_LR();
  PID_LR();
  output_motor_power_LR();
  mi.spinOnce();
  reset_LR_counters();
  return;
}

void publish_odom_LR(){
  LR_output.left   = L_target_speed - L_counter;
  LR_output.right  = R_target_speed - R_counter;
  LR_odom.publish( &LR_output );
}

void PID_LR(){
  L_duty = PID(L_target_speed, L_counter, L_ik_1, L_ek_1, PID_rate, PID_P, PID_I, PID_D);
  L_ik_1 = PID_ik_1;
  L_ek_1 = PID_ek_1;
  R_duty = PID(R_target_speed, R_counter, R_ik_1, R_ek_1, PID_rate, PID_P, PID_I, PID_D);
  R_ik_1 = PID_ik_1;
  R_ek_1 = PID_ek_1;
}

float PID(int rk, int yk, float ik_1, float ek_1, float ts, float P, float I, float D){
  float ek = rk - yk;
  float ik = ik_1 + ts * (ek + ek_1) / 2;
  ik = clamp((ik/I)-5, -255, 255);
  float dk = (ek - ek_1) / ts;
  float uk = (P * ek) + (I * ik) + (D * dk) + (P * rk);
  uk = clamp(uk, -255, 255);
  PID_ik_1 = ik;
  PID_ek_1 = ek;
  return int(uk);
}

void output_motor_power_LR(){
  output_motor_power(L_duty, L_PWM_pin, L_CW_CCW);
  output_motor_power(R_duty, R_PWM_pin, R_CW_CCW);
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
  if (abs(duty) < 50){
    digitalWrite(CW_CCW[1], LOW);
    digitalWrite(CW_CCW[0], LOW);
  }
  analogWrite(PWM_pin, abs(duty));
}

void reset_LR_counters(){
  L_counter = 0;
  R_counter = 0;
}

///////////////////////////////////////////////////

float clamp(float input ,float min_n ,float max_n){
  if (input < min_n){
    return min_n;  
  }
  if (input > max_n){
    return max_n;
  }
  return input;
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

void loop()
{
  while (1){
    delay(0.45);
    L_counter += read_hall_effect(LAB, LAB_pins, 1);
    R_counter += read_hall_effect(RAB, RAB_pins, 0);
  }
}
