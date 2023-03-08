/*
 * rosserial motor interface
 */

#include <ros.h>
#include <autonomous_robot/LR_wheel_pwm.h>
#include <autonomous_robot/LR_wheel_odom.h>

ros::NodeHandle mi; //Motor_Interface
autonomous_robot::LR_wheel_pwm LR_input;
autonomous_robot::LR_wheel_odom LR_output;

//storage variables
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
int L_duty = 0;
int R_duty = 0;
int L_angular_odom = 0;
int R_angular_odom = 0;

void pwm_cb( const autonomous_robot::LR_wheel_pwm& LR_duty){
  L_duty = LR_duty.left;
  R_duty = LR_duty.right;
}

ros::Publisher LR_odom("LR_odom", &LR_output);
ros::Subscriber<autonomous_robot::LR_wheel_pwm> sub("LR_PWM", &pwm_cb);

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
  OCR1A = 4000;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
}


ISR(TIMER1_COMPA_vect){//timer1 interrupt
  mi.spinOnce();
  return;
}

void publish_odom_LR(){
  LR_output.left   = L_duty;
  LR_output.right  = R_duty;
  LR_odom.publish( &LR_output );
}

void loop()
{
  while (1){
    delay(100);
    publish_odom_LR();
  }
}
