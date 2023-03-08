/*
 * rosserial motor interface
 */

#include <ros.h>
#include <autonomous_robot/LR_wheel_odom.h>

ros::NodeHandle mi; //Motor_Interface
autonomous_robot::LR_wheel_odom LR_output;

//storage variables
int pin_order[4] = {0, 1, 3, 2}; // order of the hall sensor output
boolean LAB[2] = {0, 0}; //L wheel hall sensors
boolean RAB[2] = {0, 0}; //R wheel hall sensors
int LAB_pins[2] = {6, 5};
int RAB_pins[2] = {3, 4};
int L_counter = 0;
int R_counter = 0;


ros::Publisher LR_odom("LR_odom", &LR_output);

void setup()
{
  cli(); //stop interrupts
  Serial.begin(57600); // open the serial port at 57600 bps:
  mi.initNode();
  mi.advertise(LR_odom);
  
  //set pins as outputs
  pinMode(LAB[0], INPUT); //LA
  pinMode(LAB[1], INPUT); //LB
  pinMode(RAB[0], INPUT); //RA
  pinMode(RAB[1], INPUT); //RB

//set timer1 interrupt at 5Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 1000;// = (16*10^6) / (5*1024) - 1 (must be <65536)
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
  publish_odom_LR();
  L_counter = 0;
  R_counter = 0;
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

void publish_odom_LR(){
  LR_output.left   = L_counter;
  LR_output.right  = R_counter;
  LR_odom.publish( &LR_output );
}

void loop()
{
  while (1){
    L_counter += read_hall_effect(LAB, LAB_pins, 1);
    R_counter += read_hall_effect(RAB, RAB_pins, 0);
    L_counter += 1;
    R_counter += 1;
    Serial.println(L_counter);
    L_counter = wrap_logic(L_counter, -100, 100);
    R_counter = wrap_logic(R_counter, -100, 100);
  }
}
