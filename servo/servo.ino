/*******************************************************************************
  Gripper Control System & Command Control
  BRIN
*******************************************************************************/

#include <DynamixelShield.h>
#include <Wire.h>
//#include <avr/dtostrf.h>
#include <math.h>


#define OPEN_MAX_POSITION     165
#define CLOSE_MAX_POSITION    80  
#define OFFSET_DEGREE_SENSOR  10
#define DEGREE_COMPENSATION   90

#define MIN_STROKE            3
#define MAX_STROKE            75

#define GUI_CONTROL           0
#define KALIBRASI             1
#define POTENSIO_CONTROL      2
#define CONTROL_DEMO          3
#define COLLECT_DATASET       4
#define ML_ACT1               5

#define READY                 0
#define REACH_TARGET          1
#define SAFE_GRASPING         2
#define GRIPPER_CLOSING       3
#define SQUEEZING             4
#define RETURN_FRUIT          5
#define FINISH                6

#define _1x1                 0
#define _4x4                 1

#define THRESHOLD_GRASP_LENGTH  5
#define THRESHOLD_GRASP_TORQUE  0.7   
//#define PERIOD_DELAY            5000    

#define POTENSIO_PIN          A0

#define DXL_ID                1
#define DXL_PROTOCOL_VERSION  2.0

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DXL_SERIAL   Serial
#define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
#define DEBUG_SERIAL SerialUSB
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
//OpenRB does not require the DIR control pin.
#define DXL_SERIAL Serial1
//#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL Serial
#else
#define DEBUG_SERIAL Serial
#endif

DynamixelShield dxl;

using namespace ControlTableItem;  //This namespace is required to use Control table item names

byte stateGripper = 0;
byte stateData = 0;
byte stateGrasp = READY;
byte stateCollect = READY;

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long PERIOD_DELAY = 5000;

bool stateCalibration = false;

bool running = false;
bool selected_sensor = _4x4;
float moving_finger = OPEN_MAX_POSITION;
float current_stroke = MAX_STROKE;
float target_stroke = MAX_STROKE;

float fruit_length;

int set_moving = OPEN_MAX_POSITION;
int period_count = 0;

long xMag[17], yMag[17], zMag[17];
long xMagNorm[17], yMagNorm[17], zMagNorm[17];
long tactile_sensX[17],tactile_sensY[17], tactile_sensZ[17];


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);

  Serial2.begin(115200);
  delay(250);
  Serial2.println("--------------------------------");
  Serial2.println("-----Gripper Control System-----");
  Serial2.println("--------------------------------");
  delay(1000);
  Serial.begin(115200);
  
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID);
  dxl.torqueOff(DXL_ID);  // Turn off torque when configuring items in EEPROM area

  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);

  Serial2.print("Sensor Calibration");
  gripper_init();
  Serial2.print("Finished!");
}

void loop() {
  // put your main code here, to run repeatedly:
  command_control();
  if(running)
  {
    gripper_control();
    //debug_data();
  }  
}

//END of loop

//--------------------Gripper Function---------------------------------

void gripper_init(){
  move_degree_gripper(CLOSE_MAX_POSITION + OFFSET_DEGREE_SENSOR + 5);
  delay(1000);
  move_degree_gripper(OPEN_MAX_POSITION);
}



void command_control(){
  if (Serial2.available() > 0)
  {
    char stateReceive = Serial2.read();
    if(stateReceive=='1') stateData = 1;
    else if(stateReceive=='2') stateData = 2;
    else if(stateReceive=='3') stateData = 3;
    else if(stateReceive=='4') stateData = 4;
    else if(stateReceive=='5') stateData = 5;
    else if(stateReceive=='6') stateData = 6;
    else if(stateReceive=='7') stateData = 7;
    else if(stateReceive=='8') stateData = 8;
    else if(stateReceive=='9') stateData = 9;
    else if(stateReceive=='A') stateData = 10;
    else if(stateReceive=='B') stateData = 11;
    else if(stateReceive=='C') stateData = 12;    
  }
  if (stateData == 1) running = true;
  else if (stateData == 2) running = false;
  if (running)
  {
    if (stateData == 3) set_moving = MAX_STROKE;
    else if (stateData == 4) set_moving = MIN_STROKE;    
    else if (stateData == 5) stateGripper = KALIBRASI; 
    else if (stateData == 6) stateGripper = CONTROL_DEMO; 
    else if (stateData == 7) stateGripper = COLLECT_DATASET; 
    else if (stateData == 8) stateGripper = ML_ACT1; 
    else if (stateData == 9) dxl.torqueOn(DXL_ID);
    else if (stateData == 10) dxl.torqueOff(DXL_ID);
    else if (stateData == 11) stateGripper = POTENSIO_CONTROL;
    else if (stateData == 12) stateGripper = GUI_CONTROL; 
  }
}

float check_limit_degree(float current_val)
{
  if(current_val <= CLOSE_MAX_POSITION + OFFSET_DEGREE_SENSOR) current_val = CLOSE_MAX_POSITION + OFFSET_DEGREE_SENSOR;
  else if(current_val >= OPEN_MAX_POSITION) current_val = OPEN_MAX_POSITION;
  return current_val; 
}

float check_limit_stroke(float stroke)
{
  if(stroke <= MIN_STROKE) stroke = MIN_STROKE;
  else if(stroke >= MAX_STROKE) stroke = MAX_STROKE;
  return stroke;
}

float degree_to_stroke_calculation(float theta_gripper)
{ 
  const float a1 = 45.761;
  const float d1 = 13.45;
  const float a2 = 55.663;
  const float d2 = 10.005;
  const float offset_length_sensor = 10;
  return (a1*sin(radians(theta_gripper-DEGREE_COMPENSATION)) + d1-(d2+offset_length_sensor))*2; 
}

float stroke_to_degree_calculation(float stroke_length)
{
  const float a1 = 45.761;
  const float d1 = 13.45;
  const float a2 = 55.663;
  const float d2 = 10.005;
  const float offset_length_sensor = 9;
  return degrees(asin(((stroke_length/2)-d1+(d2+offset_length_sensor))/a1));
}

float moving_calculation(float theta_gripper)
{
  const float a1 = 45.761;
  const float a2 = 55.663;
  return a1*cos(radians(theta_gripper-DEGREE_COMPENSATION)) + a2;   
}

float torque_calculation(float present_load)
{
  const float max_motor_torque = 1.5;
  return (present_load/1000)*max_motor_torque;
}

void gripper_control()
{
  float gripper_motor_degree = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
  float gripper_stroke_length = degree_to_stroke_calculation(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE));
  float gripper_moving_length = moving_calculation(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE));
  float gripper_load = torque_calculation(dxl.readControlTableItem(PRESENT_LOAD, DXL_ID));
  Serial2.println(gripper_motor_degree);
  Serial2.println(gripper_stroke_length);
  Serial2.println(gripper_moving_length);
  Serial2.println(gripper_load);
  

  switch(stateGripper)
  {
    case GUI_CONTROL:
      stateGrasp = READY;
      stateCollect = READY;
      period_count = 0;;
      //move_degree_gripper(set_moving);
      move_stroke_gripper(set_moving);
    break;

    case KALIBRASI:
      kalibrasi_gripper();
      stateGripper = GUI_CONTROL;
      stateData = 12;
      running = true;    
    break;

    case POTENSIO_CONTROL:
      move_stroke_gripper(potensio_control());
    break;

    case CONTROL_DEMO:
      control_demo(gripper_motor_degree);
    break;

    case COLLECT_DATASET:
      collect_dataset(gripper_stroke_length, gripper_load);
    break;

    case ML_ACT1:
    break;

  }
}


//-----------------------Motion Function---------------------------

void move_degree_gripper(float target_value)
{
  dxl.torqueOn(DXL_ID);
  target_value = check_limit_degree(target_value);
  dxl.setGoalPosition(DXL_ID, target_value, UNIT_DEGREE);
}

void move_stroke_gripper(float stroke)
{
  stroke = check_limit_stroke(stroke);  
  float target_theta = stroke_to_degree_calculation(stroke);
  move_degree_gripper(target_theta + DEGREE_COMPENSATION);
}

float potensio_control()
{
  long val_total = 0;
  for (int i = 0; i < 128; i++)
  {
    int val = analogRead(POTENSIO_PIN);
    val_total += val;
  }
  double val_double = (double) val_total / 128;
  float val_stroke = val_double * 360 / 1024;

  //invert
  val_stroke = 360 - val_stroke;  
  val_stroke = map(val_stroke, 0, 360,MIN_STROKE, MAX_STROKE);

  return val_stroke;
}

void control_demo(float current_degree)
{
  float moving_degree = 1;

  switch(stateGrasp)
  {
    case READY:
      move_degree_gripper(OPEN_MAX_POSITION);
      moving_finger = OPEN_MAX_POSITION;
      period_count++;
      if(period_count>50){
        period_count = 0;
        stateGrasp = MOVING;
      }
    break;

    case GRIPPER_CLOSING:
      if(tactile_sensZ[0]>150 or abs(tactile_sensX[0])>200 or abs(tactile_sensY[0])>200)
      {
        period_count++;
        if(period_count>1){
          period_count = 0;
          stateGrasp = SAFE_GRASPING;
        }
      }
      else {
        moving_finger = moving_finger - moving_degree;
        if(moving_finger<CLOSE_MAX_POSITION + OFFSET_DEGREE_SENSOR) {
          stateGrasp = READY;
          delay(1000);
        }
        move_degree_gripper(moving_finger);
      }
    break;

    case SAFE_GRASPING:
      if(tactile_sensZ[0]>300 or abs(tactile_sensX[0])>350 or abs(tactile_sensY[0])>350)
      {
        period_count++;
        if(period_count>50){
          period_count = 0;
          stateGrasp = READY;
        }
      }
      else {
        moving_finger = moving_finger - moving_degree;
        if(moving_finger<CLOSE_MAX_POSITION + OFFSET_DEGREE_SENSOR) {
          stateGrasp = READY;
          delay(1000);
        }
        move_degree_gripper(moving_finger);
      }
    break;
  }
}

void collect_dataset(float current_pos, float current_load)
{
  const float moving_stroke = 0.2;
  switch(stateCollect)
  {    
    case READY:
      move_stroke_gripper(MAX_STROKE);
      current_stroke = MAX_STROKE;
      target_stroke = MAX_STROKE;      
      period_count++;
      if(period_count == 1);
      if(period_count>25){
        period_count = 0;
        stateCollect = REACH_TARGET;
      }
    break;

    case REACH_TARGET:
      stateCollect = GRIPPER_CLOSING;
    break;

    case GRIPPER_CLOSING:
      long check_x, check_y, check_z;
      float load;
      load = current_load;

//      if(check_z>185 or abs(check_x)>195 or abs(check_y)>195)
      if(load < -0.35)
      //if(current_pos < 50)
      {
        startMillis = millis();
        fruit_length = current_pos;
        stateCollect = SQUEEZING;
      }
      else {
        target_stroke = current_stroke - moving_stroke;
        if(target_stroke<MIN_STROKE) {
          stateCollect = FINISH;
          delay(1000);
        }
        move_stroke_gripper(target_stroke);
      }
      current_stroke = target_stroke;
    break;
    
    case SQUEEZING:
      if((fabs(current_pos - fruit_length) > THRESHOLD_GRASP_LENGTH) || (fabs(current_load)> THRESHOLD_GRASP_TORQUE) || (period_count>0))
      {
        period_count++;
        if(period_count>150) {
          period_count = 0;
          stateCollect = RETURN_FRUIT;
        }
      }
      else if (period_count == 0)
      {
        if(fabs(current_pos - fruit_length) > THRESHOLD_GRASP_LENGTH/3){
//          dxl.setGoalPosition(DXL_ID2, 180, UNIT_DEGREE);
        }
        target_stroke = current_stroke - moving_stroke/2;
        if(target_stroke<MIN_STROKE) {
          stateCollect = FINISH;
          delay(1000);
        }
        move_stroke_gripper(target_stroke);
      }
      current_stroke = target_stroke;
    break;

    case RETURN_FRUIT:
        stateCollect = FINISH;
    break;

    case FINISH:
      period_count++;
      if(period_count>50 && period_count<75){
          move_stroke_gripper(MAX_STROKE);
      }
      else if(period_count>75){
            period_count = 0;
            stateCollect = READY;
            stateGripper = GUI_CONTROL;
            fruit_length = 0;
            stateData = 12;
      }
  }
}


void kalibrasi_gripper()
{
  gripper_init();
}
