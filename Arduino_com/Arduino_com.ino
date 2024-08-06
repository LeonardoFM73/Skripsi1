/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */
#include <DynamixelShield.h>
#include <Wire.h>
#include <ros.h>
//#include <dtostrf.h>
#include <std_msgs/String.h>
#include <string.h> // Include the string.h library for strcpy

#define DXL_ID                1
#define DXL_PROTOCOL_VERSION  2.0
#define DXL_SERIAL   Serial

#define OPEN_MAX_POSITION     165
#define CLOSE_MAX_POSITION    80
#define OFFSET_DEGREE_SENSOR  10
#define DEGREE_COMPENSATION   90


#define MIN_STROKE            5
#define MAX_STROKE            77

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

int stateGripper = 0;
byte stateData = 0;
byte stateGrasp = READY;
byte stateCollect = READY;
byte collect_status = READY;
bool running = false;

#define THRESHOLD_GRASP_LENGTH  5
#define THRESHOLD_GRASP_TORQUE  0.7

float moving_finger = OPEN_MAX_POSITION;
float current_stroke = MAX_STROKE;
float target_stroke = MAX_STROKE;

float gripper_motor_degree,gripper_stroke_length,gripper_moving_length,gripper_load;
float fruit_length;

int set_moving = OPEN_MAX_POSITION;
int period_count = 0;

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long PERIOD_DELAY = 5000;


ros::NodeHandle  nh;

char menu_data = '\0'; // Initialize menu_data globally
char hello[13];
char Status[13];
char stateGripperStr[1]; // Ukuran array karakter untuk menampung string hasil konversi
char stateCollectStr[1];
char data_print[100];
const char* msg_data;
const char* sensor_data;

DynamixelShield dxl;
using namespace ControlTableItem;  //This namespace is required to use Control table item names

void messageCb( const std_msgs::String& menu_msg){
    msg_data = menu_msg.data; // Use the data directly as a C-style string
//  Serial2.println(menu_data);
    menu_data = *msg_data; // Store the first character of msg_data in menu_data
}
ros::Subscriber<std_msgs::String> sub("menu", messageCb );

//void callback_sensor( const std_msgs::String& data_msg){
//    sensor_data = data_msg.data; // Use the data directly as a C-style string
//}
//ros::Subscriber<std_msgs::String> sub_sensor("sensor_value", callback_sensor );

std_msgs::String str_msg,servo_value,collect;
ros::Publisher Gripper("Gripper", &str_msg);
ros::Publisher Status_collect("Status_collect", &str_msg);
ros::Publisher data_servo("data_servo", &servo_value);

void setup()
{
//  Serial2.begin(115200);
  nh.getHardware()->setPort(&Serial2);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(data_servo);
  nh.advertise(Gripper);
  nh.advertise(Status_collect);
  
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID);
  dxl.torqueOff(DXL_ID);

  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  gripper_init();
}


void loop()
{
//  DEBUG_SERIAL.println("Keluar");
//    char str_buffer[2]; // Buffer to hold the character as a string
//    str_buffer[0] = menu_data; // Assign the character
//    str_buffer[1] = '\0'; // Null-terminate the string
//
//    str_msg.data = str_buffer; // Assign the C-style string to std_msgs::String
  str_msg.data = hello;  
  servo_value.data = data_print;
  collect.data = Status;
  data_servo.publish( &servo_value );
  Gripper.publish(&str_msg);
  Status_collect.publish(&collect);
  nh.spinOnce();
  gripper_control();
  if (strcmp(msg_data, "1") == 0) {
    strcpy(hello, "hello world1"); // Copy the string literal into hello array
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 0);
      dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 100);
      
      dxl.setGoalPosition(DXL_ID, 165, UNIT_DEGREE);
      delay(500);
    }
    else if (strcmp(msg_data, "2") == 0) {
      strcpy(hello, "hello world2"); // Copy the string literal into hello array
        dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 0);
        dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 100);
      
        dxl.setGoalPosition(DXL_ID, 90, UNIT_DEGREE);
        delay(500);
    }
    else if (strcmp(msg_data, "3") == 0) {
      stateGripper = COLLECT_DATASET;
    }
    else if (strcmp(msg_data, "10") == 0) {
      stateData = 10;
    }  
}

void sending_data(float a,float b){
  char str_current_stroke[8], str_current_load[8], str_fruit_length[8];
  
  dtostrf(a, 4, 2, str_current_stroke);
  dtostrf(b, 4, 2, str_current_load);
  dtostrf(fruit_length, 4, 2, str_fruit_length);
  
  sprintf(data_print,"%s,%s,%s",str_current_stroke,str_current_load,str_fruit_length);
}

void sending_gripper(int a){
  sprintf(stateGripperStr, "%d", stateCollect);
  strcpy(hello, stateGripperStr); // Copy the string literal into hello array
}
void status_collect(int a){
  sprintf(stateCollectStr, "%d", collect_status);
  strcpy(Status, stateCollectStr); // Copy the string literal into hello array
}

void gripper_init(){
  move_degree_gripper(CLOSE_MAX_POSITION + OFFSET_DEGREE_SENSOR + 5);
  delay(1000);
  move_degree_gripper(OPEN_MAX_POSITION);
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
  gripper_motor_degree = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
  gripper_stroke_length = degree_to_stroke_calculation(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE));
  gripper_moving_length = moving_calculation(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE));
  gripper_load = torque_calculation(dxl.readControlTableItem(PRESENT_LOAD, DXL_ID));
  
  sending_data(gripper_stroke_length,gripper_load);
  sending_gripper(stateCollect);
  status_collect(collect_status);
  
  switch(stateGripper)
  {
    case GUI_CONTROL:
//      move_stroke_gripper(MAX_STROKE);
      stateGrasp = READY;
      stateCollect = READY;
      period_count = 0;;
      //move_degree_gripper(set_moving);
//      move_stroke_gripper(set_moving);
    break;

    case KALIBRASI:
//      kalibrasi_gripper();
      stateGripper = GUI_CONTROL;
      stateData = 12;
      running = true;    
    break;

    case POTENSIO_CONTROL:
//      move_stroke_gripper(potensio_control());
    break;

    case CONTROL_DEMO:
//      control_demo(gripper_motor_degree);
    break;

    case COLLECT_DATASET:
      collect_dataset(gripper_stroke_length, gripper_load);
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

void collect_dataset(float current_pos, float current_load)
{
  const float moving_stroke = 0.2;

  // Menggunakan sprintf() untuk mengonversi nilai stateGripper ke dalam string
  switch(stateCollect)
  {    
    case READY:
      move_stroke_gripper(MAX_STROKE);
      current_stroke = MAX_STROKE;
      target_stroke = MAX_STROKE;      
      period_count++;
      if(period_count>25){
        period_count = 0;
        stateCollect = REACH_TARGET;
      }
    break;

    case REACH_TARGET:
      stateCollect = GRIPPER_CLOSING;
    break;

    case GRIPPER_CLOSING:
      float load;
      load = current_load;
//      if(check_z>185 or abs(check_x)>195 or abs(check_y)>195)
      if(fabs(load) > 0.35)
      //if(current_pos < 50)
      {
        startMillis = millis();
        fruit_length = current_pos;
        stateCollect = SQUEEZING;
        collect_status = FINISH;
      }
      else {
        target_stroke = current_stroke - moving_stroke;
        if(target_stroke<MIN_STROKE) {
          stateCollect = RETURN_FRUIT;
          collect_status = FINISH;
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
        target_stroke = current_stroke - moving_stroke/2;
        if(target_stroke<MIN_STROKE) {
          stateCollect = RETURN_FRUIT;
          delay(1000);
        }
        move_stroke_gripper(target_stroke);
      }
      current_stroke = target_stroke;
    break;

    case RETURN_FRUIT:
      stateCollect = FINISH;  
      collect_status = READY;
      stateGripper = GUI_CONTROL;
      stateData=12;
      fruit_length=0;
      
    break;

  }
}
