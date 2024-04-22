/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */
#include <DynamixelShield.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <string.h> // Include the string.h library for strcpy

#define DXL_ID                1
#define DXL_PROTOCOL_VERSION  2.0
#define DXL_SERIAL   Serial

#define OPEN_MAX_POSITION     165
#define CLOSE_MAX_POSITION    90

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long PERIOD_DELAY = 5000;


ros::NodeHandle  nh;

char menu_data = '\0'; // Initialize menu_data globally
char hello[13];
const char* msg_data;

DynamixelShield dxl;
using namespace ControlTableItem;  //This namespace is required to use Control table item names

void messageCb( const std_msgs::String& menu_msg){
    msg_data = menu_msg.data; // Use the data directly as a C-style string
//  Serial2.println(menu_data);
    menu_data = *msg_data; // Store the first character of msg_data in menu_data
}
ros::Subscriber<std_msgs::String> sub("menu", messageCb );

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void setup()
{
//  Serial2.begin(115200);
  nh.getHardware()->setPort(&Serial2);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID);
  dxl.torqueOff(DXL_ID);

  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
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
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(200);
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
    strcpy(hello, "hello world3"); // Copy the string literal into hello array
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 0);
      dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 100); 
      
      dxl.setGoalPosition(DXL_ID, 125, UNIT_DEGREE); 
      delay(500);
  } 
}
