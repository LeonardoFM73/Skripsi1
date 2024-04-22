/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <Wire.h>
#include <avr/dtostrf.h>

#define I2C_ADDR 0x01 // ショッカクチップのI2Cアドレス (製品に付属する仕様書を参照)
#define DATA_NUM 16 // ショッカクチップの出力データ数
#define BUFF_NUM 10 // 移動平均のサンプリング数
#define NORMALIZATION_COUNT       16 //amount of data for normalization

float D[DATA_NUM];
int xTemp[DATA_NUM],yTemp[DATA_NUM];
float V_calib = 1;

long xMag[17], yMag[17], zMag[17];
long xMagNorm[17], yMagNorm[17], zMagNorm[17];
long tactile_sensX[17],tactile_sensY[17], tactile_sensZ[17];

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("sensor_value", &str_msg);


void setup()
{
  Wire.begin();
  Wire.setClock(1000000);
  MLX_normalization();
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  update_tactile_sensor();

  char data_print[10000];
  int counter=0;
  char str_current_stroke[8], str_current_load[8], str_fruit_length[8];

  dtostrf(90.00, 4, 2, str_current_stroke);
  dtostrf(78.02, 4, 2, str_current_load);
  dtostrf(40.20, 4, 2, str_fruit_length);

//  sprintf(data_print,"%s,%s,%ld,%s,",str_current_stroke, str_current_load, 1, str_fruit_length);
//  Serial.print(data_print);   
//  Serial.println("");
  sprintf(data_print,"s%ld,%ld,%ld,%ld,s%ld,%ld,%ld,%ld,s%ld,%ld,%ld,%ld,s%ld,%ld,%ld,%ld,s%ld,%ld,%ld,%ld,s%ld,%ld,%ld,%ld,s%ld,%ld,%ld,%ld,s%ld,%ld,%ld,%ld,s%ld,%ld,%ld,%ld,s%ld,%ld,%ld,%ld,s%ld,%ld,%ld,%ld,s%ld,%ld,%ld,%ld,s%ld,%ld,%ld,%ld,s%ld,%ld,%ld,%ld,s%ld,%ld,%ld,%ld,s%ld,%ld,%ld,%ld",1,tactile_sensX[0], tactile_sensY[0],tactile_sensZ[0],2,tactile_sensX[1], tactile_sensY[1],tactile_sensZ[1],3,tactile_sensX[2], tactile_sensY[2],tactile_sensZ[2],4,tactile_sensX[3], tactile_sensY[3],tactile_sensZ[3],5,tactile_sensX[4], tactile_sensY[4],tactile_sensZ[4],6,tactile_sensX[5], tactile_sensY[5],tactile_sensZ[5],7,tactile_sensX[6], tactile_sensY[6],tactile_sensZ[6],8,tactile_sensX[7], tactile_sensY[7],tactile_sensZ[7],9,tactile_sensX[8], tactile_sensY[8],tactile_sensZ[8],10,tactile_sensX[9], tactile_sensY[9],tactile_sensZ[9],11,tactile_sensX[10], tactile_sensY[10],tactile_sensZ[10],12,tactile_sensX[11], tactile_sensY[11],tactile_sensZ[11],13,tactile_sensX[12], tactile_sensY[12],tactile_sensZ[12],14,tactile_sensX[13], tactile_sensY[13],tactile_sensZ[13],15,tactile_sensX[14], tactile_sensY[14],tactile_sensZ[14],16,tactile_sensX[15], tactile_sensY[15],tactile_sensZ[15]);  
  str_msg.data = data_print;
  chatter.publish( &str_msg );
  nh.spinOnce();
}

void getFT()
{
  int temp;

  Wire.beginTransmission(I2C_ADDR); // Slave Address 7bit
  Wire.endTransmission(); // Recieve ACK

  // Recieve Data1 (Binary Counter, 14)
  Wire.requestFrom(I2C_ADDR, DATA_NUM*2+1);
  if ( Wire.available() ) temp = Wire.read();

  // Recieve Data2~33
  if ( temp == DATA_NUM*2 )
  {
    for ( int i = 0; i < DATA_NUM*2; i++ )
    {
      // Bind H&L bit of the dada
      if (Wire.available()) temp = Wire.read();
      temp = temp << 8;
      if (Wire.available()) temp = temp | Wire.read();

      if(i%2==0){
          int i_temp=i/2;
          D[i_temp] = temp * V_calib;
      }
      i++;
    }
  }
}

void convertz_to_xy(int i){
      if (i<4){
        yTemp[i]=D[i]*0 + D[i+4]*0.8 + D[i+8]*0.15 + D[i+12]*0.05;
        if (i%4==0 ){
          xTemp[i] = D[i]*0 + D[i+1]*0.8 - D[i+2]*0.15 + D[i+3]*0.05;
        }
        else if(i%4==1){
          xTemp[i] = D[i]*0 - D[i-1]*0.5 + D[i+1]*0.4+D[i+2]*0.1;
        }
        else if(i%4==2){
          xTemp[i] = D[i]*0 - D[i-1]*0.4 - D[i-2]*0.1 + D[i+1]*0.5;
        }
        else if(i%4==3){
          xTemp[i] = D[i]*0 - D[i-1]*0.8 - D[i-2]*0.15 - D[i-3]*0.05;
        } 
      }
      else if (i<8){
        yTemp[i]= D[i]*0 - D[i-4]*0.5 + D[i+8]*0.4+D[i+12]*0.1;
        if (i%4==0 ){
          xTemp[i] = D[i]*0 + D[i+1]*0.8 - D[i+2]*0.15 + D[i+3]*0.05;
        }
        else if(i%4==1){
          xTemp[i] = D[i]*0 - D[i-1]*0.5 + D[i+1]*0.4+D[i+2]*0.1;
        }
        else if(i%4==2){
          xTemp[i] = D[i]*0 - D[i-1]*0.4 - D[i-2]*0.1 + D[i+1]*0.5;
        }
        else if(i%4==3){
          xTemp[i] = D[i]*0 - D[i-1]*0.8 - D[i-2]*0.15 - D[i-3]*0.05;
        } 
      }
      else if (i<12){
        yTemp[i]= D[i]*0 - D[i-4]*0.5 + D[i-8]*0.4 + D[i+12]*0.1;
        if (i%4==0) {
          xTemp[i] = D[i]*0 + D[i+1]*0.8 - D[i+2]*0.15 + D[i+3]*0.05;
        }
        else if(i%4==1){
          xTemp[i] = D[i]*0 - D[i-1]*0.5 + D[i+1]*0.4+D[i+2]*0.1;
        }
        else if(i%4==2){
          xTemp[i] = D[i]*0 - D[i-1]*0.4 - D[i-2]*0.1 + D[i+1]*0.5;
        }
        else if(i%4==3){
          xTemp[i] = D[i]*0 - D[i-1]*0.8 - D[i-2]*0.15 - D[i-3]*0.05;
        } 
      }
      else if (i<16){
        yTemp[i]= D[i]*0 - D[i-4]*0.8 - D[i-8]*0.15 - D[i-12]*0.05;
        if (i%4==0) {
          xTemp[i] = D[i]*0 + D[i+1]*0.8 - D[i+2]*0.15 + D[i+3]*0.05;
        }
        else if(i%4==1){
          xTemp[i] = D[i]*0 - D[i-1]*0.5 + D[i+1]*0.4+D[i+2]*0.1;
        }
        else if(i%4==2){
          xTemp[i] = D[i]*0 - D[i-1]*0.4 - D[i-2]*0.1 + D[i+1]*0.5;
        }
        else if(i%4==3){
          xTemp[i] = D[i]*0 - D[i-1]*0.8 - D[i-2]*0.15 - D[i-3]*0.05;
        } 
      }
}

void MLX_normalization()
{
  for (int i = 0; i < 16; i++) //number of sensor 4 x 4
  {
    xMagNorm[i] = 0;
    yMagNorm[i] = 0;
    zMagNorm[i] = 0;
  }

  for (int j = 0; j < NORMALIZATION_COUNT; j++) //read data NORMALIZATION_COUNT times
  {
    getFT();
    for (int i = 0; i < 16; i++) //number of sensor 4 x 4
    {
      convertz_to_xy(i);
      xMagNorm[i] = xMagNorm[i] + xTemp[i];
      yMagNorm[i] = yMagNorm[i] + yTemp[i];
      zMagNorm[i] = zMagNorm[i] + D[i];
    }
  }

  for (int i = 0; i < 16; i++)
  {
    xMagNorm[i] = xMagNorm[i] / NORMALIZATION_COUNT;
    yMagNorm[i] = yMagNorm[i] / NORMALIZATION_COUNT;
    zMagNorm[i] = zMagNorm[i] / NORMALIZATION_COUNT;
  }
}

void update_tactile_sensor()
{
  getFT();
  for (int i = 0; i < 16; i++)
  {
    convertz_to_xy(i);
    tactile_sensX[i]=xTemp[i] - xMagNorm[i];
    tactile_sensY[i]=yTemp[i] - yMagNorm[i];
    tactile_sensZ[i]=D[i] - zMagNorm[i];
  }
}
