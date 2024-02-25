// ↓↓↓↓ Include Files ↓↓↓↓ //
#include <Arduino.h>
 
// Pin configd //
const int Pin_leftMotorIn1 =  26;
const int Pin_leftMotorIn2 =  25;
const int Pin_rightMotorIn1 = 32;
const int Pin_rightMotorIn2 = 33;

void setup()
{
  Serial.begin(115200);
  Serial.println("lixx02 startup"); 
  pinMode(Pin_leftMotorIn1, OUTPUT);
  pinMode(Pin_leftMotorIn2, OUTPUT);
  pinMode(Pin_rightMotorIn1, OUTPUT);
  pinMode(Pin_rightMotorIn2, OUTPUT);
}

void forward()
{
  digitalWrite(Pin_rightMotorIn1, 0);
  digitalWrite(Pin_rightMotorIn2, 1);
  digitalWrite(Pin_leftMotorIn1, 0);
  digitalWrite(Pin_leftMotorIn2, 1);
}

void backward()
{
  digitalWrite(Pin_rightMotorIn1, 1);
  digitalWrite(Pin_rightMotorIn2, 0);
  digitalWrite(Pin_leftMotorIn1, 1);
  digitalWrite(Pin_leftMotorIn2, 0);
}

void right_turn()
{
  digitalWrite(Pin_rightMotorIn1, 1);
  digitalWrite(Pin_rightMotorIn2, 1);
  digitalWrite(Pin_leftMotorIn1, 1);
  digitalWrite(Pin_leftMotorIn2, 0);  
}

void left_turn()
{
  digitalWrite(Pin_rightMotorIn1, 1);
  digitalWrite(Pin_rightMotorIn2, 0);
  digitalWrite(Pin_leftMotorIn1, 1);
  digitalWrite(Pin_leftMotorIn2, 1);  
}

void free(int time_ms)
{
  digitalWrite(Pin_rightMotorIn1, 0);
  digitalWrite(Pin_rightMotorIn2, 0);
  digitalWrite(Pin_leftMotorIn1, 0);
  digitalWrite(Pin_leftMotorIn2, 0);
  delay(time_ms);  
}

void loop()
{
  //左右（白白）→直進
  forward();
  delay(500);
  free(100);

  //左右（白白）→後進
  backward();
  delay(500);
  free(100);  

  //左右（黒白）→右旋回（左モータ正転）
  right_turn();
  delay(500);
  free(100);  

  //左右（白黒）→左旋回（右モータ正転）
  left_turn();
  delay(500);
  free(100);  

}