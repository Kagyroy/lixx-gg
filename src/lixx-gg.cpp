/* Readme

//// Tags ////

#indev（in development）作成中
#unveri（unverified）作ったけど動作未検証
#imlat（improve later)　後で改良
#devlat（develop later)　後で実装
#dellat（delete later） 後で消す

*/ 

#include <Arduino.h>
#include "mpu6050.h"
#include "BluetoothSerial.h"

//bluetoothSerial Related
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//// Class Declarations ////

BluetoothSerial SerialBT;
 
//// Pin config ////

//for Motor
const int Pin_leftMotorIn1 =  26;
const int Pin_leftMotorIn2 =  25;
const int Pin_rightMotorIn1 = 32;
const int Pin_rightMotorIn2 = 33;

const int pwm_Pin_leftMotorIn1 =  0;  //ledcAttachPin(Pin_leftMotorIn1, 0); 
const int pwm_Pin_leftMotorIn2 =  1;  //ledcAttachPin(Pin_leftMotorIn2, 1); 
const int pwm_Pin_rightMotorIn1 = 2;  //ledcAttachPin(Pin_rightMotorIn1, 2); 
const int pwm_Pin_rightMotorIn2 = 3;  //ledcAttachPin(Pin_rightMotorIn2, 3);
  
//for PhotoReflector
const int Pin_leftSensor = 36;
const int Pin_rightSensor = 39;

//// Constants ////

//sample： ledcSetup(0, pwmFreq, pwmReso); //0ch, 12,8kHz, 10bit(1024)Resolution
const int pwmFreq = 12800;
const int pwmReso = 10;
int       pwmMv;  //pwmReso's Decimal value - 1 

//// Function Declarations ////

void forward(int duty_r, int duty_l)
{
  ledcWrite(pwm_Pin_rightMotorIn1, 0);
  ledcWrite(pwm_Pin_rightMotorIn2, duty_r);
  ledcWrite(pwm_Pin_leftMotorIn1, 0);
  ledcWrite(pwm_Pin_leftMotorIn2, duty_l);
}
void backward(int duty_r, int duty_l)
{
  ledcWrite(pwm_Pin_rightMotorIn1, duty_r);
  ledcWrite(pwm_Pin_rightMotorIn2, 0);
  ledcWrite(pwm_Pin_leftMotorIn1, duty_l);
  ledcWrite(pwm_Pin_leftMotorIn2, 0);
}

void left_turn(int duty_r, int duty_l)
{
  ledcWrite(pwm_Pin_rightMotorIn1, 0);
  ledcWrite(pwm_Pin_rightMotorIn2, duty_r);
  ledcWrite(pwm_Pin_leftMotorIn1, duty_l);
  ledcWrite(pwm_Pin_leftMotorIn2, 0);
}

void right_turn(int duty_r, int duty_l)
{
  ledcWrite(pwm_Pin_rightMotorIn1, duty_r);
  ledcWrite(pwm_Pin_rightMotorIn2, 0);
  ledcWrite(pwm_Pin_leftMotorIn1, 0);
  ledcWrite(pwm_Pin_leftMotorIn2, duty_l);
}

void free(int time_ms)
{
  ledcWrite(pwm_Pin_rightMotorIn1, 0);
  ledcWrite(pwm_Pin_rightMotorIn2, 0);
  ledcWrite(pwm_Pin_leftMotorIn1, 0);
  ledcWrite(pwm_Pin_leftMotorIn2, 0);
  delay(time_ms);  
}

void brake(int time_ms)
{
  ledcWrite(pwm_Pin_rightMotorIn1, pwmMv);
  ledcWrite(pwm_Pin_rightMotorIn2, pwmMv);
  ledcWrite(pwm_Pin_leftMotorIn1, pwmMv);
  ledcWrite(pwm_Pin_leftMotorIn2, pwmMv);
  delay(time_ms);  
}

void motorTest()
{
  Serial.println("motorTest");
  
  //左右（白白）→直進
  forward(pwmMv, pwmMv);
  delay(500);
  brake(250);
  Serial.println("finished forward");

  //左右（白白）→後進
  backward(pwmMv, pwmMv);
  delay(500);
  brake(250);  
  Serial.println("finished backward");

  //左右（黒白）→右旋回（左モータ正転）
  right_turn(pwmMv, pwmMv);
  delay(500);
  brake(250);  
  Serial.println("finished right_turn");

  //左右（白黒）→左旋回（右モータ正転）
  left_turn(pwmMv, pwmMv);
  delay(500);
  brake(250);  
  Serial.println("finished left_turn");
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Start Serial communication");
  SerialBT.begin("lixx ESP32 Ver."); //Bluetooth device name
  Serial.println("SerialBT begins");
  SerialBT.println("SerialBT begins");
  mpu6050_setup();

  pinMode(Pin_leftMotorIn1, OUTPUT);
  pinMode(Pin_leftMotorIn2, OUTPUT);
  pinMode(Pin_rightMotorIn1, OUTPUT);
  pinMode(Pin_rightMotorIn2, OUTPUT);

  ledcSetup(0, pwmFreq, pwmReso); //0ch, 12,8kHz, 10bit(1024)Resolution
  ledcSetup(1, pwmFreq, pwmReso); //1ch, 12,8kHz, 10bit(1024)Resolution
  ledcSetup(2, pwmFreq, pwmReso); //2ch, 12,8kHz, 10bit(1024)Resolution
  ledcSetup(3, pwmFreq, pwmReso); //3ch, 12,8kHz, 10bit(1024)Resolution
  ledcAttachPin(Pin_leftMotorIn1, 0);  //0ch setting
  ledcAttachPin(Pin_leftMotorIn2, 1);  //1ch setting
  ledcAttachPin(Pin_rightMotorIn1, 2);  //2ch setting
  ledcAttachPin(Pin_rightMotorIn2, 3);  //3ch setting

  pwmMv = pow(2, pwmReso)-1;  //pwmResoの10進数最大値を計算、2のpwmReso乗
  Serial.print("pwmFreq = ");
  Serial.println(pwmFreq);
  Serial.print("pwmReso = ");
  Serial.println(pwmReso);
  Serial.print("pwmMv = ");
  Serial.println(pwmMv);

  motorTest();
}

void loop()
{
  //Getting PhotoReflectors Value 
  int analogValueL = analogRead(Pin_leftSensor);  //adcから値を取得
  int analogValueR = analogRead(Pin_rightSensor); //adcから値を取得

  int Pgain_forward = 100;
  int Pgain_backward = 100;
  int Dgain_forward = 100;
  int Dgain_backward = 100;
  int Pgain=100, Dgain=100, Igain=100;
  
  int motor_duty;
  int stand_angle = 90;
  static int prev_angleY; //微分に使用する
  static int diff_angleY;  //微分値

  //MPU6050
  calcRotation();
  angleY = angleY * (-1);  //マシンへのセンサ取付角度から符号を反転
  Serial.print("angleY: ");
  Serial.println(angleY);
  SerialBT.print("angleY: ");
  SerialBT.println(angleY);

  diff_angleY = prev_angleY - angleY; //前回-今回の角度

  // PID制御の式より、制御入力uを計算
/*
  e  = r - y;                // 誤差を計算
  de = (e - e_pre)/T;        // 誤差の微分を近似計算
  ie = ie + (e + e_pre)*T/2; // 誤差の積分を近似計算
  u  = KP*e + KI*ie + KD*de; // PID制御の式にそれぞれを代入
*/

/*
//#indev
// 定数設定
  int KP = 100;  // Pゲイン
  int KD = 100;   // Dゲイン
  int KI = 100;   // Iゲイン
  int T = 0.01;// 制御周期[秒]
  int e, de, u;
  int e_pre = 0;
  int ie = 0;

  e  = stand_angle - angleY; // 誤差を計算
  de = (e - e_pre)/T;        // 誤差の微分を近似計算
  u  = KP*e + KD*de; // PD制御の式にそれぞれを代入
  Serial.print("u : ");
  Serial.println(u);
*/

//  ie = ie + (e + e_pre)*T/2; // 誤差の積分を近似計算
//  u  = KP*e + KI*ie + KD*de; // PID制御の式にそれぞれを代入


/*
  if(angleY < stand_angle)  //前に傾いているとき
  {
    motor_duty = (stand_angle - angleY) * KP; //duty計算
    if(motor_duty >= pwmMv)
    {
      motor_duty = pwmMv;
    }
    forward(motor_duty, motor_duty);
    Serial.print("forward motor_duty : ");
    Serial.println(motor_duty);
    SerialBT.print("forward motor_duty : ");
    SerialBT.println(motor_duty);
  }
  else if(angleY > stand_angle) //後ろに傾いているとき
  {    
    motor_duty = (angleY - stand_angle) * KP;
    if(motor_duty >= pwmMv)
    {
      motor_duty = pwmMv;
    }
    backward(motor_duty, motor_duty);
    Serial.print("backward motor_duty : ");
    Serial.println(motor_duty);
    SerialBT.print("backward motor_duty : ");
    SerialBT.println(motor_duty);
  }else{
    Serial.println("non");
    SerialBT.println("non");
  }
  
  Serial.println("");
  SerialBT.println("");

  //前回の角度を保存
  prev_angleY = angleY; 
  
  // 次のために現時刻の情報を記録
  e_pre = e;
  */

/*
  //PhotoReflectoro
  Serial.print("left_PhotoReflector: ");
  Serial.println(analogValueL);
  Serial.print("right_PhotoReflector: ");
  Serial.println(analogValueR);
  Serial.println();
*/
}
