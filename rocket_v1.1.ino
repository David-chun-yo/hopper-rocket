 #include "PS2X_lib.h"
#include <Servo.h>
#include <SCoop.h>

defineTask(mympu6050);   //mpu6050
defineTask(fuel);//燃料
defineTask(tvc);//方向
defineTask(rx);//方向
//*****************************mpu6050 set
#include "Wire.h"
 #include <MPU6050_light.h>
 MPU6050 mpu(Wire);
 unsigned long timer = 0;
 
//********************************************

Servo X1servo;
Servo Y1servo;
//**********************************************ps2
PS2X ps2x; 
int error;
//*****************************************
int t;    //燃料停止時間
long  xangle, yangle;
bool startbutton,stopbutton ;
//*********************************************************
void(* resetFunc) (void) = 0;                  //declare reset function @ address 0//重設程式
//******************************************************************************************///mpu6050
void mympu6050::setup() 
{
    Serial.begin(115200);
   Wire.begin();
 byte status = mpu.begin();
   Serial.print(F("MPU6050 status: "));
   Serial.println(status);
   while (status != 0) { } // stop everything if could not connect to MPU6050
 Serial.println(F("Calculating offsets, do not move MPU6050"));
   delay(1000);
   mpu.calcOffsets(); // gyro and accelero
   Serial.println("Done!\n");
}

void mympu6050::loop()
{
  mpu.update();
 if ((millis() - timer) > 10) { // print data every 10ms
//  Serial.print(timer);
//     Serial.print("X : ");
//     Serial.print(mpu.getAngleX());
//     Serial.print("\tY : ");
//     Serial.print(mpu.getAngleY());
//     Serial.print("\tZ : ");
//     Serial.println(mpu.getAngleZ());
     timer = millis();
   }
}

//******************************************************************************************///rx
void rx::setup(){
    Serial.begin(115200);              
  
  do { 
    //GamePad(clock, command, attention, data, Pressures?, Rumble?)
    error = ps2x.config_gamepad(13, 11, 10, 12, true, true);   //這行要和接線對應正確
    if (error == 0) { 
      Serial.print("Gamepad found!");
      break; 
      } 
    else { sleep(100); } 
  } while (1); 

  startbutton=false;
  stopbutton=false;
}
//*****************************
void rx::loop(){
  ps2x.read_gamepad(false, 0);  //讀取手把狀態
  if(ps2x.Button(PSB_START)>1000)  {        //Start鍵開機
    Serial.print(ps2x.Button(PSB_START));
    ps2x.Button("  ");
    
    startbutton=true;
  }

  if(ps2x.NewButtonState(PSB_CROSS)>100){       //停機
     Serial.print(ps2x.NewButtonState(PSB_CROSS));
      stopbutton=true;
      startbutton=false;
     }
     
 t=(map(ps2x.Analog(PSS_LY), 0, 255, 0, 50));   //搖桿推到最上面為0
xangle=map(ps2x.Analog(PSS_RX), 0, 255, 0, 111);
yangle=map(ps2x.Analog(PSS_RY), 0, 255, 0, 72);

}

//******************************************************************************************///燃料

void fuel::setup()
{
  Serial.begin(115200);
  pinMode(8, OUTPUT);   //停機指示燈
  pinMode(7, OUTPUT);  //h2o2
startbutton=false;
}
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -   
void fuel::loop() {
 
   
  while(startbutton=true){                                  //Start鍵開機
    
    Serial.println(t);
    //燃料主程式
      digitalWrite(7, HIGH);  //開0.2秒
      sleep(200);
      digitalWrite(7,LOW);   //關t秒
      sleep(t);
//     Serial.print(startbutton);
//    Serial.print("  ");
//     Serial.print(stopbutton);
  }
  while(stopbutton=true){    //停機後要做的
        digitalWrite(7,LOW);    // 關閉燃料
       digitalWrite(8, HIGH);   // turn the LED on (HIGH is the voltage level)
       sleep(2000);
       resetFunc();  //call reset
     }


sleep(10);
}

//**************************************************************************************************//方向

void tvc::setup() {
 
 X1servo.attach(6);
 
Y1servo.attach(3);  
Serial.begin(115200);                
}
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - ///

void tvc::loop() {
  
  
    //方向主程式
     //Serial.println("ok");
X1servo.write(xangle);
Y1servo.write(yangle); 

//Serial.print(xangle);
//Serial.print("  ");
//Serial.println(yangle);
sleep(50);                       // wait for a second   
sleep(10);
  
  
}

//***************************************************************************///執行scoop

void setup() {
  mySCoop.start();
}

void loop()
{
  yield();
}
