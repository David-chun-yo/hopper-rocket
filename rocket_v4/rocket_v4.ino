#include <ServoTimer2.h>
#include <SCoop.h>
#include <VirtualWire.h>
#include <PID_v1.h>

defineTask(mympu6050);   //mpu6050
defineTask(rx);      //接收資料
defineTask(fuel);  //燃料
defineTask(tvc);   //方向


ServoTimer2 X1servo;
ServoTimer2 Y1servo;

//*****************************mpu6050 set
#include "Wire.h"
 #include <MPU6050_light.h>
 MPU6050 mpu(Wire);
 unsigned long timer = 0;
 
//********************************************
long  xangle, yangle,button ;
int t;    //燃料停止時間

//***************************************************************************///mpu6050
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
  Serial.print(timer);
     Serial.print("X : ");
     Serial.print(mpu.getAngleX());
     Serial.print("\tY : ");
     Serial.print(mpu.getAngleY());
     Serial.print("\tZ : ");
     Serial.println(mpu.getAngleZ());
     timer = millis();
   }
}

//******************************************************************************************///接收資料
void rx::setup(){
  //Serial.begin(9600);
  vw_setup(2000);
  vw_rx_start();
  button=1;
}

void rx::loop (){
  vw_rx_start();
  int valeurs[4];
  byte message = sizeof(valeurs);
  vw_wait_rx();
  if (vw_get_message((byte *) &valeurs, &message)){
    xangle=valeurs[1];      //x方向角度
    yangle=valeurs[0];      //y方向角度
    t=(map(valeurs[2], 0, 1023, 0, 50));     //燃料停止時間(搖桿推到最上面為0   ，將搖桿數值對應"停止"時間)
    button=valeurs[3];                      //button
    sleep(10);
    vw_rx_stop() ;
  }
}

//******************************************************************************************/ //燃料

void fuel::setup()
{
  Serial.begin(115200);
  pinMode(8, OUTPUT);   //停機指示燈
  pinMode(7, OUTPUT);  //h2o2
  digitalWrite(8, LOW); 
}
void(* resetFunc) (void) = 0;                  //declare reset function @ address 0//重設程式
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -   
void fuel::loop() {
  
 if(button==0)  {             //Start button 開機
  sleep(1000);
  while(true){                                                          //- - - - - - - - - - - - - - 燃料主程式
    
  //  Serial.println("start");
    //燃料主程式
      digitalWrite(7, HIGH);  //開0.2秒
      sleep(200);
      digitalWrite(7,LOW);   //關t秒
      sleep(t);
           
     if(button==0){       //停機(要一直按著)
      Serial.println("stop");
      while(true){    //停機後要做的
        digitalWrite(7,LOW);    // 關閉燃料
       digitalWrite(8, HIGH);   // turn the LED on (HIGH is the voltage level)
       sleep(2000);
         resetFunc();  //call reset
     }
    }

  }
}    
  }
  
    

//**************************************************************************************************/ //方向

void tvc::setup() {

 X1servo.attach(6);
Y1servo.attach(3);

//  Serial.begin(9600);              
 
}
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
void tvc::loop() {
  
 while(true){
       
        //方向主程式
        //Serial.println("ok");
//        Serial.print(xangle);
//        Serial.print("    ");
//        Serial.println(yangle);
    X1servo.write(map(xangle, -15, 15, 500, 1672));           //x軸搖桿（要用脈波寬度）
    Y1servo.write(map(yangle, -15, 15, 500, 1260));            //y軸搖桿（要用脈波寬度）

    sleep(50);                       // wait for a second
   
  
 }
 
}




//***************************************************************************///執行scoop

void setup() {
  Serial.begin(115200);
  vw_setup(2000);
  vw_rx_start();
  button=1;
  mySCoop.start();
}

void loop()
{
  
  yield();
}
