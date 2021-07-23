 #include "PS2X_lib.h"
#include <Servo.h>
#include <SCoop.h>

defineTask(fuel);//燃料
defineTask(tvc);//方向

Servo X1servo;
Servo X2servo;

Servo Y1servo;
Servo Y2servo;

PS2X ps2x; 
int error;
int t;    //燃料停止時間
//******************************************************************************************//燃料

void fuel::setup()
{
  Serial.begin(9600);
  pinMode(8, OUTPUT);   //停機指示燈
  pinMode(7, OUTPUT);  //h2o2

  do { 
    //GamePad(clock, command, attention, data, Pressures?, Rumble?)
    error = ps2x.config_gamepad(13, 11, 10, 12, true, true);   //這行要和接線對應正確
    if (error == 0) { 
      Serial.println("Gamepad found!");
      break; 
      } 
    else { sleep(100); } 
  } while (1); 
}
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -   
void fuel::loop() {
  ps2x.read_gamepad(false, 0);  //讀取手把狀態
   if(ps2x.Button(PSB_START))  {             //Start鍵開機
   
  while(true){
    ps2x.read_gamepad(false, 0);  //讀取手把狀態 
    t=(map(ps2x.Analog(PSS_LY), 0, 255, 0, 50));   //搖桿推到最上面為0
    Serial.println(t);
    //燃料主程式
      digitalWrite(7, HIGH);  //開0.2秒
      sleep(200);
      digitalWrite(7,LOW);   //關t秒
      sleep(t);
     
     if(ps2x.NewButtonState(PSB_CROSS)){       //停機(要一直狂按X)
      Serial.println("stop");
      break;
     }
  }
  while(true){    //停機後要做的
        digitalWrite(7,LOW);    // 關閉燃料
       digitalWrite(8, HIGH);   // turn the LED on (HIGH is the voltage level)
     }

}
sleep(10);
}

//**************************************************************************************************//方向

void tvc::setup() {
 X1servo.attach(9);
 X2servo.attach(6);
 
Y1servo.attach(3);
Y2servo.attach(5);
 
  
  Serial.begin(9600);              
  
  do { 
    //GamePad(clock, command, attention, data, Pressures?, Rumble?)
    error = ps2x.config_gamepad(13, 11, 10, 12, true, true);   //這行要和接線對應正確
    if (error == 0) { 
      Serial.print("Gamepad found!");
      break; 
      } 
    else { sleep(100); } 
  } while (1); 
}
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
void tvc::loop() {
  ps2x.read_gamepad(false, 0);  //讀取手把狀態 
  while(true){
    ps2x.read_gamepad(false, 0);  //讀取手把狀態 
    //方向主程式
     //Serial.println("ok");
X1servo.write(map(ps2x.Analog(PSS_RX), 0, 255, 0, 111));
X2servo.write(map(ps2x.Analog(PSS_RX), 0, 255, 0, 111));

Y1servo.write(map(ps2x.Analog(PSS_RY), 0, 255, 0, 72)); 
Y2servo.write(map(ps2x.Analog(PSS_RY), 0, 255, 0, 72));

Serial.print(map(ps2x.Analog(PSS_RX), 0, 255, 0, 111));
Serial.print("  ");
Serial.println(map(ps2x.Analog(PSS_RY), 0, 255, 0, 72));
sleep(50);                       // wait for a second
     
   
     sleep(10);
  }
  
}

//***************************************************************************執行scoop

void setup() {
  mySCoop.start();
}

void loop()
{
  yield();
}
