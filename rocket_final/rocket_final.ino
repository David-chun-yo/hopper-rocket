#include <ServoTimer2.h>
#include <SCoop.h>
#include <VirtualWire.h>
#include <PID_v1.h>

defineTask(mympu6050);   //mpu6050
defineTask(rx);      //接收資料
defineTask(fuel);  //燃料
defineTask(tvc);   //方向

//********************************************//變數
long button;
double  setxangle, setyangle ;//搖桿偵測到的角度（我要的）
double  realxangle,realyangle;    //mpu6050偵測到的角度
double  xangle,yangle;    //servo要轉的的角度
float t;    //燃料停止時間

//*****************************servo set
ServoTimer2 X1servo;
ServoTimer2 Y1servo;

//*****************************pid set

//Specify the links and initial tuning parameters
double Kpx=0.54 , Kix=0.31, Kdx=0.23;
double Kpy=0.72, Kiy=0.5, Kdy=0.26;


//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID PIDx(&realxangle, &xangle, &setxangle, Kpx, Kix, Kdx, DIRECT);
PID PIDy(&realyangle, &yangle, &setyangle, Kpy, Kiy, Kdy, DIRECT);

//*****************************mpu6050 set   (不要改)
#include <Wire.h>
#include <ADXL345.h>

ADXL345 accelerometer;

//===================================================================================================================
//***************************************************************************///mpu6050
void mympu6050::setup() 
{
Serial.begin(115200);

  // Initialize ADXL345
  Serial.println("Initialize ADXL345");

  if (!accelerometer.begin())
  {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  // +/-  2G: ADXL345_RANGE_2G
  // +/-  4G: ADXL345_RANGE_4G
  // +/-  8G: ADXL345_RANGE_8G
  // +/- 16G: ADXL345_RANGE_16G
  accelerometer.setRange(ADXL345_RANGE_16G);
}
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
void mympu6050::loop()
{ 
  
 // Read normalized values
  Vector norm = accelerometer.readNormalize();

  // Low Pass Filter to smooth out data. 0.1 - 0.9
  Vector filtered = accelerometer.lowPassFilter(norm, 0.5);

  // Calculate Pitch & Roll
  int pitch = -(atan2(norm.XAxis, sqrt(norm.YAxis*norm.YAxis + norm.ZAxis*norm.ZAxis))*180.0)/M_PI;
  int roll  = (atan2(norm.YAxis, norm.ZAxis)*180.0)/M_PI;

  // Calculate Pitch & Roll (Low Pass Filter)
  int fpitch = -(atan2(filtered.XAxis, sqrt(filtered.YAxis*filtered.YAxis + filtered.ZAxis*filtered.ZAxis))*180.0)/M_PI;
  int froll  = (atan2(filtered.YAxis, filtered.ZAxis)*180.0)/M_PI;

  // Output
//  Serial.print(" Pitch = ");
//  Serial.print(pitch);
//  Serial.print(" Roll = ");
//  Serial.print(roll+10);

  // Output (filtered)
  realxangle=fpitch-2;
  realyangle=froll+10;
//  Serial.print(" (filter)Pitch = ");
//  Serial.print(fpitch-2);
//  Serial.print(" (filter)Roll = ");
//  Serial.print(froll+10);
//  Serial.println();

  delay(100);
   
}

//***************************************************************************///接收資料
void rx::setup(){
  Serial.begin(115200);
  vw_setup(2000);
  vw_rx_start();
  button=1;
  Serial.print(" n  ");
}
//-- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
void rx::loop (){
  vw_rx_start();
  int valeurs[4];
  byte message = sizeof(valeurs);
 vw_wait_rx();
  if (vw_get_message((byte *) &valeurs, &message)){
    Serial.print(" y ");
    setxangle=valeurs[1];      //x方向角度
    setyangle=valeurs[0];      //y方向角度
    t=(map(valeurs[2], 0, 1023, 0, 50));     //燃料停止時間(搖桿推到最上面為0   ，將搖桿數值對應"停止"時間)
    button=valeurs[3];                      //button
    sleep(10);
  vw_rx_stop() ;
  }
Serial.print(" m ");
vw_rx_stop() ;
}

//*****************************************************************************/ //燃料

void fuel::setup()
{
  Serial.begin(115200);
  pinMode(8, OUTPUT);   //停機指示燈
  pinMode(7, OUTPUT);  //h2o2
  digitalWrite(8, LOW); 
}
void(* resetFunc) (void) = 0;                  //declare reset function @ address 0//重設程式
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
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
  
    

//***********************************************************************************/ //方向

void tvc::setup() {

 X1servo.attach(6);
Y1servo.attach(5);

//  Serial.begin(115200);   


//turn the PID on
PIDx.SetOutputLimits(-90, 90);
PIDy.SetOutputLimits(-90, 90);
  PIDx.SetMode(AUTOMATIC);     
  PIDy.SetMode(AUTOMATIC); 
  
}
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
void tvc::loop() {
  
 while(true){
    PIDx.Compute();
    PIDy.Compute();   
        //方向主程式
        //Serial.println("ok");
        xangle=xangle+55.5;
        yangle=yangle+36;
        
//        Serial.print(setxangle);
//        Serial.print("    ");
//        Serial.print(setyangle);
//        Serial.print("    ");
//        Serial.print(xangle);
//        Serial.print("    ");
//        Serial.println(yangle);
        
    X1servo.write(map(xangle, 0,180, 500, 2400));           //x軸搖桿（要用脈波寬度）
    Y1servo.write(map(yangle, 0, 180, 500, 2400));            //y軸搖桿（要用脈波寬度）

    sleep(50);                       // wait for a second
   
  
 }
 
}
//========================================================================================================
//***************************************************************************/ //執行scoop

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
