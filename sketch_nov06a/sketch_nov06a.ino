#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h> 
#include <SoftwareSerial.h>
#define RST_PIN  49      
#define SS_PIN  53
#define rightmotorspwm 8
#define rightmotorselection1 23
#define rightmotorselection2 22
#define leftmotorspwm 9
#define leftmotorselection1 24
#define leftmotorselection2 25
#define startengine 33
#define seatbelt 35
#define seatbeltbuzzer 31
#define lanebuzzer 29
#define fuel A0
#define rain A1
#define A 11
#define v0 10
#define joystick A2
#define servo 12
#define lockedunlocked 48


const int rs = 28, en = 26, d4 = 42, d5 = 40, d6 = 38, d7 = 36;
int i=0;
int RFstate=0;
int RFoldstate=0;
int LFstate=0;
int LFoldstate=0;

int seatbeltstate=0;
int seatbeltbuzzerstate=0;
int lanebuzzerstate=0;
int mainCardUID []={215,124,5,52};
int fuellevel=0;
int raining=0;
double fuelpercentage=0.0;
boolean rainingstate=0;
int servovalue=90;
int motorSpeedright = 0;
int motorSpeedleft = 0;
int xAxis = 512;
int yAxis = 512;
int  x = 127;
int  y = 127;

SemaphoreHandle_t lock;
SemaphoreHandle_t engine_Sem;
SemaphoreHandle_t STOP;
TaskHandle_t task1handle=NULL;

void Task1( void *pvParameters);
void Task2( void *pvParameters);
void Task3( void *pvParameters);
void Task4( void *pvParameters);

MFRC522 mfrc522(SS_PIN, RST_PIN);
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
Servo myservo;
SoftwareSerial mySerial(13, 14); // RX, TX

void setup() {
  mySerial.begin(9600);
  //DC motors
  pinMode(leftmotorselection1,OUTPUT);//Left wheels Selection 
  pinMode(leftmotorselection2,OUTPUT);
  pinMode(rightmotorselection1,OUTPUT);//Right wheels Selection
  pinMode(rightmotorselection2,OUTPUT);//
  pinMode(leftmotorspwm,OUTPUT); //left wheels pwm signal
  pinMode(rightmotorspwm,OUTPUT); //right wheels pwm signal
  // Line Sensor
  pinMode(47,INPUT);//Right-back
  pinMode(45,INPUT);//Right-middle
  pinMode(43,INPUT);//Right-front
  pinMode(41,INPUT);//Left-back
  pinMode(39,INPUT);//Left-middle
  pinMode(37,INPUT);//Left-front
  //Buttons & Buzzer & LED
  pinMode(startengine, INPUT);//Start Engine
  pinMode(seatbelt, INPUT);//Seat Belt
  pinMode(seatbeltbuzzer, OUTPUT);//Buzzer
  pinMode(lanebuzzer, OUTPUT);//Buzzer
  pinMode(lockedunlocked,OUTPUT); 
  //RFID
  SPI.begin();      // Init SPI bus
  mfrc522.PCD_Init();   // Init MFRC522
  delay(4);
  //fuel & rain
  pinMode(fuel,INPUT);
  pinMode(rain,INPUT);
  //lcd
  analogWrite(v0,100);
  analogWrite(A,180);
  lcd.begin(16,2);
  lcd.write("LOCKED");
  //servo and joystick
  myservo.attach(servo);
  pinMode(joystick,INPUT);

  
  lock=xSemaphoreCreateBinary();
  engine_Sem=xSemaphoreCreateBinary();
  STOP=xSemaphoreCreateBinary();
  xSemaphoreGive(STOP);
  
 xTaskCreate(Task1,"Task1",4000,NULL,10,&task1handle);
 xTaskCreate(Task2,"Task2",1000,NULL,9,NULL);
 xTaskCreate(Task3,"Task3",1000,NULL,8,NULL);
 xTaskCreate(Task4,"Task4",500,NULL,7,NULL);
}

void loop() {
 
//  //Serial.println(fuelpercentage); 
//  

//  

//
//////forward
////analogWrite(leftmotorspwm,255);
////analogWrite(rightmotorspwm,255);
////digitalWrite(leftmotorselection1,HIGH);
////digitalWrite(leftmotorselection2,LOW);
////digitalWrite(rightmotorselection1,HIGH);
////digitalWrite(rightmotorselection2,LOW);
////delay(1000);
////
////
//////backward
////analogWrite(leftmotorspwm,255);
////analogWrite(rightmotorspwm,255);
////digitalWrite(leftmotorselection1,LOW);
////digitalWrite(leftmotorselection2,HIGH);
////digitalWrite(rightmotorselection1,LOW);
////digitalWrite(rightmotorselection2,HIGH);
////delay(1000);
////
//////left
////analogWrite(leftmotorspwm,255);
////analogWrite(rightmotorspwm,0);
////digitalWrite(leftmotorselection1,HIGH);
////digitalWrite(leftmotorselection2,LOW);
////digitalWrite(rightmotorselection1,LOW);
////digitalWrite(rightmotorselection2,LOW);
////delay(1000);
//////right
////analogWrite(leftmotorspwm,0);
////analogWrite(rightmotorspwm,255);
////digitalWrite(leftmotorselection1,LOW);
////digitalWrite(leftmotorselection2,LOW);
////digitalWrite(rightmotorselection1,HIGH);
////digitalWrite(rightmotorselection2,LOW);
////delay(1000);
//

}
void Task1(void * pvparameters){
  int carstate=0; //0 Locked 1 is unlocked
  TickType_t xLastWakeTime=xTaskGetTickCount();
  const TickType_t xDelays=pdMS_TO_TICKS(300);
  while(1)
  {
    xSemaphoreTake(STOP,portMAX_DELAY);
    xSemaphoreGive(STOP);
    if (mfrc522.PICC_IsNewCardPresent()&&mfrc522.PICC_ReadCardSerial())
    {
      boolean check=true;
      for (byte i = 0; i < mfrc522.uid.size; i++) 
      {
       if(!(mainCardUID[i]== mfrc522.uid.uidByte[i]))
       {
        check=false;
        break;
       }
      }
      carstate=(check ==true)?!carstate:carstate;

      if(carstate==1&&check==true){
        xSemaphoreGive(lock);       
      }
      else if (carstate==0&&check==true){
        xSemaphoreGive(lock);
        xSemaphoreTake(lock,portMAX_DELAY);
        lcd.clear();
        lcd.write("LOCKED");
      }
      
      }
      digitalWrite(lockedunlocked,carstate); 
   
  
    
    if(carstate==0){
        while(mySerial.available()>0){
          mySerial.read();
        }
    }
    vTaskDelayUntil(&xLastWakeTime,xDelays);
    
  }
}
void Task2(void * pvparameters){
  int startenginestate=0;
  int oldstartenginestate=0;
  TickType_t xLastWakeTime=xTaskGetTickCount();
  const TickType_t xDelays=pdMS_TO_TICKS(400);
  while(1)
  { 
    xSemaphoreGive(engine_Sem);
    xSemaphoreTake(engine_Sem,portMAX_DELAY);
    xSemaphoreTake(lock,portMAX_DELAY);
    xSemaphoreGive(lock);
    startenginestate=digitalRead(startengine);
    
    if(startenginestate==1){
      xSemaphoreGive(engine_Sem);  
    }
    if(startenginestate==0){
      lcd.clear();
      lcd.write("START ENGINE");
      while(mySerial.available()>0){
        mySerial.read();
      }
      x=127;
      y=127;
      analogWrite(leftmotorspwm, 0); // Send PWM signal to motor A
      analogWrite(rightmotorspwm, 0); // Send PWM signal to motor B
      lanebuzzerstate=0;
      digitalWrite(lanebuzzer,lanebuzzerstate);
      digitalWrite(7,lanebuzzerstate);
      digitalWrite(seatbeltbuzzer,0);
        
    }
    if(oldstartenginestate==1&&startenginestate==0){
      xSemaphoreGive(STOP);
      vTaskPrioritySet(task1handle,10);
      
    }
    if(oldstartenginestate==0&&startenginestate==1){
      xSemaphoreGive(STOP);
      xSemaphoreTake(STOP,portMAX_DELAY);
      vTaskPrioritySet(task1handle,5);
      
    }
    oldstartenginestate=startenginestate;   
    vTaskDelayUntil(&xLastWakeTime,xDelays);  
  }
}
void Task3(void * pvparameters){
  TickType_t xLastWakeTime=xTaskGetTickCount();
  const TickType_t xDelays=pdMS_TO_TICKS(400);
  while(1)
  {
    xSemaphoreTake(engine_Sem,portMAX_DELAY);
    xSemaphoreGive(engine_Sem);
    while (mySerial.available() >= 2) {
      x=mySerial.read();
      y=mySerial.read();
    }
  
    xAxis = map(x,0,255,0,1023);
    yAxis = map(y,0,255,0,1023);
  
  // Y-axis used for forward and backward control
  if (yAxis < 512) {
    // Set Motor A backward
    digitalWrite(rightmotorselection1, HIGH);
    digitalWrite(rightmotorselection2, LOW);
    // Set Motor B backward
    digitalWrite(leftmotorselection1, HIGH);
    digitalWrite(leftmotorselection2, LOW);
    // Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedleft = map(yAxis, 510, 0, 0, 255);
    motorSpeedright = map(yAxis, 510, 0, 0, 255);
  }
  else if (yAxis > 512) {
    // Set Motor A forward
    digitalWrite(rightmotorselection1, LOW);
    digitalWrite(rightmotorselection2, HIGH);
    // Set Motor B forward
    digitalWrite(leftmotorselection1, LOW);
    digitalWrite(leftmotorselection2, HIGH);
    // Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedleft = map(yAxis, 514, 1023, 0, 255);
    motorSpeedright = map(yAxis, 514, 1023, 0, 255);
  }
  // If joystick stays in middle the motors are not moving
  else {
    motorSpeedleft = 0;
    motorSpeedright = 0;
  }
  // X-axis used for left and right control
  if (xAxis < 512) {
    // Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value
    int xMapped = map(xAxis, 512, 0, 0, 255);
    // Move to left - decrease left motor speed, increase right motor speed
    motorSpeedleft = motorSpeedleft - xMapped;
    motorSpeedright = motorSpeedright + xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedleft < 0) {
      motorSpeedleft = 0;
    }
    if (motorSpeedright > 255) {
      motorSpeedright = 255;
    }
  }
  if (xAxis > 512) {
    // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value
    int xMapped = map(xAxis, 550, 1023, 0, 255);
    // Move right - decrease right motor speed, increase left motor speed
    motorSpeedleft = motorSpeedleft + xMapped;
    motorSpeedright = motorSpeedright - xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedleft > 255) {
      motorSpeedleft = 255;
    }
    if (motorSpeedright < 0) {
      motorSpeedright = 0;
    }
  }
  // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
  if (motorSpeedleft < 90) {
    motorSpeedleft = 0;
  }
  if (motorSpeedright < 90) {
    motorSpeedright = 0;
  }
  //Serial.print("speed left : ");
  //Serial.println(motorSpeedleft);
  //Serial.print("speed righ : ");
  //Serial.println(motorSpeedright);
  analogWrite(leftmotorspwm, motorSpeedleft); // Send PWM signal to motor A
  analogWrite(rightmotorspwm, motorSpeedright); // Send PWM signal to motor B
    
      RFstate=digitalRead(43);
      LFstate=digitalRead(37);
      if((RFoldstate==0&&RFstate==1)||(LFoldstate==0&&LFstate==1)){
        lanebuzzerstate=!lanebuzzerstate;
        digitalWrite(lanebuzzer,lanebuzzerstate);
        digitalWrite(7,lanebuzzerstate);
        
      }
      RFoldstate=RFstate;
      LFoldstate=LFstate;
      
   vTaskDelayUntil(&xLastWakeTime,xDelays);  
  }
}
void Task4(void * pvparameters){
  TickType_t xLastWakeTime=xTaskGetTickCount();
  const TickType_t xDelays=pdMS_TO_TICKS(400);
  while(1)
  {
    xSemaphoreTake(engine_Sem,portMAX_DELAY);
    xSemaphoreGive(engine_Sem);  
    seatbeltstate=digitalRead(seatbelt);
    seatbeltbuzzerstate= !seatbeltstate;
    digitalWrite(seatbeltbuzzer,seatbeltbuzzerstate);
    //Serial.println(seatbeltstate);
    int joyval=analogRead(joystick);
  
    if(joyval<500)
      servovalue-=10;
    if(joyval >600)
      servovalue+=10;
    servovalue=servovalue<0?0:(servovalue>90?90:servovalue);
    myservo.write(servovalue);

   

      
    
  
  
  raining=analogRead(rain);
  rainingstate=raining <600?1:0;
  fuellevel=analogRead(fuel);
   if(fuellevel>=360){
    fuelpercentage=100.0-(380.0-fuellevel)*(50.0/20.0);
       
  }
  else if(fuellevel<360&&fuellevel>=280){
    fuelpercentage=50.0-(360.0-fuellevel)*(50.0/80.0);
  }

  else{
    fuelpercentage=0; 
  }
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.write("fuel %:");
    lcd.setCursor(8,0);
   // Serial.println(fuelpercentage);
   
    lcd.print(fuelpercentage);
    
    lcd.setCursor(0,1);
    (rainingstate==1)?lcd.write("Raining"):lcd.write("Not Raining");
    vTaskDelayUntil(&xLastWakeTime,xDelays);  
  }
}
