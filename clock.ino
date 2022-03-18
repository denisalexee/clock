#include <Wire.h>
#include "SparkFunHTU21D.h"
#include "MAX30105.h"           // Библиотека пульсометра

#include <GyverOLED.h>          // Библиотека экрана
#include <Rtc_Pcf8563.h>        // Библиотека часов реального времени
#include "MPU6050.h"            // Библиотека гироскоп
#include "heartRate.h"
#include "spo2_algorithm.h"  
#include <FS.h>                // Доступ к файловой системе
#include <SPIFFS.h>

// bluetooth
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
//----------------bitmap--------------------------
#include "Still.h"
#include "batt.h"
#include "Bluetooth.h"
#include "daw_x.h"
#include "heart.h"
#include "mist_two.h"
#include "vibr.h"


HTU21D myHumidity;
GyverOLED<SSH1106_128x64> oled;
Rtc_Pcf8563 rtc;
File Data_array_file;
SemaphoreHandle_t btnSemaphore;
MAX30105 PARTICLE_SENSOR;
MPU6050 mpu;
BLEServer *pServer = NULL;

BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pTxTimeCharacteristic;
//BLECharacteristic *pTxminTempParCharacteristic;

bool deviceConnected = false;
bool oldDeviceConnected = false;


const int RES = 27;
/* -------clock---------=*/
String hm,dm;
String Time;
byte century =0;
byte day = 15;
byte week = 4;
byte month=7;
byte year = 21;
byte hour = 0;
byte minute = 0;
byte second = 0;
bool flagTime = 0;

// MPU6050
int16_t ax, ay, az;
int16_t gx, gy, gz;
long ACC = 0;
long GYR = 0;
long maxACC = 0;
long maxGYR = 0;

byte flagDisplay = 0;
unsigned long previousMillis = 0;
unsigned long interval = 25000;

String SizeF;

float previousTemperature = -100.0;
float temperature = 0;
float CPU = 0;
float humidity= 0;

uint32_t irBuffer[100];                             //  32-битный массив данных от сенсора со значениями от ИК-светодиода
uint32_t redBuffer[100];                            //  32-битный массив данных от сенсора со значениями от красного светодиода
int32_t bufferLength;                               //  длина буфера данных
int32_t spo2;                                       //  значение SpO2 (насыщенности крови кислородом)
//int8_t  validSPO2;                                  //  флаг валидности значений сенсора по SpO2
int32_t heartRate = 0;                                  //  значение ЧСС
//int8_t  validHeartRate;                             //  флаг валидности значений сенсора по ЧСС
const char * Data_arrayPath = "/data.txt";
String VR  ;
int temper ;
byte VL,pulse, metka  ;
uint32_t currentMillisAcc = 0 ;
int t = 100;
const long time_press = 1000;
unsigned long Button2_previousMillis = 0; 
int bounceTime_B2 = 10;                             // задержка для подавления дребезга
int doubleTime_B2 = 500;                            // время, в течение которого нажатия можно считать двойным
int i = 0;
long onTime_B2 = 0;                                 // переменная обработки временного интервала
long lastSwitchTime_B2 = 0;                         // переменная времени предыдущего переключения состояния
long onTime_B3 = 0;                                 // переменная обработки временного интервала
int bounceTime_B3 = 10;                             // задержка для подавления дребезга
int holdTime_B3 = 500;                              // время, в течение которого нажатие можно считать удержанием кнопки
int doubleTime_B3 = 500;                            // время, в течение которого нажатия можно считать двойным
int holdTimeSum_B3 = 0;  
int j = 0;
long lastSwitchTime_B3 = 0;                         // переменная времени предыдущего переключения состояния

int max_heartRate = 150;
int max_time = 150000;
int max_temp = 120;
int deltaTemp = 20;
int int_Par = 15000;
int minTempPar = 80.0;
byte disp = 2;
uint32_t intervalWriteFlash = 600000;
uint32_t currentMilliisFLash = 0;

/*    flags    */
byte flagACC=0;                                   //  флаг срабатывания акселерометра
byte otlDisp = 0;                                 //  флаг обновления дисплея
int8_t  validSPO2;                                  //  флаг валидности значений сенсора по SpO2
int8_t  validHeartRate;                             //  флаг валидности значений сенсора по ЧСС
bool action_Button1 = LOW;
bool state_BT = LOW;
bool Button2_pressed = LOW;
bool Button2_released = LOW; 
bool double_press = LOW;            
boolean lastReading = false;                      // флаг предыдущего состояния кнопки
boolean button_2_Single = false;                  // флаг состояния "краткое нажатие"
boolean button_2_Multi = false;                   // флаг состояния "двойное нажатие"
boolean last_Button2_pressed = false;             // флаг предыдущего состояния кнопки
boolean last_Button3_pressed = false;             // флаг предыдущего состояния кнопки
boolean button_3_Single = false;                  // флаг состояния "краткое нажатие"
boolean button_3_Multi = false;                   // флаг состояния "двойное нажатие"
boolean button_3_Hold = false;                    // флаг состояния "долгое нажатие"
bool state_BT3 = LOW;
bool action_Button4 = LOW;
bool Button3_pressed = LOW;
bool Button3_released = LOW;
bool flagPar = 0;
bool flagUpdatePar = 0;                           //флаг обновления значка парилки
bool flagButt2 = 0;                               //флаг для отображения параметров
bool flageState = 0;                              //флаг текущего измерения
bool flageSinxr = 0;
bool flagConn = 0;

#define INTERVAL_ACC 300000
#define TM_BUTTON 100                            // Минимальный таймаут между событиями нажатия кнопки
#define PIN_BUTTON1 34
#define PIN_BUTTON2 27
#define PIN_BUTTON3 12
#define PIN_BUTTON4 19
//#define PIN_INPUT 35
//#define PIN_OUTPUT 11

#define SERVICE_UUID                 "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_TIME_RX_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_TP_RX_UUID  "acd787aa-08f2-4e4f-9078-b97194585906"

#define CHARACTERISTIC_TIME_TX_UUID  "60171725-f8a0-41b9-809d-c2044db71d8f"
#define CHARACTERISTIC_TEMP_TX_UUID  "bcdea365-b767-48a9-aa06-57c19859c502"
#define CHARACTERISTIC_TP_TX_UUID  "669115d4-2f37-4678-8012-fe22376b8a49"


extern "C" {
uint8_t temprature_sens_read();
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
        Serial.println(rxValue.length());
        Serial.println("*********");
        if (rxValue.length() == 14){
          Time = rxValue.c_str();
          day = Time.substring(0,2).toInt();
          week = Time[2] - '0';
          month = Time.substring(3,5).toInt();
          century = Time[5] - '0';
          year = Time.substring(6,8).toInt();
          hour = Time.substring(8,10).toInt();
          minute = Time.substring(10,12).toInt();
          second = Time.substring(12,14).toInt();
          Serial.print("day: ");
          Serial.println(day);
          Serial.println("Week: ");
          Serial.println(week);
          flagTime = 1;
        }
        if (flagTime == 1)
        {
          rtc.setDate(day, week, month, century, year);
          rtc.setTime(hour, minute, second);
          flagTime =0;
        }
      }
    }
};

//class MyCallbacksTempPar: public BLECharacteristicCallbacks {
//    void onWrite(BLECharacteristic *pCharacteristic) {
//      std::string rx1Value = pCharacteristic->getValue();
//
//      if (rx1Value.length() > 0) {
//        Serial.println("*********");
//        Serial.print("Received Value: ");
//        for (int i = 0; i < rx1Value.length(); i++)
//          Serial.print(rx1Value[i]);
//
//        Serial.println();
//        Serial.println("*********");
//        Serial.println(rx1Value.length());
//        Serial.println("*********");
//        String vrem =rx1Value.c_str();
//        minTempPar = vrem.toInt();
//        
//      }
//    }
//};
// Display 128X64 



int NamePict[8][6] = {
                     {104, 0, 23, 10,BITMAP_NORMAL, BUF_REPLACE},              // battery      ind = 0     
                     {90, 0, 12, 10,BITMAP_NORMAL, BUF_REPLACE},                // heart       ind = 1
                     {70, 0, 12, 10,BITMAP_NORMAL, BUF_REPLACE},                // bt       ind = 2
                     {45, 0, 23, 15, BITMAP_NORMAL, BUF_REPLACE},               // Mist       ind = 3
                     {30, 0, 12, 10,BITMAP_NORMAL, BUF_REPLACE},                // bt        ind = 4
                     {10,0, 12, 10,BITMAP_NORMAL, BUF_REPLACE},                 // Vibrstion ind  =5
                     {48,20,32,32, BITMAP_NORMAL, BUF_REPLACE},                  // Still  = 6
                     {20,30,107,43, BITMAP_NORMAL, BUF_REPLACE}                   // Text = 7
                     };   

void setup() {
  Serial.begin(115200);
  initBLE();
  oled.init();
  oled.clear();
  drawPictogr(batt_fifty_23x10,0);
  drawPictogr(heart_on_12x10,1);
  drawPictogr(Daw_12x10,2);
  drawPictogr(mist_two_23x15,3);
  drawPictogr(Bluetooth_12x10,4);
  drawPictogr(Vibr_12x10,5);
  drawPictogr(Still_one_32x32,6);
  delay(250);
  drawPictogr(Still_two_32x32,6);
  
  // initializing the gyroscope
  mpu.initialize();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    for (int i = 0; i < 30; i++) 
      {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        ACC = abs(ax) + abs(ay) + abs(az); GYR = abs(gx) + abs(gy) + abs(gz);
        if (ACC > maxACC) maxACC = ACC;
        if (GYR > maxGYR) maxGYR = GYR;
      }
  delay(250);
  drawPictogr(Still_three_32x32,6);
  delay(250);
  drawPictogr(Still_four_32x32,6);
  // initializing HTU21D
  myHumidity.begin();
  delay(250);
  drawPictogr(Still_fie_32x32,6);
  delay(250);
  drawPictogr(Still_sixe_32x32,6);
  /*----------Initializate clock---------*/
  rtc.initClock();
  rtc.setDate(day, week, month, century, year);
  rtc.setTime(hour, minute, second);
  dm = rtc.formatDate();
  hm = rtc.formatTime(RTCC_TIME_HM);
  delay(250);
  drawPictogr(Still_seven_32x32,6);
  delay(250);
  drawPictogr(Still_eight_32x32 ,6);
  // initializate MAX30105
  if (! PARTICLE_SENSOR.begin(Wire, I2C_SPEED_FAST))                  //  We initiate work with the module. If initialization failed, то
      {                    
        Serial.println("MAX30105 was not found");                       //  hen we output a message about this to the serial port monitor
        while (1);                                                      //  and stop further execution of the sketch
       } 
    byte ledBrightness = 0x50; //Options: 0=Off to 255=50mA
    byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
    byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
    int sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 411; //Options: 69, 118, 215, 411
    int adcRange = 8192; //Options: 2048, 4096, 8192, 16384
    // Set up the wanted parameters
    PARTICLE_SENSOR.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  
  delay(250);
  drawPictogr(Still_nine_32x32 ,6);
  delay(250);
  drawPictogr(Still_ten_32x32,6);
  // initializate bittom and flash
  pinMode (PIN_BUTTON1, INPUT);
  pinMode (PIN_BUTTON2, INPUT);
  pinMode (PIN_BUTTON3, INPUT);
  pinMode (PIN_BUTTON4, INPUT);
//  pinMode(PIN_OUTPUT, OUTPUT);
  //xTaskCreateUniversal(taskButtons, "buttons", 4096, NULL, 2, NULL,1);                     // The task of working with the button is started 
  //ReadFILE_FLASH ();
  delay(250);
  drawPictogr(Still_eleven_32x32 ,6);
  delay(250);
  drawPictogr(Still_twelve_32x32,6);
  delay(250);
  oled.clear();
}

void loop() {
  humidity = myHumidity.readHumidity();
  temperature = myHumidity.readTemperature();
  CPU = (temprature_sens_read() - 32) / 1.8;
  unsigned long currentMillis = millis();

  if (deviceConnected) {
    String val = dm +" "+ hm;
    unsigned char* buf = new unsigned char[20];

    val.getBytes(buf, 20, 0);
    const char *str2 = (const char*)buf;
    pTxCharacteristic->setValue(str2);
    pTxCharacteristic->notify();
    
    char x[1];                                                   //
	  dtostrf(temperature, 5/*Полная_длина_строки*/, 1/*Количество_символов_после_запятой*/,x);
    pTxTimeCharacteristic ->setValue(x);
    pTxTimeCharacteristic ->notify();
   
//   char ax[1];                                                   //
////	  dtostrf(minTempPar, 5/*Полная_длина_строки*/,1/*Количество_символов_после_запятой*/,ax);
//    sprintf(ax, "%d", minTempPar);
//    pTxminTempParCharacteristic ->setValue(ax);
//    pTxminTempParCharacteristic ->notify();

		delay(10); // bluetooth stack will go into congestion, if too many packets are sent
	}

    // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
    // connecting
  if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }

  //currentMillis = millis();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  acseler();

   
  if (( millis() - currentMilliisFLash > intervalWriteFlash) and (flagPar == 1))
      {
        currentMilliisFLash = millis();
        //ADDin_FLASH (dm+hm, int(temperature), byte(humidity), byte(heartRate),metka);
        metka = 0;
               
      }
    if ( flagPar == 1 and flagUpdatePar == 0) 
      {
         drawPictogr(mist_two_23x15, 2);
         flagUpdatePar = 1;
         interval = 10000;
      }
    else
      {
         if (flagPar == 0 and flagUpdatePar == 1)
          {
            drawPictogr(mist_two_23x15, 2);
            flagUpdatePar = 0;
            interval = 30000;
          }
      }
    if (flagACC == 1)
      {
        otlDisp = 0;
        oled.setPower(1);
        printTest(temperature, humidity);
        currentMilliisFLash = millis();
        if (currentMillis - previousMillis >= interval) 
          {
//              mean_hrb();
              humidity = myHumidity.readHumidity();
              previousTemperature = temperature;
              temperature =myHumidity.readTemperature();
              dm = rtc.formatDate();
              hm = rtc.formatTime(RTCC_TIME_HM);
           }
        if (heartRate > 0 )
          {
            disp = 3;
          }
        else
          {
            disp = 2;
          }
        
        if ((abs(temperature -previousTemperature) > deltaTemp) and (temperature > minTempPar) )
            {
              flagPar= 1;
            }
        else 
          {
            if (temperature < minTempPar )  flagPar = 0;
          }
        
        //printTemperatureToSerial(); 
        if (currentMillis - previousMillis >= interval) 
          {
            if (flagPar == 1 and flagButt2 == 0)//
              {
                 flagDisplay++;
                 if (flagDisplay > disp) flagDisplay = 0;
                 printTest(temperature, humidity);
              }
            else
              {
                // flagDisplay++;
                //  if (flagDisplay > disp) flagDisplay = 0;
                // previousMillis = currentMillis;
                printTest(temperature, humidity);
              }
          }
      }
     else
      {
        if (otlDisp ==0) 
          {
            // drawStill(clear_display, sizeof(clear_display));
            otlDisp = 1;
          }
       }
    //  Button 1  processing
if (action_Button1)
  {
    if (state_BT == LOW)
       {
         //drawPictogr(vibr, 3);
         //Size_and_ReadFILE_FLASH ();               //ВРЕМЕННО ДЛЯ ПРОВЕРКИ ЧТЕНИЯ С FLASH  
         //Serial.println(SizeF);
         flagDisplay++;
         if (flagDisplay > disp) flagDisplay = 0;
         printTest(temperature, humidity);
                       
       }   
    if  (state_BT == HIGH)
       {
         //drawPictogr(vibr_off, 3);
       }   
    state_BT = !state_BT;
    action_Button1 = LOW;
  }
if (Button2_released)
  {
    if (double_press == HIGH)
      {
       //Serial.println("Double 1");                       // автоматическое отображение параметров в цикле
       flagButt2 = 0;
       Button2_released = LOW;
       double_press = LOW;
      }
 
  }
else if (Button2_pressed && !last_Button2_pressed)
  {
    Button2_released = LOW;
    double_press = LOW;
    onTime_B2 = millis();
  }
if (!Button2_pressed && last_Button2_pressed)
  {
    Button2_released = LOW;
    double_press = LOW;
    if (((millis() - onTime_B2) > bounceTime_B2))
      {
        if ((millis() - lastSwitchTime_B2) >= doubleTime_B2)
          {
           lastSwitchTime_B2 = millis();
           button_2_Single = true;
           i=1;
          } 
        else 
          {
           i++;
           lastSwitchTime_B2 = millis();
           button_2_Single = false;
           button_2_Multi = true;
          }
     }
  }
last_Button2_pressed = Button2_pressed; 
  
if (button_2_Single && (millis() - lastSwitchTime_B2) > doubleTime_B2)
   {
    isButton2Single();
   }
if (button_2_Multi && (millis() - lastSwitchTime_B2) > doubleTime_B2)
   {
    isButton2Multi(i);
   }


//  Button 3  processing  
if (Button3_pressed && !last_Button3_pressed)
  {
   // Button3_released = LOW;
   // Button3_pressed = LOW;
    onTime_B3 = millis();
  }
if (Button3_pressed && last_Button3_pressed)         //анализ долгого нажатия
  {
   // Button3_released = LOW;
    //Button3_pressed = LOW;
    if ((millis() - onTime_B3) > holdTime_B3)
      {
       holdTimeSum_B3 = (millis() - onTime_B3);
       button_3_Hold = true;
      }
  }
if (!Button3_pressed && last_Button3_pressed)
  {
   //Button3_released = LOW;
   //Button3_pressed = LOW;
   if (((millis() - onTime_B3) > bounceTime_B3) && !button_3_Hold)
     {
      if ((millis() - lastSwitchTime_B3) >= doubleTime_B3)
        {
         lastSwitchTime_B3 = millis();
         button_3_Single = true;
         j=1;
        } 
      else 
        {
         j++;
         lastSwitchTime_B3 = millis();
         button_3_Single = false;
         button_3_Multi = true;
        }
     }
     if (button_3_Hold)
       {
        isButton_3_Hold( holdTimeSum_B3 );
       }
  }
last_Button3_pressed = Button3_pressed;
 
if (button_3_Single && (millis() - lastSwitchTime_B3) > doubleTime_B3)
  {
    isButton_3_Single();
  }
if (button_3_Multi && (millis() - lastSwitchTime_B3) > doubleTime_B3)
  {
    isButton_3_Multi(i);
  }


//  Button 4  processing
if (action_Button4)
  {
    isButton_4();
  }
}

void isButton2Single()                    // processing functions of the second button
   {                                      //1 press
      button_2_Multi = false;
      button_2_Single = false;
      //Serial.println(1);
     //Write_FLASH (VR, temper, VL, pulse, metka);           // ВРЕМЕННО ДЛЯ ПРОВЕРКИ FLASH
     flagButt2 = 1;
     if (flagDisplay > disp) flagDisplay = 0;
     printTest(temperature, humidity);
     flagDisplay++;
   }

void isButton2Multi( int count )          // processing functions of the second button
   {                                      //2 press
      button_2_Single = false;
      button_2_Multi = false;
        if (count == 2)
          {
            //Serial.println("Double 2"); 
            flagButt2 = 0;
          }
   }

void isButton_3_Single()                  //processing functions of the third button
  {
      button_3_Multi = false;
      button_3_Single = false;
      //Serial.println("Single B3");
      //Erase_FLASH ();                    //ВРЕМЕННО ПРОВЕРКА СТИРАНИЯ FLASH
      metka = 1;
      //drawPictogr(daw, 5);
  }
void isButton_3_Multi( int count_B3 )     //processing functions of the third button
  {
      button_3_Single = false;
      button_3_Multi = false;
      if (count_B3 == 2)
        {
          Serial.println("Double B3");
        }
  }

void isButton_3_Hold( int count_B3 )      //processing functions of the third button
  {
      button_3_Hold = false;
      button_3_Multi = false;
      Button3_released = LOW;
      if (state_BT3 == LOW)
        {
          //drawPictogr(bt, 4);
        }   
       if (state_BT3 == HIGH)
        {
          //drawPictogr(bt_off, 4);
        }   
      state_BT3 = !state_BT3;
      Serial.println("Hold B3");
  }

  void isButton_4()                     //processing functions of the fourth button
  {
    Serial.println("Bad Value");
    action_Button4 = LOW;
    //ADDin_FLASH (VR, temper, VL, pulse, metka);                        //ВРЕМЕННО ДЛЯ ПРОВЕРКИ ДОБАВКИ FLASH 
  }

void printTest(const float& t,const float& hu ) {
  unsigned int x = NamePict[7][0];
  unsigned int y = NamePict[7][1];
  unsigned int h = NamePict[7][2];
  unsigned int w = NamePict[7][3];
  oled.setScale(2); 
  //oled.drawBitmap(x,y,Pictogr,h,w,  NamePict[ind][4],  NamePict[ind][5]);
  oled.update(25,20,127,60);
  oled.setCursorXY( x, y);
  switch (flagDisplay)
      {
        case 0:
          {
            
            oled.print(hm);
            oled.update();
            break;
          }
        case 1:
          {
            char data[] = "Temp: ";
            oled.print(t);
            oled.println(" C");
            oled.update();
            break;
          }
        case 2:
          {
            char data[] = "Humi: ";
            //oled.print(data);
            oled.print(hu);
            oled.println(" %");
            oled.update();
            break;
          }
        case 3:
          {
            String strbeatAvg = String(heartRate,0);
            //oled.print("AVG= ");
            oled.println(strbeatAvg);
            oled.update();
            break;
          }
      }
 }


void drawPictogr(const unsigned char *Pictogr, unsigned int ind)
{
    unsigned int x = NamePict[ind][0];
    unsigned int y = NamePict[ind][1];
    unsigned int h = NamePict[ind][2];
    unsigned int w = NamePict[ind][3];
    oled.drawBitmap(x,y,Pictogr,h,w,  NamePict[ind][4],  NamePict[ind][5]);
    oled.update();
    
}

// void IRAM_ATTR ISR_btn()
//   {
//      xSemaphoreGiveFromISR( btnSemaphore, NULL );        // Прерывание по кнопке, отпускаем семафор
//   }

// void taskButtons( void *pvParameters )
//   {
//     bool isISR     = true;
//     bool state_btn1 = true, state_btn2 = true, state_btn3 = true, state_btn4 = true;
//     btnSemaphore = xSemaphoreCreateBinary();              // Создаем семафор 
//     xSemaphoreTake( btnSemaphore, 100 );                  // Сразу "берем" семафор чтобы не было первого ложного срабатывания кнопки 
//     attachInterrupt(PIN_BUTTON1, ISR_btn, FALLING);       // Запускаем обработчик прерывания (кнопка замыкает GPIO на землю) на все кнопки
//     attachInterrupt(PIN_BUTTON2, ISR_btn, FALLING);   
//     attachInterrupt(PIN_BUTTON3, ISR_btn, FALLING);
//     attachInterrupt(PIN_BUTTON4, ISR_btn, FALLING);
//     while(true)
//       {
//         // Обработчик прерывания выключен, функция ждет окончания действия с кнопкой     
//         if( isISR )
//           {
         
//             xSemaphoreTake( btnSemaphore, portMAX_DELAY );         // Ждем "отпускание" семафора
//             detachInterrupt(PIN_BUTTON1);                          // Отключаем прерывания по всем кнопкам
//             detachInterrupt(PIN_BUTTON2);
//             detachInterrupt(PIN_BUTTON3);
//             detachInterrupt(PIN_BUTTON4);
//             isISR = false;                                              // переводим задачу в цикл обработки кнопки
//            }
//         else 
//           {
//             bool st1 = digitalRead(PIN_BUTTON1);
//             bool st2 = digitalRead(PIN_BUTTON2);
//             bool st3 = digitalRead(PIN_BUTTON3);
//             bool st4 = digitalRead(PIN_BUTTON4);
//             if( st1 != state_btn1 )                                      // Проверка изменения состояния кнопки1 
//              {
//                 state_btn1 = st1;
//                 if( st1 == LOW )
//                   { 
//                     Serial.println("Button1 pressed"); 
//                   }
//                 else 
//                   { 
//                     action_Button1 = HIGH;
//                     Serial.println("Button1 released"); 
//                   }
//               }        
//             if( st2 != state_btn2 )                                           // Проверка изменения состояния кнопки2
//               {  
//                 state_btn2 = st2;
//                 if( st2 == LOW )
//                   {
//                     Button2_pressed = HIGH; 
//                     Serial.println("Button2 pressed"); 
//                   }
//                 else 
//                   {
//                     Button2_pressed = LOW;
//                     if (Button2_released == HIGH)
//                       {
//                         double_press = HIGH;
//                       }
//                     Button2_released = HIGH;
//                     Serial.println("Button2 released"); 
//                   }
//               }        
//            if( st3 != state_btn3 )// Проверка изменения состояния кнопки3
//               {
//                 state_btn3 = st3;
//                 if( st3 == LOW )
//                   { 
//                     Button3_pressed = HIGH;
//                     Serial.println("Button3 pressed"); 
//                    }
//                 else 
//                   { 
//                     Button3_pressed = LOW;
//                     Button3_released = HIGH;
//                     Serial.println("Button3 released"); 
//                   }
//                 } 
//              if( st4 != state_btn4 )// Проверка изменения состояния кнопки4
//                {
//                   state_btn4 = st4;
//                   if( st4 == LOW )
//                     { 
//                       Serial.println("Button4 pressed"); 
//                      // digitalWrite(PIN_OUTPUT, LOW);
//                     }
//                   else 
//                     { 
//                       Serial.println("Button4 released");
//                       action_Button4 = HIGH; 
//                      // digitalWrite(PIN_OUTPUT, HIGH);
//                     }
//                 }                
// // Проверка что все четыре кнопки отработали
//              if( st1 == HIGH && st2 == HIGH && st3 == HIGH && st4 == HIGH )
//                 { 
//                   attachInterrupt(PIN_BUTTON1, ISR_btn, FALLING);   
//                   attachInterrupt(PIN_BUTTON2, ISR_btn, FALLING);   
//                   attachInterrupt(PIN_BUTTON3, ISR_btn, FALLING);
//                   attachInterrupt(PIN_BUTTON4, ISR_btn, FALLING);
//                   isISR = true;
//                   }
//               vTaskDelay(100);
//           }
//       }
//   }

void ReadFILE_FLASH ()
  {
    // Инициализация SPIFFS
    if(!SPIFFS.begin(true))
      {
        Serial.println("Error while mounting SPIFFS");
        return;
      } 
    // Прочитать содержимое файла
    Data_array_file = SPIFFS.open(Data_arrayPath, FILE_READ);
    //Serial.print("File content: \"");
//    while(Data_array_file.available()) 
//      {
//        Serial.write(Data_array_file.read());
//      }
//    Serial.println("\"");   
    // Проверить размер файла
//    Serial.print("File size: ");
//    //Serial.println(Data_array_file.size());
 SizeF = Data_array_file.size();
    Data_array_file.close();
  }

void initBLE()
  {
    // Create the BLE Device
    BLEDevice::init("UART Service");

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
				CHARACTERISTIC_TIME_TX_UUID,
				BLECharacteristic::PROPERTY_NOTIFY
				);
                      
    pTxCharacteristic->addDescriptor(new BLE2902());

    pTxTimeCharacteristic = pService->createCharacteristic(
				CHARACTERISTIC_TEMP_TX_UUID,
				BLECharacteristic::PROPERTY_NOTIFY
				);
    pTxTimeCharacteristic->addDescriptor(new BLE2902());

  //  pTxminTempParCharacteristic= pService->createCharacteristic(
  //										CHARACTERISTIC_minTempPar_TX_UUID,
  //										BLECharacteristic::PROPERTY_NOTIFY
  //									);
  //  pTxminTempParCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
				CHARACTERISTIC_TIME_RX_UUID,
				BLECharacteristic::PROPERTY_WRITE
				);

    pRxCharacteristic->setCallbacks(new MyCallbacks());

  //  BLECharacteristic * pRxTCharacteristic = pService->createCharacteristic(
  //											CHARACTERISTIC_minTempPar_RX_UUID,
  //											BLECharacteristic::PROPERTY_WRITE
  //										);
  //
  //  pRxTCharacteristic->setCallbacks(new MyCallbacksTempPar());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    Serial.println("Waiting a client connection to notify...");
  }
void acseler ()
  {
   
    ACC = abs(ax) + abs(ay) + abs(az);
    GYR = abs(gx) + abs(gy) + abs(gz);
    if (ACC > (maxACC) || GYR > maxGYR) 
      {
        // Serial.print("ACC = ");
        // Serial.println(ACC);
        flagACC = 1;
        currentMillisAcc = millis();
        maxACC = ACC;
        maxGYR = GYR;
        //maxACC
      }
    else 
      {
        if ((millis() - currentMillisAcc) > INTERVAL_ACC)
          {
            flagACC = 0;
            currentMillisAcc = millis();
            maxACC = 0;
            maxGYR = 0;
          }
      }
  }

  void mean_hrb()
  {
    bufferLength = 100;                               //  Устанавливаем длину буфера равным 100 (куда будут записаны пакеты по 25 значений в течении 4 секунд)
                                                      //  считываем первые 100 значений и определяем диапазон значений сигнала:
    if (PARTICLE_SENSOR.getRed()>5000)
      {                                                     
        for (byte i = 0 ; i < bufferLength ; i++)     //  проходим в цикле по буферу и
          {         
            while (PARTICLE_SENSOR.available() == false)      //  отправляем сенсору запрос на получение новых данных
            PARTICLE_SENSOR.check();
            redBuffer[i] = PARTICLE_SENSOR.getIR();           //  Записываем в массив значения сенсора, полученные при работе с КРАСНЫМ светодиодом
            irBuffer[i] = PARTICLE_SENSOR.getRed();           //  Записываем в массив значения сенсора, полученные при работе с ИК      светодиодом
            PARTICLE_SENSOR.nextSample();                     //  Как только в буфер было записано 100 значений - отправляем сенсору команду начать вычислять значения ЧСС и SpO2
          }
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
        Serial.print("spo2");
        Serial.println(spo2);
          }
    else
      {
        spo2 = 0;
        heartRate = 0;  
      }
  }
