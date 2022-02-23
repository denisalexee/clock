#include <Wire.h>
#include "SparkFunHTU21D.h"
#include "MAX30105.h"           // Библиотека пульсометра
// #include "BluetoothSerial.h"    // Библиотека обычного Bluetooth
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
#include "001_batt+.h"
#include "000_heart.h"

MAX30105 particleSensor;
HTU21D myHumidity;
//BluetoothSerial ESP_BT;
GyverOLED<SSH1106_128x64> oled;
Rtc_Pcf8563 rtc;


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
BLECharacteristic *pTime;
BLECharacteristic *pTxTime ;
bool flagTime = 0;

bool deviceConnected = false;

#define TM_BUTTON 100                            // Минимальный таймаут между событиями нажатия кнопки
#define PIN_BUTTON1 34
#define PIN_BUTTON2 35
#define PIN_BUTTON3 33
#define PIN_BUTTON4 19
#define PIN_OUTPUT 11

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_TIME_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_TIME_TX_UUID "60171725-f8a0-41b9-809d-c2044db71d8f"
extern "C" {
uint8_t temprature_sens_read();
}

// Создание Callback функции для сервера. 
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      // Обработка подключения телефона к устройству
      deviceConnected = 1;
    };

    void onDisconnect(BLEServer* pServer) {
      // Обработка отключения
      deviceConnected = 0;
    }
};

class ElemCallbacs : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value1 = pCharacteristic->getValue();
       }
};

class ElemCallbacsTime : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        Time = rxValue.c_str();
        day = Time.substring(0,2).toInt();
        week = Time[2] - '0';
        month = Time.substring(3,5).toInt();
        century = Time[5] - '0';
        year = Time.substring(6,8).toInt();
        hour = Time.substring(8,10).toInt();
        minute = Time.substring(10,12).toInt();
        second = Time.substring(12,14).toInt();
      }
      flagTime = 1;
       }
};



void setup()
{
  Serial.begin(115200); 
  Serial.println("Battary_low");
  oled.init();
  oled.clear();
  oled.drawBitmap(104,0,bitmap_23x10,23,10);
  oled.drawBitmap(90,0,bitmap_12x10,12,10);
  
  oled.update();//, sizeof(Still_one)
  delay(10000);
  // ESP_BT.begin("ESP32_Temper");
  String devName = "Clock";
  BLEDevice::init( "Clock");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTime= pService->createCharacteristic(CHARACTERISTIC_TIME_UUID,BLECharacteristic::PROPERTY_READ| BLECharacteristic::PROPERTY_WRITE);
  pTime->setCallbacks(new ElemCallbacsTime());
  //pTxTime = pService->createCharacteristic(
  //                  CHARACTERISTIC_TIME_TX_UUID,
  //                  BLECharacteristic::PROPERTY_NOTIFY
  //                );
  //pTxTime->addDescriptor(new BLE2902());

  BLEAdvertising *pAdvertising = pServer->getAdvertising();

  // BLEAdvertisementData adv;
  // adv.setName(devName.c_str());
  // pAdvertising->setAdvertisementData(adv);

  // BLEAdvertisementData adv2;
  // adv2.setCompleteServices(BLEUUID(SERVICE_UUID));
  // pAdvertising->setScanResponseData(adv2);

  pAdvertising->start();

  Serial.println("HTU21D Example!");
  // pinMode(RES, OUTPUT);
  // digitalWrite(RES,HIGH);
  // delay(1000); 
  // digitalWrite(RES, LOW);
  // //             инициализация
  // digitalWrite(RES, LOW);
  // delay(1000);
  // digitalWrite(RES,HIGH);  
  myHumidity.begin();
/*----------Initializate clock---------*/
  rtc.initClock();
  rtc.setDate(day, week, month, century, year);
  rtc.setTime(hour, minute, second);
  dm = rtc.formatDate();
  hm = rtc.formatTime(RTCC_TIME_HM);

}

void loop()
{
  if (deviceConnected) {
      String val = dm + hm;
        unsigned char* buf = new unsigned char[20];
        val.getBytes(buf, 20, 0);
        const char *str2 = (const char*)buf;
       // pTxTime ->setValue(str2);
       // pTxTime ->notify();
        
    }

  float humd = myHumidity.readHumidity();
  float temp = myHumidity.readTemperature();
  float CPU = (temprature_sens_read() - 32) / 1.8;
  dm = rtc.formatDate();
  hm = rtc.formatTime(RTCC_TIME_HM);
  Serial.print("Time:");
  Serial.print(millis());
  Serial.print(" Temperature:");
  Serial.print(temp, 1);
  Serial.print("C");
  Serial.print(" Humidity:");
  Serial.print(humd, 1);
  Serial.print("%");
  printTest(temp, humd, CPU);
  Serial.print("Temperature CPU: ");
  // Convert raw temperature in F to Celsius degrees
  Serial.print(CPU);
  Serial.print(" C");
  Serial.println();
  Serial.print(dm+" "+hm);
 // ESP_BT.print("Hello World");
 // ESP_BT.print("Time:");
  //ESP_BT.print(millis());
 // ESP_BT.print(" Temperature:");
 // ESP_BT.print(temp, 1);
//  ESP_BT.print("C");
//  ESP_BT.print(" Humidity:");
//  ESP_BT.print(humd, 1);
//  ESP_BT.print("%");
// ESP_BT.print("Temperature CPU: ");
  // Convert raw temperature in F to Celsius degrees
//  ESP_BT.print(CPU,1);
//  ESP_BT.print(" C");
//  ESP_BT.println();
if (flagTime == 1)
      {
        rtc.setDate(day, week, month, century, year);
        rtc.setTime(hour, minute, second);
        flagTime =0;
      }
 
  
  delay(10000);
}

void printTest(const float& t,const float& h, const float& cpu ) {
  oled.clear();
  char data0[] = "Temp: ";
  char data1[] = "Humi: ";
  char data2[] = "CPU:  ";
  oled.home();
  oled.setScale(1);
  oled.print(data0);
  oled.println(t);
  oled.print(data1);
  oled.println(h);
  oled.print(data2);
  oled.println(cpu);
  //oled.invertText(true);
  //oled.println(data2);
  //oled.println(data3);
  //oled.invertText(false);
  oled.println(dm+" "+hm);
  oled.update();
  
  //delay(5000);
}
