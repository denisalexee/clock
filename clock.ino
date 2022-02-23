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
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

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

#define TM_BUTTON 100                            // Минимальный таймаут между событиями нажатия кнопки
#define PIN_BUTTON1 34
#define PIN_BUTTON2 35
#define PIN_BUTTON3 33
#define PIN_BUTTON4 19
#define PIN_OUTPUT 11

#define SERVICE_UUID                 "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_TIME_RX_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_TIME_TX_UUID  "60171725-f8a0-41b9-809d-c2044db71d8f"

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


void setup() {
  Serial.begin(115200);

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

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											CHARACTERISTIC_TIME_RX_UUID,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

  Serial.println("Battary_low");
  oled.init();
  oled.clear();
  oled.drawBitmap(104,0,bitmap_23x10,23,10);
  oled.drawBitmap(90,0,bitmap_12x10,12,10);
  
  oled.update();//, sizeof(Still_one)
  delay(5000);
  
  Serial.println("HTU21D Example!");
  myHumidity.begin();
/*----------Initializate clock---------*/
  rtc.initClock();
  rtc.setDate(day, week, month, century, year);
  rtc.setTime(hour, minute, second);
  dm = rtc.formatDate();
  hm = rtc.formatTime(RTCC_TIME_HM);

}

void loop() {

  if (deviceConnected) {
    String val = dm + hm;
    unsigned char* buf = new unsigned char[20];
    val.getBytes(buf, 20, 0);
    const char *str2 = (const char*)buf;
    pTxCharacteristic->setValue(str2);
    pTxCharacteristic->notify();
    txValue++;
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
  oled.println(dm+" "+hm);
  oled.update();
 }
