#include <Wire.h>
#include "SparkFunHTU21D.h"
#include "MAX30105.h"           // Библиотека пульсометра
#include "BluetoothSerial.h"    // Библиотека обычного Bluetooth
#include <GyverOLED.h>          // Библиотека экрана
#include <Rtc_Pcf8563.h>        // Библиотека часов реального времени
#include "MPU6050.h"            // Библиотека гироскоп
#include "heartRate.h"
#include "spo2_algorithm.h"  
#include <FS.h>                // Доступ к файловой системе
#include <SPIFFS.h>
//----------------bitmap--------------------------
#include "Still.h"
#include "batt.h"
#include "Bluetooth.h"
#include "daw_x.h"
#include "heart.h"
#include "mist.h"
#include "vibr.h"

MAX30105 particleSensor;
HTU21D myHumidity;
BluetoothSerial ESP_BT;
GyverOLED<SSH1106_128x64> oled;


const int RES = 27;

#ifdef __cplusplus
extern "C" {
#endif

uint8_t temprature_sens_read();

#ifdef __cplusplus
}
#endif

uint8_t temprature_sens_read();


void setup()
{
  Serial.begin(115200); 
  Serial.println("Battary_low");
  oled.init();
  oled.clear();
  oled.drawBitmap(104,0,batt_low_23x10,23,10);
  oled.drawBitmap(90,0,heart_on_12x10,12,10);
  oled.drawBitmap(70,0,Daw_12x10,12,10);
  oled.drawBitmap(45,0,Mist_23x15,23,15);
  oled.drawBitmap(30,0,Bluetooth_12x10,12,10);
  oled.drawBitmap(10,0, Vibr_12x10,12,10);
  oled.drawBitmap(48,20, Still_one_32x32,32,32);
  oled.update();//, sizeof(Still_one)
  delay(2000);

  ESP_BT.begin("ESP32_Temper");
  Serial.println("HTU21D Example!");
  oled.drawBitmap(48,20, Still_two_32x32,32,32);
  oled.drawBitmap(104,0,batt_full_23x10,23,10,BITMAP_NORMAL, BUF_REPLACE);
  oled.update();//, sizeof(Still_one)
  delay(1000);
  oled.drawBitmap(48,20, Still_three_32x32,32,32,BITMAP_NORMAL, BUF_REPLACE);
  oled.update();//, sizeof(Still_one)
  delay(1000);
  oled.drawBitmap(48,20, Still_four_32x32,32,32,BITMAP_NORMAL, BUF_REPLACE);
  oled.update();//, sizeof(Still_one)
  delay(1000);
  oled.drawBitmap(48,20, Still_five_32x32,32,32,BITMAP_NORMAL, BUF_REPLACE);
  oled.update();//, sizeof(Still_one)
  delay(1000);
  oled.drawBitmap(48,20, Still_six_32x32,32,32,BITMAP_NORMAL, BUF_REPLACE);
  oled.update();//, sizeof(Still_one)
  delay(1000);
  oled.drawBitmap(48,20, Still_seven_32x32,32,32,BITMAP_NORMAL, BUF_REPLACE);
  oled.update();//, sizeof(Still_one)
  delay(1000);
  oled.drawBitmap(48,20, Still_eight_32x32,32,32,BITMAP_NORMAL, BUF_REPLACE);
  oled.update();//, sizeof(Still_one)
  delay(1000);
  oled.drawBitmap(48,20, Still_nine_32x32,32,32,BITMAP_NORMAL, BUF_REPLACE);
  oled.update();//, sizeof(Still_one)
  delay(1000);
  oled.drawBitmap(48,20, Still_ten_32x32,32,32,BITMAP_NORMAL, BUF_REPLACE);
  oled.update();//, sizeof(Still_one)
  delay(1000);
  oled.drawBitmap(48,20, Still_eleven_32x32,32,32,BITMAP_NORMAL, BUF_REPLACE);
  oled.update();//, sizeof(Still_one)
  delay(1000);
  oled.drawBitmap(48,20, Still_twelve_32x32,32,32,BITMAP_NORMAL, BUF_REPLACE);
  oled.update();//, sizeof(Still_one)
  delay(1000);
  oled.drawBitmap(48,20, Still_thirteen_32x32,32,32,BITMAP_NORMAL, BUF_REPLACE);
  oled.update();//, sizeof(Still_one)
  delay(1000);
  
  myHumidity.begin();
}

void loop()
{
  float humd = myHumidity.readHumidity();
  float temp = myHumidity.readTemperature();
  float CPU = (temprature_sens_read() - 32) / 1.8;
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
  ESP_BT.print("Hello World");
  ESP_BT.print("Time:");
  ESP_BT.print(millis());
  ESP_BT.print(" Temperature:");
  ESP_BT.print(temp, 1);
  ESP_BT.print("C");
  ESP_BT.print(" Humidity:");
  ESP_BT.print(humd, 1);
  ESP_BT.print("%");
  ESP_BT.print("Temperature CPU: ");
  // Convert raw temperature in F to Celsius degrees
  ESP_BT.print(CPU,1);
  ESP_BT.print(" C");
  ESP_BT.println();
  
  delay(10000);
}

void printTest(const float& t,const float& h, const float& cpu ) {
  oled.clear();
  char data0[] = "Temp: ";
  char data1[] = "Humi: ";
  char data2[] = "CPU:  ";
//  char data1 = 'b';
//  int data2 = -50;
//  float data3 = 1.25;
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
  oled.update();
  //delay(5000);
}
