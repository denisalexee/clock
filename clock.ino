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
#include "mist_one.h"
#include "mist_two.h"
#include "mist_three.h"
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

int NamePict[7][6] = {
                     {104, 0, 23, 10,BITMAP_NORMAL, BUF_REPLACE},              // battery      ind = 0     
                     {90, 0, 12, 10,BITMAP_NORMAL, BUF_REPLACE},                // heart       ind = 1
                     {70, 0, 12, 10,BITMAP_NORMAL, BUF_REPLACE},                // bt       ind = 2
                     {45, 0, 23, 15, BITMAP_NORMAL, BUF_REPLACE},               // Mist       ind = 3
                     {30, 0, 12, 10,BITMAP_NORMAL, BUF_REPLACE},                // bt        ind = 4
                     {10,0, 12, 10,BITMAP_NORMAL, BUF_REPLACE},                 // Vibrstion ind  =5
                     {48,20,32,32, BITMAP_NORMAL, BUF_REPLACE}                  // Still and Text = 6
                     };   



void setup()
{
  Serial.begin(115200); 
  Serial.println("Battary_low");
  oled.init();
  oled.clear();
  drawPictogr(batt_fifty_23x10,0);
  drawPictogr(heart_on_12x10,1);
  drawPictogr(Daw_12x10,2);
  drawPictogr(mist_one_23x15,3);
  drawPictogr(Bluetooth_12x10,4);
  drawPictogr(Vibr_12x10,5);
  drawPictogr(Still_one_32x32,6);
 
  delay(2000);

  ESP_BT.begin("ESP32_Temper");
  Serial.println("HTU21D Example!");
  drawPictogr(Still_two_32x32,6);
  drawPictogr(mist_two_23x15,3);
  delay(1000);
  drawPictogr(Still_three_32x32,6);
  drawPictogr(mist_three_23x15,3);
  delay(1000);
  drawPictogr(Still_four_32x32,6);
  delay(1000);
  drawPictogr(Still_fie_32x32,6);
  delay(1000);
  drawPictogr(Still_sixe_32x32,6);
  delay(1000);
  drawPictogr(Still_seven_32x32,6);
  delay(1000);
  drawPictogr(Still_eight_32x32 ,6);
  delay(1000);
  drawPictogr(Still_nine_32x32 ,6);
  delay(1000);
  drawPictogr(Still_ten_32x32,6);
  delay(1000);
  drawPictogr(Still_eleven_32x32 ,6);
  delay(1000);
  drawPictogr(Still_twelve_32x32,6);
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
  oled.home();
  oled.setScale(1);
  oled.print(data0);
  oled.println(t);
  oled.print(data1);
  oled.println(h);
  oled.print(data2);
  oled.println(cpu);
  oled.update();
  //delay(5000);
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
