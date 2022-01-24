#include <Wire.h>
#include "SparkFunHTU21D.h"
#include "MAX30105.h"  
#include "BluetoothSerial.h"
#include <GyverOLED.h>

MAX30105 particleSensor;
HTU21D myHumidity;
BluetoothSerial ESP_BT;
GyverOLED<SSH1106_128x64> oled;

char Lorem_ipsum[] = "Lorem ipsum dolor sit amet, лорем ипсум долор сит амет привет народ ё, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip...";
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
  ESP_BT.begin("ESP32_Temper");
  Serial.println("HTU21D Example!");
  pinMode(RES, OUTPUT);
  digitalWrite(RES,HIGH);
  delay(1000); 
  digitalWrite(RES, LOW);
  //             инициализация
  digitalWrite(RES, LOW);
  delay(1000);
  digitalWrite(RES,HIGH);  

  oled.init();  
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
  oled.invertText(true);
  //oled.println(data2);
  //oled.println(data3);
  oled.invertText(false);
  oled.update();
  //delay(5000);
}