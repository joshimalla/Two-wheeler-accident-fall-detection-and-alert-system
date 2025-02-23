/*
 @file BMI088Test.ino
 @brief  This is a test code for the BMI088 accelerometer and gyroscope sensor using SPI
 @detail The values for acceleration,gyroscope and from BMI088 will be displayed in serial monitor

 * Reference aries IoT board: https://vegaprocessors.in/devboards/ariesiot.html
 * Reference aries IoT pin diagram: https://vegaprocessors.in/files/ARIES-IoT-Pin-out.pdf
 * 
 * Required Libraries : VEGA_BMI088 (Version 1.0.0)
 * 
 * Note:
 * No external connections required for testing this code using ARIES IoT v1
*/
/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2018 Bolder Flight Systems
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
* and associated documentation files (the "Software"), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies or 
* substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <SPI.h>
#include <SoftwareSerial.h>

#include "BMI088.h"
#define VEGA_ARIES_IOT_V2
#define buzzer 7  //buzzer connected to PWM pin 7
#define period 500
SoftwareSerial ss(4, 3); //TX-GPIO-4 and  RX-GPIO-3




SPIClass SPI(2);

/* accel object */
Bmi088Accel accel(SPI,0xFF);
/* gyro object */
Bmi088Gyro gyro(SPI,21);

void setup() 
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  delay(2000);
  int status;
  /* USB Serial to print data */
  Serial.begin(9600);
  while(!Serial) {}
  /* start the sensors */
  status = accel.begin();
  ss.begin(9600);
 
  if (status < 0) {
    Serial.println("Accel Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro Initialization Error");
    Serial.println(status);
    while (1) {}
  }

  
}


void loop() 
{
  #if defined (VEGA_ARIES_IOT_V2)
  
  /* read the accel */
  accel.readSensor();
  /* read the gyro */
  gyro.readSensor();
  /* print the data */
  Serial.print("X_axis = ");
  float accely = accel.getAccelY_mss();
  Serial.print(accel.getAccelX_mss());
  Serial.println();
//  Serial.print("\t");
  Serial.print("Y_axis = ");
  Serial.print(accel.getAccelY_mss());
  Serial.println();
//  Serial.print("\t");
  Serial.print("Z_axis = ");
  Serial.print(accel.getAccelZ_mss());
  Serial.println();
//  Serial.print("\t");
//  delay(1000);
  if (accely > 7)
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    
    while (ss.available() > 0){
    // get the byte data from the GPS
    tone(buzzer,period);
    byte gpsData = ss.read();
    Serial.write(gpsData);
    //delay(1000);
    //analogWrite(buzzer, 0);
    
  }
   // delay(1000);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    analogWrite(buzzer, 0);
   // delay(1000); 
  }
//  Serial.print(gyro.getGyroX_rads());
//  Serial.print("\t");
//  Serial.print(gyro.getGyroY_rads());
//  Serial.print("\t");
//  Serial.print(gyro.getGyroZ_rads());
//  Serial.print("\t");
//  Serial.print(accel.getTemperature_C());
//  Serial.print("\n");
  /* delay to help with printing */
//  delay(20);
    

  #else
  Serial.println("Choose the correct board! " );
  #endif  
}
