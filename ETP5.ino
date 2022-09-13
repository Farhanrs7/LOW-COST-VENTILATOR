/*
  Optical Heart Rate Detection (PBA Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30105_Breakout

  This is a demo to show the reading of heart rate or beats per minute (BPM) using
  a Penpheral Beat Amplitude (PBA) algorithm.

  It is best to attach the sensor to your finger using a rubber band or other tightening
  device. Humans are generally bad at applying constant pressure to a thing. When you
  press your finger against the sensor it varies enough to cause the blood in your
  finger to flow differently which causes the sensor readings to go wonky.

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected

  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/

#include <Wire.h>
#include "MAX30105.h"
#include <LiquidCrystal.h>
#include "heartRate.h"
#include <Adafruit_BMP085.h>

#define dirPin 12
#define stepPin 13
#define stepsPerRevolution 80

MAX30105 particleSensor;

int Contrast = 30;
LiquidCrystal lcd(11, 10, 5, 4, 3, 2);
Adafruit_BMP085 bmp;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
long irValue;
long delta;


int diff;
int sec;
int intime;




void setup()
{
  //motor pins setup
  pinMode(dirPin,OUTPUT);
  pinMode(stepPin,OUTPUT);
  
  analogWrite(7, Contrast);
  lcd.begin(16, 2);
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }
}

void loop()
{
  digitalWrite(dirPin, HIGH);
  for (int i = 0; i < stepsPerRevolution; i++) {
  // These four lines result in 1 step:
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(2000);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(2000);
  }
  delay(1000);

  // Set the spinning direction counterclockwise:
  digitalWrite(dirPin, LOW);

  // Spin the stepper motor 1 revolution quickly:
  for (int i = 0; i < stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000);
  }
  intime = millis();
  while(diff < 1000)
  {
    sec = millis();
    diff = sec - intime;
    irValue = particleSensor.getIR();
  
    if (checkForBeat(irValue) == true)
    {
      //We sensed a beat!
      delta = millis() - lastBeat;
      lastBeat = millis();

  
      beatsPerMinute = 60 / (delta / 1000.0);
  
      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable
  
        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
     }

    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM="); 
    Serial.print(beatAvg);
    Serial.println();

//  Serial.println(particleSensor.getIR()); //Send raw data to plotter
     Serial.print("Pressure="); 
    Serial.print(bmp.readPressure());
    Serial.println();
    
   }
  lcd.clear();
  lcd.setCursor(0,0);
  Serial.print("hey ");
  Serial.println(irValue);
  if (irValue < 50000){
      lcd.print(" No finger");
  }
  else {
      lcd.setCursor(0, 0);
      lcd.print("Avg BPM= ");
      lcd.print(beatAvg);
      lcd.setCursor(0, 1);
      lcd.print("Pressure= ");
      lcd.print(bmp.readPressure());
  }
    //delay(1000);
   // lcd.clear();

  Serial.println();
  diff=0;
}
