#include "max6675.h"
#include <Servo.h>
#include "time.h"
#include "EasyNextionLibrary.h"
#include "Nextion.h"

byte soPin = 4; //Assigns Exhaust Gas Temperature (EGT) sensor interface pin to Digital pin 4
byte csPin = 5; //Assigns EGT sensor interface pin to Digital pin 5
byte sckPin = 6; //Assigns EGT sensor interface pin to Digital pin 6

const unsigned long timeEgt = 180; //Assigns time interval in milliseconds for getting/sending EGT values
const unsigned long timeServo = 100;
const unsigned long timeMap = 50;//Assigns time interval in milliseconds for getting/sending EGT values
const unsigned long timeAir = 100;

unsigned long previousTime1 = 0;
unsigned long previousTime2 = 0;
unsigned long previousTime3 = 0;
unsigned long previousTime4 = 0;

MAX6675 robojax(sckPin, csPin, soPin); //Initializes the EGT sensor interface:
Servo servo; //Initializes servo:

void setup(){
  Serial.begin(9600);
  Serial3.begin(9600);
  
  servo.attach(9); //Attaches servo to didgital pin 9:
  servo.write(0); //Writes Servo angle to 0 on start up: //Create a shutdown function that sets servo angle to 0 when power is lost:
  
  delay(1000);
  //Serial.print("F = "):
  //Serial.println(robojax.readFarenheit()): //Prints temp in Farenheit to serial monitor:  
}

void loop() {
  unsigned long currentTime = millis();
  if(currentTime - previousTime1 >= timeEgt){
    float temperature = robojax.readFarenheit();
    if(temperature > 200.2){
      float in_min = 200.2;
      float in_max = 1500.0;
      float out_min = 0.0;
      float out_max = 270.0;
      const unsigned long tempangle = (float)(temperature - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
      Serial3.print("va0.val=" + String(tempangle));
      Serial3.write(0xff);
      Serial3.write(0xff);
      Serial3.write(0xff);
      
     }else{
      float in_min = 0.0;
      float in_max = 198.8;
      float out_min = 315.0;
      float out_max = 0.0;
      const unsigned long tempangle = (float)(temperature - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
      Serial3.print("va0.val=" + String(tempangle));
      Serial3.write(0xff);
      Serial3.write(0xff);
      Serial3.write(0xff); 
     }
     previousTime1 = currentTime;
    
  }
  
  if(currentTime - previousTime2 >= timeServo){
    float poten1 = ((analogRead(A0))-73.00)/3.95;
    float poten2 = ((analogRead(A1))-143.00)/7.7;
    servo.write((poten1+poten2)/2);
    previousTime2 = currentTime;
    
  }
  if(currentTime - previousTime3 >= timeMap){
    float mapVoltage = analogRead(A2);
    int i = 0;
    float mapVal = 0.0;
    while(i < 11){
      mapVal = mapVal + mapVoltage;
      i++;
      delay(5); 
    }
    long avg_map = (mapVal/10.0);
    long in_min = 80.0; //in_min minimum value the arduino reads from the sensor:
    long in_max = 1050.0; //in_max maximum value:
    long out_min = 0.25; //Bosch analog map (manifold absolute pressure) Sensor minimum output voltage, represents 50kpa pressure:
    long out_max = 4.48; //Bosch map sensor max voltage represnts 400 kpa, these are linearly proportional to the input voltage at the senor:
    const unsigned long mapped = (float)(avg_map - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
    //Using the math from the map() function to get the output ranges within the given range of 0.25 to 4.48 volts as a float for the map sensor:
    float pressure = ((387.2340426*(mapped/4.68))-46.80851064)/6.8;
    float in_min2 = 0.0;
    float in_max2 = 50.0;
    float out_min2 = 0.0;
    float out_max2 = 180.0;
    const unsigned long pressureangle = (float)(pressure - in_min2) * (out_max2 - out_min2) / (float)(in_max2 - in_min2) + out_min2;

    Serial3.print("va1.val=" + String(pressureangle));
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
        
    
    //https://www.youtube.com/watch?v=Hn_z9eu8nR4 in depth explanation of mapsensor equations:
    //Using the linear relationship of the output voltage to find pressure then converting from kpa to psi:
    //The above video helped me understand this a lot better but he used a different sensor so I followed the equation with my sensor's ranges:
    //Serial.println(pressure):
    
    
    previousTime3 = currentTime;
  }
}
  
