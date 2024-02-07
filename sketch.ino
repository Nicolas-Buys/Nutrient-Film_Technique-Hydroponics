#include "DFRobot_PH.h"
#include "DFRobot_EC10.h"
#include <DHT.h>
#include <EEPROM.h>


#define PH_PIN A1
#define EC_PIN A0
#define DHTPIN 2     
#define DHTTYPE DHT22 
#define MG_PIN A2
#define DC_GAIN 8.5
#define READ_SAMPLE_INTERVAL 50    //define how many samples you are going to take in normal operation
#define READ_SAMPLE_TIMES    5     //time interval in milliseconds
#define ZERO_POINT_VOLTAGE   0.220 //define the output of the sensor in volts when the concentration of CO2 is 400PPM
#define REACTION_VOLTGAE     0.030 //define the voltage drop of the sensor when move the sensor from air into 1000ppm CO2
float CO2Curve[3]  =  {2.602,ZERO_POINT_VOLTAGE,(REACTION_VOLTGAE/(2.602-3))};
float phVoltage, ecVoltage, co2Voltage, phValue, ecValue, temperature, humidity, co2Level = 25;
int liquidLevel = LOW;
int counter = 0;
volatile double waterFlow;
DFRobot_PH ph;
DHT dht(DHTPIN, DHTTYPE);
DFRobot_EC10 ec;

void setup() {
  // put your setup code here, to run once:
  pinMode(4, INPUT); //Water level sensor
  pinMode(5, OUTPUT); //Acid add
  pinMode(6, OUTPUT); //Base Add
  pinMode(7, OUTPUT); //Nutrient A add
  pinMode(8, OUTPUT); //Nutrient B add
  pinMode(9, OUTPUT); //Light Control
  pinMode(10, OUTPUT);  //Intake Fan control
  pinMode(11, OUTPUT);  //Extractor fan control
  Serial.begin(115200);
  waterFlow = 0;
  ph.begin();
  dht.begin();
  ec.begin();
  
  attachInterrupt(digitalPinToInterrupt(3), pulse, RISING);
}

void loop() {
  
  
  static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U){                  //time interval: 1s
        timepoint = millis();
        temperature = dht.readTemperature();         // read your temperature sensor to execute temperature compensation
        humidity = dht.readHumidity();
        phVoltage = analogRead(PH_PIN)/1024.0*5000;  // read the voltage
        ecVoltage = analogRead(EC_PIN)/1024.0*5000;  // read the voltage
        co2Voltage = MGRead(MG_PIN);
        phValue = ph.readPH(phVoltage,temperature);  // convert voltage to pH with temperature compensation
        ecValue =  ec.readEC(ecVoltage,temperature);
        co2Level = MGGetPercentage(co2Voltage,CO2Curve); 
        liquidLevel = digitalRead(4);
        Serial.print("temperature:");
        Serial.print(temperature,1);
        Serial.print("C  pH:");
        Serial.print(phValue,2);
        Serial.print("  humidity:");
        Serial.print(humidity,3);
        Serial.print("%  EC:");
        Serial.print(ecValue,4);
        Serial.print("ms/cm  liquidLevel=");
        Serial.print(liquidLevel,5);
        Serial.print("  CO2:");
        if (co2Level == -1) {
        Serial.print( "<400" );
        } 
        else {
        Serial.print(co2Level,6);
        }
        Serial.print( "ppm  waterFlow:" );
        Serial.print(waterFlow, 7);
        Serial.println("L");
        //Serial.print(temperature);
        //Serial.print(";");
        //Serial.print(phValue);
        //Serial.print(";");
        //Serial.print(humidity);
        //Serial.print(";");
        //Serial.print(ecValue);
        //Serial.print(";");
        //Serial.print(liquidLevel);
        //Serial.print(";");
        //if (co2Level == -1) {
        //Serial.print( "<400" );
        //} 
        //else {
        //Serial.print(co2Level);
        //}
        //Serial.print(";");
        //Serial.println(waterFlow);
    }
    if(phValue<5){
      digitalWrite(6, HIGH);//add base to decrease acidity
    }
    else if(phValue>7){
      digitalWrite(5, HIGH); // add acid to increase acidity
    }
    else{
      digitalWrite(5, LOW);//add nothing, ph in desired range.
      digitalWrite(6, LOW);
      
      // move on to EC

      if(ecValue < 1000){
        digitalWrite(7, HIGH);//add nutrient a
        digitalWrite(8, HIGH);//add nutrient b
      }
      else if(ecValue > 1300){
        digitalWrite(7, LOW);
        digitalWrite(8, LOW);
        Serial.println("Add water. Conductivity too high");
      }
      else{
        digitalWrite(7, LOW);
        digitalWrite(8, LOW);// in desired range for nec
      }
    }
    if(co2Level>1200){
      digitalWrite(10, HIGH);//turn on intake fan
    }
    else{
      digitalWrite(10, LOW);
    }
    if(temperature>32){
      digitalWrite(11, HIGH);// turn on exaust fan
    }
    else{
      digitalWrite(11, LOW);
    }
    if (counter <= 24)
  		{
        counter++;
        delay(1000);
  		}
  else
  		{
    counter = 0;
  		}
  
  if ( counter <= 12)
  		{
    digitalWrite(9, HIGH);
 		}
  else
  		{ 
    digitalWrite(9, LOW);
  		}

}

float MGRead(int mg_pin)
{
    int i;
    float v=0;

    for (i=0;i<READ_SAMPLE_TIMES;i++) {
        v += analogRead(mg_pin);
        delay(READ_SAMPLE_INTERVAL);
    }
    v = (v/READ_SAMPLE_TIMES) *5/1024 ;
    return v;
}

int  MGGetPercentage(float volts, float *pcurve)
{
   if ((volts/DC_GAIN )>=ZERO_POINT_VOLTAGE) {
      return -1;
   } else {
      return pow(10, ((volts/DC_GAIN)-pcurve[1])/pcurve[2]+pcurve[0]);
   }
}

void pulse()   //measure the quantity of square wave
{
  waterFlow += 1.0 / 450.0; // 450 pulses for 1 liter 
}
