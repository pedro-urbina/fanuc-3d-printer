// Pedro Urbina
// ROBO 497 - Capstone Senior Project
// Summer 2019
// Advisor: Ravindra Thamma, Ph.D.

// EXTRUDER **********
#include <Stepper.h>
const int stepsPerRevolution = 200;
const int enApin = 12;
const int enBpin = 13;
int enAstate = 0;
int enBstate = 0;
// initialize the stepper library:
Stepper myStepper(stepsPerRevolution, 5, 4, 3, 2);
int stepCount = 0;         // number of steps the motor has taken
int extrudeStepCount = 10;
float motorSpeed = 20; //rpm
int extrudeDelay = 1000;

const int enableButton = 9;     // the number of the pushbutton pin
const int ledPin =  10;      // the number of the LED pin
int enableState = LOW;         // the current state of the output pin
int enableButtonState;         // variable for reading the pushbutton status
int lastEnableButtonState = HIGH;   // the previous reading from the input pin
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 200;    // the debounce time; increase if the output flickers

const int extrudeButton = 7;     // the number of the pushbutton pin
int extrudeButtonState;         // variable for reading the pushbutton status
int extrudeState = LOW;         // the current state of the output pin
int lastExtrudeButtonState = HIGH;   // the previous reading from the input pin

const int retractButton = 8;     // the number of the pushbutton pin
int retractButtonState = 0;         // variable for reading the pushbutton status

// HOT END **********
// which analog pin to connect
#define HOTEND_THERMISTORPIN A0         
// resistance at 25 degrees C
#define HOTEND_THERMISTORNOMINAL 102900      
// temp. for nominal resistance (almost always 25 C)
#define HOTEND_TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define HOTEND_NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define HOTEND_BCOEFFICIENT 3950
// the value of the 'other' resistor
#define HOTEND_SERIESRESISTOR 99100    
int hotend_samples[HOTEND_NUMSAMPLES];

unsigned int hotEndTargetTemp = 260;
unsigned int minHotEndTemp = 200;
unsigned int maxHotEndTemp = 280;
const int hotEndRelayPin = 6;
unsigned int hotEndTemp;

// HEATED BUILD PLATE **********
// which analog pin to connect
#define PLATE_THERMISTORPIN A1
// resistance at 25 degrees C
#define PLATE_THERMISTORNOMINAL 100000      
// temp. for nominal resistance (almost always 25 C)
#define PLATE_TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define PLATE_NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define PLATE_BCOEFFICIENT 3950
// the value of the 'other' resistor
#define PLATE_SERIESRESISTOR 100870
int plate_samples[PLATE_NUMSAMPLES];

unsigned int minPlateTemp = 60; //(value of the reading of your thermistor which is the lower bound of your desired range)
unsigned int maxPlateTemp = 65; //(value of the reading of your thermistor which is the higher bound of your desired range)
const int plateRelayPin = 11;
unsigned int plateTemp;

void setup() {
  
  Serial.begin(9600);

  // EXTRUDER **********
  pinMode(enApin,OUTPUT);
  pinMode(enBpin,OUTPUT);
  pinMode(enableButton, INPUT);
  pinMode(extrudeButton, INPUT);
  pinMode(retractButton, INPUT);
  pinMode(ledPin, OUTPUT);
  myStepper.setSpeed(motorSpeed);
  
  // THERMISTOR VOLTAGE REFERENCE **********
  analogReference(EXTERNAL);
  
  // HOT END **********
  pinMode(hotEndRelayPin,OUTPUT);
  digitalWrite(hotEndRelayPin,HIGH);

  // HEATED BUILD PLATE **********
  pinMode(plateRelayPin,OUTPUT);
  digitalWrite(plateRelayPin,HIGH);
}

void loop() {
  
  enable();
  
  if (enableState == HIGH) {    
    hotend_thermistorReading();
    if(hotEndTemp > hotEndTargetTemp) {
        hotEndOff();
      }
      else if(hotEndTemp < hotEndTargetTemp) {
        hotEndOn();
      }

      plate_thermistorReading();
      if(plateTemp > maxPlateTemp) {
        plateOff();
      }
      else if(plateTemp < minPlateTemp) {
        plateOn();
      }
      
    if(hotEndTemp > minHotEndTemp && hotEndTemp < maxHotEndTemp && plateTemp > 55 && plateTemp < 70) {
      extrudeButtonFunction();
      retractButtonState = digitalRead(retractButton);
      if(extrudeState == HIGH) {
        extrude();
        delay(extrudeDelay);
      }
      else if(retractButtonState == HIGH) {
        retract();
      }
    }    
  }
  else {
    hotEndOff();
    plateOff();
    extrudeState = LOW;
    disableMotor();
  }
}

void enableMotor() {
  digitalWrite(enApin,HIGH);
  digitalWrite(enBpin,HIGH);
}

void disableMotor() {
  digitalWrite(enApin,LOW);
  digitalWrite(enBpin,LOW);
}

void hotEndOn () {
  digitalWrite(hotEndRelayPin,LOW);
}

void hotEndOff () {
  digitalWrite(hotEndRelayPin,HIGH);
}

void plateOn () {
  digitalWrite(plateRelayPin,LOW);
}

void plateOff () {
  digitalWrite(plateRelayPin,HIGH);
}

void extrude() {
  enableMotor();
  myStepper.step(-extrudeStepCount);
  disableMotor();
}

void retract() {
  enableMotor();
  myStepper.step(extrudeStepCount);
  disableMotor();
}

void enable() {
  int reading = digitalRead(enableButton);
  if (reading == HIGH && lastEnableButtonState == LOW && millis() - lastDebounceTime > debounceDelay) {
    if (enableState == HIGH)
      enableState = LOW;
    else
      enableState = HIGH;

    lastDebounceTime = millis();    
  }
  digitalWrite(ledPin, enableState);
  lastEnableButtonState = reading;
}

void extrudeButtonFunction() {
  int reading = digitalRead(extrudeButton);
  if (reading == HIGH && lastExtrudeButtonState == LOW && millis() - lastDebounceTime > debounceDelay) {
    if (extrudeState == HIGH)
      extrudeState = LOW;
    else
      extrudeState = HIGH;

    lastDebounceTime = millis();    
  }
  lastExtrudeButtonState = reading;
}

void hotend_thermistorReading() {
  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i=0; i< HOTEND_NUMSAMPLES; i++) {
   hotend_samples[i] = analogRead(HOTEND_THERMISTORPIN);
   delay(10);
  }
  
  // average all the samples out
  average = 0;
  for (i=0; i< HOTEND_NUMSAMPLES; i++) {
     average += hotend_samples[i];
  }
  average /= HOTEND_NUMSAMPLES;

  //Serial.print("Average analog reading "); 
  //Serial.println(average);
  
  // convert the value to resistance
  average = 1023 / average - 1;
  average = HOTEND_SERIESRESISTOR / average;
  //Serial.print("Thermistor resistance "); 
  //Serial.println(average);

  float steinhart;
  steinhart = average / HOTEND_THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= HOTEND_BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (HOTEND_TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C

  hotEndTemp = steinhart;

  Serial.print("Hot End Temperature "); 
  Serial.print(hotEndTemp);
  Serial.print(" *C");
}

void plate_thermistorReading() {
  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i=0; i< PLATE_NUMSAMPLES; i++) {
   plate_samples[i] = analogRead(PLATE_THERMISTORPIN);
   delay(10);
  }
  
  // average all the samples out
  average = 0;
  for (i=0; i< PLATE_NUMSAMPLES; i++) {
     average += plate_samples[i];
  }
  average /= PLATE_NUMSAMPLES;

  //Serial.print("Average analog reading "); 
  //Serial.println(average);
  
  // convert the value to resistance
  average = 1023 / average - 1;
  average = PLATE_SERIESRESISTOR / average;
  //Serial.print("Thermistor resistance "); 
  //Serial.println(average);

  float steinhart;
  steinhart = average / PLATE_THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= PLATE_BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (PLATE_TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C

  plateTemp = steinhart;
  
  Serial.print("\tPlate Temperature "); 
  Serial.print(plateTemp);
  Serial.println(" *C");
}
