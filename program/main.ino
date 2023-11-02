#include <AutoPID.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

//pins
#define OUTPUT_PIN 5
int lm35= A1;

#define TEMP_READ_DELAY 800 //can only read digital temp sensor every ~750ms

//pid settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 76
#define KI 0
#define KD 95


double temperature, setPoint, outputVal, suhu;

//input/output variables passed by reference, so they are updated automatically
AutoPID myPID(&temperature, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long lastTempUpdate; //tracks clock time of last temp update

//call repeatedly in loop, only updates after a certain time interval
//returns true if update happened
bool updateTemperature() {
  if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
    temperature = analogRead(lm35); //get temp reading
     Serial.print((setPoint*500)/1023);
     Serial.print(" ");
     Serial.println((temperature*500)/1023);
    lastTempUpdate = millis();

    return true;
  }
  return false;
}//void updateTemperature


void setup() {
  Serial.begin(9600);
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(lm35, INPUT);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  

  while (!updateTemperature()) {} //wait until temp sensor updated

  //if temperature is more than 4 degrees below or above setpoint, OUTPUT will be set to min or max respectively
  myPID.setBangBang(4);
  //set PID update interval to 4000ms
  myPID.setTimeStep(4000);

}//void setup


void loop() {
  updateTemperature();
  setPoint = 113;
  myPID.run(); //call every loop, updates automatically at certain time interval
  analogWrite(OUTPUT_PIN, outputVal);
  lcd.setCursor(0,0);
  lcd.print("Set Point: ");
  lcd.print((setPoint*500)/1023);
  lcd.setCursor(0,1);
  lcd.print("Suhu: ");
  lcd.print((temperature*500)/1023);

}//void loop
